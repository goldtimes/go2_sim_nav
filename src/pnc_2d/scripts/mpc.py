"""误差动力学 L-MPC（OSQP）—— 差速机器人轨迹跟踪。

算法推导 / 调参 / 踩坑清单见 doc/mpc_nmpc_guide.md。

原版为什么跟踪效果很差（实测 r=3 m 的圆上横向误差 RMS 1.85 m / max 2.35 m）
===========================================================================
1. **丢了前馈项（决定性）**。误差模型在 e≈0 处线性化后是

       ė = A e + B (u - u_ref),   e = [ex, ey, etheta],  u = [v, w]

   原代码写成 ``ė = A e + B u``（把 u 当绝对量），等于把常数前馈项
   ``c = [v_ref, 0, w_ref]`` 丢掉了。后果非常严重：当 e = 0 时 QP 的梯度
   q = 0，问题退化为 ``min ũ'P ũ``，解出 ``u = [0, 0]`` —— **机器人原地不动**，
   而正确答案是 ``u = u_ref = [1.0, 0.333]``。机器人一走不动，参考点就永远跑在
   前面，误差再也不收敛。
   验证：``e=0`` 时原版输出 ``[0., 0.]``，本版输出 ``[1.0, 0.333]``。

2. **参考点会跑掉**。原代码用循环计数 ``step`` 直接索引参考轨迹，隐含假设
   "机器人每一拍都恰好走 v_ref*dt"。一旦落后，参考点越跑越远且永不回同步，
   误差无界增长。本版用最近点投影（进度索引、窗口受限）确定参考点。

3. **采样与 dt 不一致**。原 ``generate_circle_traj`` 用 ``linspace(0, 4π, 400)``
   采样，隐含速度 = Δs/dt = 1.89 m/s，而 ``V_MAX = 1.2 m/s``，机器人物理上
   追不上参考点。本版按弧长采样（间隔 = v_ref*dt），隐含速度严格等于 v_ref。

4. **被控对象积分误差**。原代码 plant 用显式欧拉：dt=0.05、v=1、w=1/3 时每圈
   凭空向外多走约 0.16 m，再好的控制器也会显示成"跟踪差"。本版用中点法
   （常速度下对圆弧**精确**）。

5. **终端代价重复加权**。原代码最后一级既保留 Q 又叠加 Qf（相当于 Q+Qf），
   本版最后一级只保留 Qf。

6. **R 权重被隐式放大 2 倍**。OSQP 的目标是 ``min 0.5 z'Pz + q'z``，原代码直接令
   ``P = Su'QSu + R``（少了因子 2，q 也少 2），整体缩放虽不影响最优解，但 R 相对
   Q 被放大了 2 倍。本版严格按 P = 2H、q = 2h 填写。

其余改进：逐级线性化（参考速度可随预测步变化）、Δu 平滑惩罚、限幅可行性保护
（参考速度超限幅时前馈自动限幅并告警）、完整评价指标。
"""

import argparse
import time

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm

import traj_utils as tu

# ===================== 参数 =====================
DT = 0.05          # 控制周期 [s]
N = 15             # 预测步长（15 * 0.05 = 0.75 s）
NX, NU = 3, 2

VR = 1.0           # 参考线速度 [m/s]
RADIUS = 3.0       # 圆半径 [m]
LAPS = 2.0         # 圈数

Q = np.diag([20.0, 20.0, 8.0])      # 跟踪误差权重 [ex, ey, etheta]
R = np.diag([0.05, 0.05])           # 偏差输入 ũ = u - u_ref 的权重
R_DU = np.diag([0.5, 0.5])          # 输入变化率 Δu 权重（平滑），None 表示关闭
Qf = np.diag([40.0, 40.0, 12.0])    # 终端误差权重

V_MAX, V_MIN = 1.2, 0.0
W_MAX, W_MIN = 1.2, -1.2

BACK, FWD = 10, 60   # 进度投影搜索窗口 [采样点]，FWD 应 >= N
LIM_LO = np.array([V_MIN, W_MIN])
LIM_HI = np.array([V_MAX, W_MAX])


def differential_kinematic(x, u):
    """连续运动学 x_dot = f(x, u)（仅用于说明/自检）。"""
    v, w = u
    theta = x[2]
    return np.array([v * np.cos(theta), v * np.sin(theta), w])


def calc_error_dynamics(x_robot, x_ref):
    """机器人坐标系下的跟踪误差 e = [ex, ey, etheta]。

    ex     : 参考点在机器人前方为正（纵向误差）
    ey     : 参考点在机器人左侧为正（横向误差）
    etheta : 航向误差，已包裹到 [-pi, pi)
    """
    xr, yr, thetar = x_ref
    x, y, theta = x_robot
    dx = xr - x
    dy = yr - y
    ex = dx * np.cos(theta) + dy * np.sin(theta)
    ey = -dx * np.sin(theta) + dy * np.cos(theta)
    return np.array([ex, ey, tu.wrap_angle(thetar - theta)])


def linearize_error_model(vr, wr, dt, exact=True):
    """误差模型在 e≈0 处线性化 + 离散化（输入取**偏差** ũ = u - u_ref）。

        ėx     =  wr*ey + v_ref - v   =  wr*ey - ũ_v
        ėy     = -wr*ex + v_ref*etheta
        ėtheta =  w_ref - w           = -ũ_w

    写成 ė = A e + B ũ。注意这里是 ũ 而不是 u —— 原版把 u 当绝对量代入，
    等于丢掉前馈 [v_ref, 0, w_ref]，是跟踪差的**主因**。

    exact=True 用零阶保持精确离散化（expm），否则用欧拉（原版用法，误差次要）。
    """
    A_c = np.array([[0.0, wr, 0.0],
                    [-wr, 0.0, vr],
                    [0.0, 0.0, 0.0]])
    B_c = np.array([[-1.0, 0.0],
                    [0.0, 0.0],
                    [0.0, -1.0]])
    if not exact:
        return np.eye(NX) + A_c * dt, B_c * dt
    M = np.zeros((NX + NU, NX + NU))
    M[:NX, :NX] = A_c
    M[:NX, NX:] = B_c
    E = expm(M * dt)
    return E[:NX, :NX], E[:NX, NX:]


def build_prediction(A_seq, B_seq):
    """误差预测矩阵：e_k = Sx_k e_0 + Su_k ũ，k = 1..N（行块 k-1 对应 e_k）。

    A_seq / B_seq 是逐级的离散矩阵（长度为 N），因此支持参考速度随时间变化
    （原版只用第 0 拍的 vr/wr 线性化一次，全程套用）。
    """
    Sx = np.zeros((NX * N, NX))
    Su = np.zeros((NX * N, N * NU))
    Ak = np.eye(NX)
    Pk = np.zeros((NX, N * NU))
    for k in range(1, N + 1):
        A_pre, B_pre = A_seq[k - 1], B_seq[k - 1]
        Ak = A_pre @ Ak
        Pk = A_pre @ Pk
        Pk[:, (k - 1) * NU:k * NU] = B_pre
        Sx[(k - 1) * NX:k * NX, :] = Ak
        Su[(k - 1) * NX:k * NX, :] = Pk
    return Sx, Su


def _du_operator():
    """Δu_k = u_k - u_{k-1} = D z + c 中的常数矩阵 D（c 由参考和 u_prev 决定）。"""
    D = np.eye(N * NU)
    for k in range(1, N):
        D[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] = -np.eye(NU)
    return D


def build_qp(e_curr, u_ff_flat, A_seq, B_seq, u_prev, warn):
    """构造并求解 QP，返回 (下发指令 u0, 是否求解成功)。

    决策变量 z = [ũ_0, ..., ũ_{N-1}]，ũ_k = u_k - u_ff_k。
    目标： Σ e_k' Q e_k + e_N' Qf e_N + Σ ũ_k' R ũ_k + Σ Δu_k' R_DU Δu_k
    约束： u_min <= u_ff_k + ũ_k <= u_max
    """
    Sx, Su = build_prediction(A_seq, B_seq)

    # 状态权重：前 N-1 级 Q，最后一级 Qf（不再叠加 Q）
    Q_blk = np.zeros((NX * N, NX * N))
    for k in range(N):
        Q_blk[k * NX:(k + 1) * NX, k * NX:(k + 1) * NX] = Qf if k == N - 1 else Q

    H = Su.T @ Q_blk @ Su + np.kron(np.eye(N), R)
    h = Su.T @ Q_blk @ (Sx @ e_curr)

    # ---- 前馈限幅：保证 ũ = 0（纯前馈）永远可行 ----
    u_ff = np.clip(u_ff_flat, np.tile(LIM_LO, N), np.tile(LIM_HI, N))
    if not np.allclose(u_ff, u_ff_flat) and not warn["ref_over_limit"]:
        print("  [警告] 参考速度超出 v/w 限幅，前馈已限幅（机器人无法完全跟踪）")
        warn["ref_over_limit"] = True

    # ---- 输入变化率惩罚：Δu_k = u_k - u_{k-1} = D z + c ----
    if R_DU is not None:
        D = _du_operator()
        c = np.empty(N * NU)
        c[:NU] = u_ff[:NU] - u_prev
        for k in range(1, N):
            c[k * NU:(k + 1) * NU] = u_ff[k * NU:(k + 1) * NU] - u_ff[(k - 1) * NU:k * NU]
        RD = np.kron(np.eye(N), R_DU)
        H += D.T @ RD @ D
        h += D.T @ RD @ c

    # ---- 盒式约束（相对偏差输入）----
    lo = np.tile(LIM_LO, N) - u_ff
    hi = np.tile(LIM_HI, N) - u_ff

    # OSQP 求解 min 0.5 z'Pz + q'z
    P = sparse.csc_matrix(2.0 * H)
    q = 2.0 * h
    A_sp = sparse.eye(N * NU, format="csc")

    prob = osqp.OSQP()
    prob.setup(P=P, q=q, A=A_sp, l=lo, u=hi, verbose=False,
               eps_abs=1e-6, eps_rel=1e-6, max_iter=8000)
    res = prob.solve()
    if res.info.status == "solved":
        return u_ff[:NU] + res.x[:NU], True

    # 求解失败：保持上一拍指令（不要硬编码某个速度）
    print(f"  [警告] OSQP 求解失败: {res.info.status}")
    return u_prev, False


def main():
    ap = argparse.ArgumentParser(description="误差动力学 L-MPC 轨迹跟踪演示")
    ap.add_argument("--save", metavar="PNG", help="保存图像到文件（无界面环境）")
    ap.add_argument("--mode", choices=["progress", "time"], default="progress",
                    help="参考点索引方式：progress=最近点投影(推荐)，time=按时间索引")
    ap.add_argument("--no-show", action="store_true", help="不弹出窗口")
    args = ap.parse_args()

    ref = tu.Reference.circle(vr=VR, r=RADIUS, dt=DT, laps=LAPS)
    nstep = ref.num
    print("=" * 78)
    print(f"L-MPC (误差动力学) | 圆 r={RADIUS} m x{LAPS} 圈 | 参考 {VR} m/s | dt={DT} s")
    print(f"参考轨迹 {ref.num} 点, 采样间隔 {ref.ds:.4f} m = vr*dt, "
          f"隐含速度 {ref.ds / DT:.3f} m/s, 总时长 {nstep * DT:.1f} s")
    print(f"限幅 V[{V_MIN},{V_MAX}] W[{W_MIN},{W_MAX}] | 预测 {N} 步 | 索引 {args.mode}")
    print("=" * 78)

    x = ref.state(0).copy()
    u_prev = np.array([ref.v[0], ref.w[0]])
    k_prog = 0
    warn = {"ref_over_limit": False}
    n_fail = 0
    xy_log, th_log, u_log = [], [], []
    u_ff_flat = np.empty(N * NU)

    # 计时只覆盖控制循环本身（不含 import / 画图），与 nmpc.py 口径一致
    t0 = time.time()

    for step in range(nstep):
        # 1) 确定当前跟踪目标（参考点）
        if args.mode == "progress":
            k_prog, _ = tu.project_progress(x[0], x[1], ref, k_prog, back=BACK, fwd=FWD)
            k0 = k_prog
        else:
            k0 = step

        # 2) 未来 N 步参考
        X_r, V_r, W_r = ref.block(k0, N)
        u_ff_flat[0::2] = V_r
        u_ff_flat[1::2] = W_r

        # 3) 当前误差（相对跟踪目标）
        e_curr = calc_error_dynamics(x, X_r[:, 0])

        # 4) 逐级线性化（参考速度可随时间变化）
        A_seq, B_seq = [], []
        for k in range(N):
            A_k, B_k = linearize_error_model(V_r[k], W_r[k], DT)
            A_seq.append(A_k)
            B_seq.append(B_k)

        # 5) 求解 → 实际下发 u = u_ref + ũ
        u, ok = build_qp(e_curr, u_ff_flat, A_seq, B_seq, u_prev, warn)
        n_fail += 0 if ok else 1

        # 6) 被控对象（中点法精确积分），航向按估计器习惯包裹
        x = tu.integrate_plant(x, u, DT, exact=True)
        x[2] = tu.wrap_angle(x[2])

        xy_log.append(x[:2].copy())
        th_log.append(x[2])
        u_log.append(u.copy())
        u_prev = u

    elapsed = time.time() - t0
    print(f"\n控制循环总耗时 {elapsed:.2f} s ({elapsed / nstep * 1000:.2f} ms/步)"
          f"  | 单核占用 {elapsed / nstep / DT * 100:.1f}%")

    m = tu.tracking_metrics(np.array(xy_log), np.array(th_log), np.array(u_log), ref,
                            f"L-MPC 误差动力学 (前馈 + 进度投影, mode={args.mode})",
                            u_lim=(V_MIN, W_MIN, V_MAX, W_MAX), n_fail=n_fail)
    tu.plot_result(ref, np.array(xy_log), m,
                   "Error-Dynamics L-MPC (with feedforward)",
                   save=args.save, show=not (args.save or args.no_show))
    return m


if __name__ == "__main__":
    main()
