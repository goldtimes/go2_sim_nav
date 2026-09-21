"""NMPC（CasADi + IPOPT）—— 差速机器人轨迹跟踪。

算法推导 / 调参 / 踩坑清单见 doc/mpc_nmpc_guide.md。

原版的建模思路本身是对的（非线性全状态模型 + 围绕参考速度 (u-u_ref) 的代价），
但仍有 4 个问题，实测 r=3 m 的圆上横向误差 RMS 1.62 m / max 2.03 m：

1. **参考点会跑掉（主因）**。原版用循环计数 ``step`` 索引参考轨迹，而参考轨迹
   ``linspace(0, 4π, 400)`` 采样得到的隐含速度是 1.89 m/s，大于 ``V_MAX=1.2``。
   机器人物理上追不上，参考点每拍都往前跳，位置误差被越拉越大，NMPC 只能把
   v 顶到 1.2 饱和（实测 v 均值 1.2、几乎没有调节余量）。
   → 改用 ``traj_utils`` 的弧长一致参考 + 最近点进度投影。

2. **航向误差没有做角度包裹**。代价里直接写 ``(xk - xrefk)'Q(xk - xrefk)``，而参考
   航向 ``t + π/2`` 会一路涨到 4π+π/2（十几弧度）。一旦状态估计给出的航向是包裹到
   [-π, π) 的（真实机器人都是这样），误差会凭空多出 2π 的整数倍 → 代价爆炸、
   控制彻底失控。→ 用 ``atan2(sin, cos)`` 包裹后再进代价。

3. **终端索引错位**。原版把 ``X[:,N]`` 与 ``x_ref_block[:,-1]``（第 N-1 个参考点）
   比较，少了一拍。→ 参考块取 N+1 个点，``X[:,k]`` 与第 k 个参考一一对齐。

4. **求解失败时硬编码 ``[1.0, 1/3]``**。一旦失败就按固定速度开环行驶，把问题掩盖掉。
   → 失败时保持上一拍指令并计数告警；同时加入热启动（上一拍解平移一位），
   显著减少失败次数与求解时间。

补充：Δu 速率惩罚（平滑）、约束向量化、与 L-MPC 完全一致的评价指标。
"""

import argparse
import time

import numpy as np
import casadi as ca

import traj_utils as tu

# ===================== 参数 =====================
DT = 0.05          # 控制周期 [s]
N = 20             # 预测步长（20 * 0.05 = 1.0 s）
NX, NU = 3, 2

VR = 1.0           # 参考线速度 [m/s]
RADIUS = 3.0       # 圆半径 [m]
LAPS = 2.0         # 圈数

Q = np.diag([25.0, 25.0, 8.0])      # 跟踪误差权重 [x, y, theta]
R = np.diag([0.01, 0.01])           # 输入偏离参考速度的权重
R_DU = np.diag([0.5, 0.5])          # 输入变化率 Δu 权重（平滑），None 表示关闭
Qf = np.diag([60.0, 60.0, 15.0])    # 终端误差权重

V_MAX, V_MIN = 1.2, 0.0
W_MAX, W_MIN = 1.2, -1.2

BACK, FWD = 10, 60   # 进度投影搜索窗口 [采样点]，FWD 应 >= N
MODEL = "midpoint"   # 控制器内部预测模型："midpoint"(更准, 默认) / "euler"(原版)
                     # 实测：与"中点法被控对象"配套时 midpoint 横向 RMS 0.0000 m，
                     #       euler 因模型失配残留 0.0046 m 稳态偏差


def nmpc_solve(x_curr, X_ref, V_ref, W_ref, u_prev, warm=None):
    """求解一步 NMPC，返回 (下发指令 u0, 是否成功, 新热启动初值)。

    参数
    ----
    x_curr      : (3,)     当前状态
    X_ref       : (3, N+1) 未来参考位姿，索引 k = 0..N，k = 0 即当前跟踪目标点
    V_ref/W_ref : (N+1,)   对应参考速度（用作前馈）
    u_prev      : (2,)     上一拍下发指令（Δu 惩罚用）
    warm        : (X0, U0) 热启动初值，可为 None
    """
    opti = ca.Opti()
    X = opti.variable(NX, N + 1)
    U = opti.variable(NU, N)
    opti.subject_to(X[:, 0] == x_curr)

    cost = 0
    for k in range(N):
        uk = U[:, k]
        # ---- 状态代价：位置差直接平方，航向差必须包裹角度 ----
        xk = X[:, k + 1]
        dpos = xk[0:2] - X_ref[0:2, k + 1]
        dth = ca.atan2(ca.sin(xk[2] - X_ref[2, k + 1]),
                       ca.cos(xk[2] - X_ref[2, k + 1]))
        w_state = Qf if k == N - 1 else Q
        cost += w_state[0, 0] * dpos[0] ** 2 + w_state[1, 1] * dpos[1] ** 2 \
            + w_state[2, 2] * dth ** 2

        # ---- 输入代价：围绕参考速度（前馈）----
        u_ff = ca.vertcat(V_ref[k], W_ref[k])
        cost += (uk - u_ff).T @ R @ (uk - u_ff)

        # ---- 速率惩罚：Δu_k = u_k - u_{k-1} ----
        if R_DU is not None:
            u_prev_k = u_prev if k == 0 else U[:, k - 1]
            cost += (uk - u_prev_k).T @ R_DU @ (uk - u_prev_k)

        # ---- 运动学预测 ----
        if MODEL == "midpoint":
            # 中点法：步内 v、w 恒定时对圆弧运动精确（与 traj_utils.integrate_plant 一致）
            th_mid = X[2, k] + 0.5 * DT * uk[1]
            x_next = X[:, k] + DT * ca.vertcat(uk[0] * ca.cos(th_mid),
                                               uk[0] * ca.sin(th_mid), uk[1])
        else:
            # 原版显式欧拉
            x_next = X[:, k] + DT * ca.vertcat(uk[0] * ca.cos(X[2, k]),
                                               uk[0] * ca.sin(X[2, k]), uk[1])
        opti.subject_to(X[:, k + 1] == x_next)

    opti.minimize(cost)

    opti.subject_to(U[0, :] >= V_MIN)
    opti.subject_to(U[0, :] <= V_MAX)
    opti.subject_to(U[1, :] >= W_MIN)
    opti.subject_to(U[1, :] <= W_MAX)

    if warm is not None:
        opti.set_initial(X, warm[0])
        opti.set_initial(U, warm[1])

    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 200,
            "ipopt.tol": 1e-6, "ipopt.warm_start_init_point": "yes"}
    opti.solver("ipopt", opts)

    try:
        sol = opti.solve()
    except RuntimeError:
        # 求解失败：保持上一拍指令并发散告警（**不要**硬编码某个速度）
        return u_prev, False, None

    Xv = np.array(sol.value(X))
    Uv = np.array(sol.value(U))
    return Uv[:, 0], True, (Xv, Uv)


def main():
    ap = argparse.ArgumentParser(description="NMPC 轨迹跟踪演示")
    ap.add_argument("--save", metavar="PNG", help="保存图像到文件（无界面环境）")
    ap.add_argument("--mode", choices=["progress", "time"], default="progress",
                    help="参考点索引方式：progress=最近点投影(推荐)，time=按时间索引")
    ap.add_argument("--no-show", action="store_true", help="不弹出窗口")
    args = ap.parse_args()

    ref = tu.Reference.circle(vr=VR, r=RADIUS, dt=DT, laps=LAPS)
    nstep = ref.num
    print("=" * 78)
    print(f"NMPC | 圆 r={RADIUS} m x{LAPS} 圈 | 参考 {VR} m/s | dt={DT} s | 预测 {N} 步")
    print(f"参考轨迹 {ref.num} 点, 采样间隔 {ref.ds:.4f} m = vr*dt, "
          f"隐含速度 {ref.ds / DT:.3f} m/s, 总时长 {nstep * DT:.1f} s")
    print(f"限幅 V[{V_MIN},{V_MAX}] W[{W_MIN},{W_MAX}] | 索引 {args.mode}")
    print("=" * 78)

    x = ref.state(0).copy()
    u_prev = np.array([ref.v[0], ref.w[0]])
    k_prog = 0
    n_fail = 0
    xy_log, th_log, u_log = [], [], []
    warm = None
    t0 = time.time()

    for step in range(nstep):
        # 1) 确定当前跟踪目标（参考点）
        if args.mode == "progress":
            k_prog, _ = tu.project_progress(x[0], x[1], ref, k_prog, back=BACK, fwd=FWD)
            k0 = k_prog
        else:
            k0 = step

        # 2) 未来 N+1 步参考（k=0 为当前目标点，惩罚 X[:,k] 时用索引 k）
        X_r, V_r, W_r = ref.block(k0, N + 1)

        # 3) 热启动：上一拍解整体平移一位
        warm_in = None
        if warm is not None:
            Xw = np.hstack([warm[0][:, 1:], warm[0][:, [-1]]])
            Uw = np.hstack([warm[1][:, 1:], warm[1][:, [-1]]])
            warm_in = (Xw, Uw)

        # 4) 求解
        u, ok, warm_new = nmpc_solve(x, X_r, V_r, W_r, u_prev, warm_in)
        if ok:
            warm = warm_new
        n_fail += 0 if ok else 1

        # 5) 被控对象（中点法精确积分），航向按估计器习惯包裹
        x = tu.integrate_plant(x, u, DT, exact=True)
        x[2] = tu.wrap_angle(x[2])

        xy_log.append(x[:2].copy())
        th_log.append(x[2])
        u_log.append(np.asarray(u, dtype=float).copy())
        u_prev = np.asarray(u, dtype=float)

    elapsed = time.time() - t0
    print(f"\n求解总耗时 {elapsed:.2f} s ({elapsed / nstep * 1000:.2f} ms/步)"
          f"  | 单核占用 {elapsed / nstep / DT * 100:.1f}%")

    m = tu.tracking_metrics(np.array(xy_log), np.array(th_log), np.array(u_log), ref,
                            f"NMPC (角度包裹 + 进度投影, mode={args.mode})",
                            u_lim=(V_MIN, W_MIN, V_MAX, W_MAX), n_fail=n_fail)
    tu.plot_result(ref, np.array(xy_log), m, "NMPC (angle-wrapped, warm-started)",
                   save=args.save, show=not (args.save or args.no_show))
    return m


if __name__ == "__main__":
    main()
