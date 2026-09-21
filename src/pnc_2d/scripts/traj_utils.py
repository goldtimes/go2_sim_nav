"""轨迹跟踪公共工具：参考轨迹生成、进度投影、误差评价与绘图。

`mpc.py` / `nmpc.py` 共用本模块，保证两个控制器的**参考轨迹、跟踪目标、
评价口径完全一致**，否则两者无法公平对比。

三个关键约定
------------
1. **参考轨迹按弧长采样**：相邻采样点距离 = ``v_ref * dt``，采样序号 k 严格
   对应时刻 ``k*dt``。原脚本用 ``np.linspace(0, 4*pi, num)`` 等参数采样，隐含
   速度 = Δs/dt，与 v_ref 无关：400 点 2 圈 r=3 时隐含 1.89 m/s，而 V_MAX =
   1.2 m/s，机器人**物理上追不上参考点**，任何控制器都会表现为"跟踪很差"。
2. **跟踪目标由最近点投影给出**（进度变量，带搜索窗口）：参考点不会跑到机器人
   前方越来越远。原脚本直接用循环计数索引参考轨迹，隐含"每拍恰好走 v_ref*dt"
   的假设，一旦落后参考点就永不回同步，误差无界增长。
3. **评价指标与控制器无关**：横向误差取"到参考路径的**最小距离**"，不使用参考
   索引，因此不会把"索引落后"误算成横向误差。
"""

import numpy as np


def wrap_angle(a):
    """角度归一化到 [-pi, pi)，标量/数组通用。"""
    return np.arctan2(np.sin(a), np.cos(a))


def integrate_plant(x, u, dt, exact=True):
    """差速机器人运动学积分（被控对象仿真）。

    exact=True  : 中点法。步内 v、w 恒定时对圆弧运动是**精确**解。
    exact=False : 显式欧拉（原脚本用法）。dt=0.05、v=1、w=1/3 时每圈会凭空
                  向外多走 ~0.16 m，再好的控制器也会被判成跟踪差。
    """
    v, w = float(u[0]), float(u[1])
    th = float(x[2])
    if exact:
        th_mid = th + 0.5 * w * dt
        return np.array([x[0] + v * dt * np.cos(th_mid),
                         x[1] + v * dt * np.sin(th_mid),
                         th + w * dt])
    return np.array([x[0] + v * dt * np.cos(th),
                     x[1] + v * dt * np.sin(th),
                     th + w * dt])


class Reference:
    """按弧长参数化的参考路径 + 速度曲线。

    属性
    ----
    xy     : (2, num) 路径点（世界系）
    th     : (num,)   期望航向（已归一化到 [-pi, pi)）
    v, w   : (num,)   参考线速度 / 角速度
    s      : (num,)   各采样点对应的弧长
    ds     : 采样间隔（= v*dt）
    length : 路径总长
    dt     : 采样时间间隔
    closed : 是否闭合路径（闭合时参考索引取模，可绕任意圈）
    """

    def __init__(self, xy, th, v, w, dt, closed=False):
        self.xy = np.asarray(xy, dtype=float)
        self.th = wrap_angle(np.asarray(th, dtype=float))
        self.v = np.asarray(v, dtype=float)
        self.w = np.asarray(w, dtype=float)
        self.dt = float(dt)
        self.closed = bool(closed)
        self.num = int(self.xy.shape[1])
        seg = np.hypot(np.diff(self.xy[0]), np.diff(self.xy[1])) if self.num > 1 else np.zeros(1)
        self.ds = float(np.mean(seg))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = self.ds * self.num if self.closed else float(self.s[-1])

    # ---------- 索引 ----------
    def idx(self, k):
        """把（可能带绕圈计数的）进度索引映射到 [0, num)。"""
        k = np.asarray(k, dtype=int)
        return np.mod(k, self.num) if self.closed else np.clip(k, 0, self.num - 1)

    def state(self, k):
        """第 k 个参考位姿 [x, y, theta]。"""
        i = int(self.idx(k))
        return np.array([self.xy[0, i], self.xy[1, i], self.th[i]])

    def block(self, k0, n):
        """未来 n 步参考：返回 (X(3,n), V(n), W(n))。"""
        i = self.idx(int(k0) + np.arange(int(n)))
        return (np.vstack([self.xy[0, i], self.xy[1, i], self.th[i]]),
                self.v[i], self.w[i])

    # ---------- 常用参考 ----------
    @classmethod
    def circle(cls, vr=1.0, r=3.0, dt=0.05, laps=2.0):
        """圆形参考轨迹（按弧长采样 ⇒ 隐含速度严格等于 vr，与 dt 时间一致）。"""
        laps = float(laps)
        sgn = 1.0 if laps >= 0 else -1.0
        ds = abs(float(vr)) * float(dt)
        num = max(int(round(2 * np.pi * r * abs(laps) / ds)), 3)
        phi = sgn * (np.arange(num) * ds) / r
        xy = np.vstack([r * np.cos(phi), r * np.sin(phi)])
        th = phi + sgn * np.pi / 2.0
        return cls(xy, th, np.full(num, float(vr)), np.full(num, sgn * float(vr) / r),
                   dt, closed=abs(laps - round(laps)) < 1e-9)


def project_progress(x, y, ref, k_prog, back=10, fwd=60):
    """在参考路径上找距 (x, y) 最近的点，返回 (新进度索引, 该点距离)。

    只在 ``[k_prog-back, k_prog+fwd]`` 窗口内搜索：

    * 窗口限制单次跳变 —— 路径自交时参考点不会跳到另一条分支上；
    * ``fwd`` 提供足够的前向捕捉范围（应 >= 预测步长 N）；
    * ``back`` 允许少量回退 —— 机器人落后时参考点会停在机器人附近，
      不会"跑掉"，纵向滞后由控制器自然消除。
    """
    offs = np.arange(-int(back), int(fwd) + 1)
    i = ref.idx(int(k_prog) + offs)
    d = np.hypot(ref.xy[0, i] - x, ref.xy[1, i] - y)
    j = int(np.argmin(d))
    k = int(k_prog) + int(offs[j])
    if not ref.closed:
        k = int(np.clip(k, 0, ref.num - 1))
    return k, float(d[j])


def tracking_metrics(xy_hist, th_hist, u_hist, ref, label, u_lim=None, n_fail=0):
    """统计跟踪指标并打印，返回指标字典（供脚本间对比使用）。

    横向误差 = 到参考路径的**最小距离**（与参考索引无关，避免把索引落后误算成误差）。
    航向误差 = 机器人航向 与 **最近点处的路径切向** 之差（含角度包裹）。
    """
    xy = np.atleast_2d(np.asarray(xy_hist, dtype=float))
    th = np.asarray(th_hist, dtype=float).ravel()
    u = np.atleast_2d(np.asarray(u_hist, dtype=float))
    nq = xy.shape[0]

    d2 = (ref.xy[0][None, :] - xy[:, 0, None]) ** 2 + (ref.xy[1][None, :] - xy[:, 1, None]) ** 2
    k_near = np.argmin(d2, axis=1)
    ct = np.sqrt(d2[np.arange(nq), k_near])          # 横向误差
    eth = wrap_angle(th - ref.th[k_near])            # 航向误差

    # 实际路程用机器人自身轨迹累加，避免闭合/多圈路径上"最近点"落在另一圈的歧义
    traveled = float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))
    total_t = nq * ref.dt
    half = nq // 2

    print(f"\n--- {label}")
    print(f"  横向误差 (到路径最近距离) : RMS {ct.mean():.4f} m | max {ct.max():.4f} m "
          f"| 后半程 RMS {ct[half:].mean():.4f} m")
    print(f"  航向误差 (对路径切向)     : RMS {np.abs(eth).mean():.4f} rad | max {np.abs(eth).max():.4f} rad"
          f" | 后半程 RMS {np.abs(eth[half:]).mean():.4f} rad")
    print(f"  实际路程 {traveled:.2f} m / 路径长 {ref.length:.2f} m "
          f"(平均 {traveled / total_t:.3f} m/s, 参考 {ref.v.mean():.3f} m/s)")
    print(f"  指令 v [{u[:, 0].min():.3f}, {u[:, 0].max():.3f}] 均值 {u[:, 0].mean():.3f}"
          f" | w [{u[:, 1].min():.3f}, {u[:, 1].max():.3f}] 均值 {u[:, 1].mean():.3f}")
    if u_lim is not None:
        lo = np.array([u_lim[0], u_lim[1]], dtype=float)
        hi = np.array([u_lim[2], u_lim[3]], dtype=float)
        hit = np.mean(np.any((u <= lo + 1e-4) | (u >= hi - 1e-4), axis=1))
        print(f"  指令触及限幅的比例        : {hit * 100:.1f}%")
    print(f"  求解失败                  : {n_fail} / {nq} 步")

    return dict(label=label, ct_rms=float(ct.mean()), ct_max=float(ct.max()),
                ct_half=float(ct[half:].mean()), eth_rms=float(np.abs(eth).mean()),
                eth_max=float(np.abs(eth).max()), eth_half=float(np.abs(eth[half:]).mean()),
                traveled=traveled, n_fail=int(n_fail), n=nq,
                ct=ct, eth=eth, u=u)


def plot_result(ref, xy_hist, m, title, save=None, show=True):
    """画三张图：轨迹对比、误差-时间、指令-时间。

    图内文字用英文，避免无 CJK 字体环境下出现方框（控制台输出仍是中文）。
    """
    import os
    import matplotlib
    if save and not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")          # 无界面环境保存图片
    import matplotlib.pyplot as plt

    xy = np.asarray(xy_hist, dtype=float)
    t = np.arange(xy.shape[0]) * ref.dt

    fig, ax = plt.subplots(1, 3, figsize=(17, 5))

    ax[0].plot(ref.xy[0], ref.xy[1], 'r--', lw=1.2, label='reference')
    ax[0].plot(xy[:, 0], xy[:, 1], 'b-', lw=1.4, label=title)
    ax[0].plot(xy[0, 0], xy[0, 1], 'go', ms=6, label='start')
    ax[0].set_aspect('equal')
    ax[0].grid(True)
    ax[0].legend(fontsize=9)
    ax[0].set_title('trajectory (xy)')
    ax[0].set_xlabel('x [m]')
    ax[0].set_ylabel('y [m]')

    ax[1].plot(t, m['ct'], 'b', lw=1.2, label='cross-track [m]')
    ax[1].plot(t, np.abs(m['eth']), 'g', lw=1.2, label='heading [rad]')
    ax[1].set_xlabel('t [s]')
    ax[1].grid(True)
    ax[1].legend(fontsize=9)
    ax[1].set_title(f"error vs time (cross-track RMS {m['ct_rms']:.4f} m)")

    ax[2].plot(t, m['u'][:, 0], 'b', lw=1.2, label='v [m/s]')
    ax[2].plot(t, m['u'][:, 1], 'r', lw=1.2, label='w [rad/s]')
    ax[2].set_xlabel('t [s]')
    ax[2].grid(True)
    ax[2].legend(fontsize=9)
    ax[2].set_title('control commands')

    fig.suptitle(title)
    plt.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
        print(f"  图像已保存: {save}")
    if show:
        plt.show()
    return fig
