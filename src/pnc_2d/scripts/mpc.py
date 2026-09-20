import numpy as np
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

# ========= 参数 =========
DT = 0.05
N = 15
NX = 3
NU = 2

# 误差状态权重，惩罚跟踪误差e_x,e_y,e_theta
Q = np.diag([20.0, 20.0, 8.0])
R = np.diag([0.05, 0.05])
Qf = np.diag([40.0, 40.0, 12.0])

V_MAX = 1.2
V_MIN = 0.0
W_MAX = 1.2
W_MIN = -1.2

SIM_STEP = 400

def differential_kinematic(x, u):
    v, w = u
    theta = x[2]
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = w
    return np.array([dx, dy, dtheta])

def calc_error_dynamics(x_robot, x_ref):
    """
    计算机器人局部坐标系下跟踪误差 e = [ex, ey, etheta]
    x_robot: [x,y,theta] 机器人世界坐标
    x_ref: [xr,yr,thetar] 参考点世界坐标
    return e
    """
    xr, yr, thetar = x_ref
    x, y, theta = x_robot
    dx = xr - x
    dy = yr - y
    ex =  dx * np.cos(theta) + dy * np.sin(theta)
    ey = -dx * np.sin(theta) + dy * np.cos(theta)
    etheta = thetar - theta
    etheta = np.arctan2(np.sin(etheta), np.cos(etheta))
    return np.array([ex, ey, etheta])

def linearize_error_model(vr, wr, dt):
    """
    在e≈0处线性化误差动力学，离散 A,B
    vr：参考线速度，wr参考角速度
    """
    A_cont = np.array([
        [0, wr, 0],
        [-wr, 0, vr],
        [0, 0, 0]
    ])
    B_cont = np.array([
        [-1, 0],
        [0, 0],
        [0, -1]
    ])
    A_d = np.eye(NX) + A_cont * dt
    B_d = B_cont * dt
    return A_d, B_d

def generate_circle_traj(num, r=3.0):
    t = np.linspace(0, 2*np.pi*2, num)
    x_ref = np.zeros((NX, num))
    x_ref[0, :] = r * np.cos(t)
    x_ref[1, :] = r * np.sin(t)
    x_ref[2, :] = t + np.pi/2.0
    # 参考速度：圆形轨迹 vr, wr
    vr = np.full(num, 1.0)
    wr = np.full(num, 1.0 / r)
    return x_ref, vr, wr

def build_qp(A,B, e_curr, vr_seq, wr_seq, u_prev):
    nz = N*NU
    Sx = np.zeros((NX*N, NX))
    Su = np.zeros((NX*N, nz))
    for k in range(N):
        Ak = np.linalg.matrix_power(A,k)
        Sx[k*NX:(k+1)*NX,:] = Ak
        for i in range(k):
            Aki = np.linalg.matrix_power(A, k-1-i)
            Su[k*NX:(k+1)*NX, i*NU:(i+1)*NU] = Aki @ B

    Q_bar = np.kron(np.eye(N), Q)
    R_bar = np.kron(np.eye(N), R)

    P = Su.T @ Q_bar @ Su + R_bar
    q = Su.T @ Q_bar @ (Sx @ e_curr)

    #终端代价
    idx_term = (N-1)*NX
    Su_term = Su[idx_term:N*NX, :]
    Sx_term = Sx[idx_term:N*NX, :]
    P += Su_term.T @ Qf @ Su_term
    q += Su_term.T @ Qf @ (Sx_term @ e_curr)

    u_min = np.array([V_MIN, W_MIN])
    u_max = np.array([V_MAX, W_MAX])
    l_list = []
    u_list = []
    for _ in range(N):
        l_list.extend(u_min.tolist())
        u_list.extend(u_max.tolist())
    l = np.array(l_list)
    u = np.array(u_list)

    P_sp = sparse.csc_matrix(P)
    A_sp = sparse.eye(nz, format="csc")
    prob = osqp.OSQP()
    prob.setup(P=P_sp, q=q, A=A_sp, l=l, u=u, verbose=False)
    res = prob.solve()
    if res.info.status != "solved":
        print(f"solver fail {res.info.status}")
        return u_prev
    u0 = res.x[0:NU]
    return u0

def main():
    x_ref_all, vr_all, wr_all = generate_circle_traj(SIM_STEP, r=3.0)
    x_robot = np.array([3.0, 0.0, np.pi/2.0])
    u_last = np.array([1.0, 1.0/3.0])

    log_x = []
    log_y = []

    for step in range(SIM_STEP):
        idx_end = min(step + N, SIM_STEP)
        need_n = idx_end - step
        #取未来N步参考
        vr_block = vr_all[step:idx_end]
        wr_block = wr_all[step:idx_end]
        if need_n < N:
            vr_block = np.hstack([vr_block, np.full(N-need_n, vr_all[-1])])
            wr_block = np.hstack([wr_block, np.full(N-need_n, wr_all[-1])])

        x_ref_k = x_ref_all[:, step]
        e_curr = calc_error_dynamics(x_robot, x_ref_k)
        vr0 = vr_block[0]
        wr0 = wr_block[0]
        A_d, B_d = linearize_error_model(vr0, wr0, DT)
        u_opt = build_qp(A_d, B_d, e_curr, vr_block, wr_block, u_last)

        dx = differential_kinematic(x_robot, u_opt)
        x_robot = x_robot + dx * DT
        u_last = u_opt

        log_x.append(x_robot[0])
        log_y.append(x_robot[1])

    plt.figure(figsize=(7,7))
    plt.plot(x_ref_all[0,:], x_ref_all[1,:], 'r--', label="reference")
    plt.plot(log_x, log_y, 'b-', label="mpc(error dynamics)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("Error‑Dynamics L‑MPC")
    plt.show()

if __name__ == "__main__":
    main()
