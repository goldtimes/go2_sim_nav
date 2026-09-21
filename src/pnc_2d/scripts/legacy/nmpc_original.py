import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

DT = 0.05
N = 20
NX = 3
NU = 2

Q = np.diag([25.0, 25.0, 8.0])
R = np.diag([0.01, 0.01])
Qf = np.diag([60.0, 60.0, 15.0])

V_MAX = 1.2
V_MIN = 0.0
W_MAX = 1.2
W_MIN = -1.2

SIM_STEP = 400

def generate_circle_traj(num, r=3.0):
    t = np.linspace(0, 2*np.pi*2, num)
    x_ref = np.zeros((NX, num))
    x_ref[0, :] = r * np.cos(t)
    x_ref[1, :] = r * np.sin(t)
    x_ref[2, :] = t + np.pi/2.0
    v_ref = np.full(num, 1.0)
    w_ref = np.full(num, 1.0 / r)
    return x_ref, v_ref, w_ref

def nmpc_solve(x_curr, x_ref_block, v_ref_block, w_ref_block):
    opti = ca.Opti()
    X = opti.variable(NX, N+1)
    U = opti.variable(NU, N)

    opti.subject_to(X[:,0] == x_curr)

    cost = 0
    for k in range(N):
        xk = X[:,k]
        uk = U[:,k]
        xrefk = x_ref_block[:,k]
        vk_r = v_ref_block[k]
        wk_r = w_ref_block[k]
        u_ref_k = ca.vertcat(vk_r, wk_r)
        cost += (xk - xrefk).T @ Q @ (xk - xrefk) + (uk - u_ref_k).T @ R @ (uk - u_ref_k)

        x_next = xk + DT * ca.vertcat(
            uk[0]*ca.cos(xk[2]),
            uk[0]*ca.sin(xk[2]),
            uk[1]
        )
        opti.subject_to(X[:,k+1] == x_next)

    x_term = X[:,N]
    x_term_ref = x_ref_block[:,-1]
    cost += (x_term - x_term_ref).T @ Qf @ (x_term - x_term_ref)

    opti.minimize(cost)

    for k in range(N):
        opti.subject_to(U[0,k] >= V_MIN)
        opti.subject_to(U[0,k] <= V_MAX)
        opti.subject_to(U[1,k] >= W_MIN)
        opti.subject_to(U[1,k] <= W_MAX)

    opts = {
        "ipopt.print_level":0,
        "print_time":0,
        "ipopt.max_iter":100
    }
    opti.solver("ipopt", opts)

    try:
        sol = opti.solve()
        u_sol = sol.value(U)
        u0 = np.array(u_sol[:,0])
        return u0
    except RuntimeError:
        return np.array([1.0, 1.0/3.0])


def main():
    x_ref_all, v_ref_all, w_ref_all = generate_circle_traj(SIM_STEP, r=3.0)
    x_robot = np.array([3.0, 0.0, np.pi/2.0])

    log_x = []
    log_y = []

    for step in range(SIM_STEP):
        idx_end = min(step + N, SIM_STEP)
        need_n = idx_end - step
        x_ref_block = x_ref_all[:, step:idx_end]
        v_ref_block = v_ref_all[step:idx_end]
        w_ref_block = w_ref_all[step:idx_end]

        if need_n < N:
            pad_x = np.tile(x_ref_all[:,[-1]], N-need_n)
            pad_v = np.full(N-need_n, v_ref_all[-1])
            pad_w = np.full(N-need_n, w_ref_all[-1])
            x_ref_block = np.hstack([x_ref_block, pad_x])
            v_ref_block = np.hstack([v_ref_block, pad_v])
            w_ref_block = np.hstack([w_ref_block, pad_w])

        u_opt = nmpc_solve(x_robot, x_ref_block, v_ref_block, w_ref_block)

        v,w = u_opt
        theta = x_robot[2]
        dx = v*np.cos(theta)
        dy = v*np.sin(theta)
        dtheta = w
        x_robot = x_robot + DT*np.array([dx, dy, dtheta])

        log_x.append(x_robot[0])
        log_y.append(x_robot[1])

    plt.figure(figsize=(7,7))
    plt.plot(x_ref_all[0,:], x_ref_all[1,:], 'r--', label="reference")
    plt.plot(log_x, log_y, 'b-', label="NMPC + soft terminal cost")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("NMPC soft terminal cost")
    plt.show()

if __name__ == "__main__":
    main()
