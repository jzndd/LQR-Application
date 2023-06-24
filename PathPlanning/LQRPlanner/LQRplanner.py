"""

LQR local path planning

author: jzndd

"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
import math

show_animation = True

MAX_TIME = 100.0  # Maximum simulation time
DT = 0.1  # Time tick


def LQRplanning(sx, sy, gx, gy):

    rx, ry = [sx], [sy]

    error = []                                      # 误差
    u_array= []                                     # 控制律

    x = np.array([sx - gx, sy - gy]).reshape(2, 1)  # 状态空间

    # Linear system model
    A, B = get_system_model()

    found_path = False

    time = 0.0
    while time <= MAX_TIME:
        time += DT

        u = LQR_control(A, B, x)
        u_array.append(u)

        x = A @ x + B @ u            # 状态方程： x_{k+1} = A x_{k} + B * u_{k}

        rx.append(x[0, 0] + gx)
        ry.append(x[1, 0] + gy)

        d = math.sqrt((gx - rx[-1])**2 + (gy - ry[-1])**2)
        error.append(d)
        
        if d <= 0.1:
            #  print("Goal!!")
            found_path = True
            break

        # animation
        if show_animation:  # pragma: no cover
            plt.plot(sx, sy, "or")
            plt.plot(gx, gy, "ob")
            plt.plot(rx, ry, "-r")
            plt.axis("equal")
            plt.pause(1.0)

    if not found_path:
        print("Cannot found path")
        return [], []

    return rx, ry,error,u_array


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * \
            la.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):

    # first,solve ricatti equation
    X = solve_DARE(A, B, Q, R)

    # second, compute k
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals


def get_system_model():

    A = np.array([[DT, 1.0],
                  [0.0, DT]])
    B = np.array([0.0, 1.0]).reshape(2, 1)

    return A, B


def LQR_control(A, B, x):

    Kopt, X, ev = dlqr(A, B, np.eye(2), np.eye(1))

    u = -Kopt @ x

    return u


def main():
    print(__file__ + " start!!")

 
    area = 100.0  # sampling area

  
    sx = 0.0
    sy = 0.0
    gx = 6.0
    gy = 7.0
    # gx = random.uniform(-area, area)
    # gy = random.uniform(-area, area)

    rx, ry,error,u_array = LQRplanning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "ob")
        plt.plot(rx, ry, "-r")
        plt.axis("equal")
        plt.pause(1.0)

    print(error)
    print(u_array)

if __name__ == '__main__':
    main()
