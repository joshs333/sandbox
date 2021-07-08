#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import math
import scipy
import scipy.linalg

def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

class DynamicBicycleModel:
    class Params:
        cornering_stiffness_front = 0.0
        cornering_stiffness_rear = 0.0
        yaw_moment_intertia = 0.0
        length_front = 0.0
        length_rear = 0.0
        mass = 0.0

    def __init__(self, params_ = Params()):
        self.params = params_

    def f(self, state, control):
        """
        state:
            x
            y
            theta
            delta
            v_x
            v_y
            r
        control:
            gamma
            accel
        """
        state_shape = state.shape
        state = state.reshape((7))
        control = control.reshape((2))

        return np.array([
            0.,
            0.,
            0.,
            0.,
            
            state[3] * math.cos(state[2]),
            state[3] * math.sin(state[2]),
            state[3] * math.tan(control[0]) / self.params.L,
            control[1]
        ]).reshape(state_shape)

    def A(self, state, control):
        state = state.reshape((4))
        control = control.reshape((2))

        return np.array([
            [0., 0., -state[3] * math.sin(state[2]), math.cos(state[2])],
            [0., 0., state[3] * math.cos(state[2]),math.sin(state[2])],
            [0.,0.,0.,math.tan(control[0]) / self.params.L],
            [0.,0.,0.,0.]
        ])


    def B(self, state, control):
        state = state.reshape((4))
        control = control.reshape((2))

        return np.array([
            [0., 0.],
            [0., 0.],
            [state[3] / (math.cos(control[0])**2 * self.params.L ), 0.],
            [0., 1.]
        ])

track_len = 1000
finer_dt = 10
horizon = 50
dt = 0.01

def getTraj(dyn, start):
    res_traj = [start]
    res_u = []
    un = np.array([0.,0.])
    un_dir = 0.005
    un_max = 0.5
    v_max = 2.0

    x_n = start
    for i in range(track_len):
        if(un[0] > un_max and un_dir > 0.):
            un_dir *= -1.
        elif(un[0] < -un_max and un_dir < 0.):
            un_dir *= -1.
        un = np.array([un[0] + un_dir, 0.])

        x_n = x_n + dt * dyn.f(x_n, un)
        res_traj.append(x_n)
        res_u.append(un)
        
    return np.array(res_traj), np.array(res_u)


def lqr(A, B, Q, R):
    # Solve the Ricatti equation using our state space equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
    
    #compute the LQR gain K
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
    return K

def get_closest_point(path, xn, ladist = None):
    dist = None
    f = 0
    for i in range(len(path)):
        dist_n = np.linalg.norm(path[i,:2] - xn[:2])
        if dist is None or dist_n < dist:
            dist = dist_n
            f = i
    if ladist is not None:
        for i in range(len(path) - f):
            dist_n = np.linalg.norm(path[i + f,:2] - xn[:2])
            if dist_n >= ladist:
                f = i + f
                break

    return f


if __name__ == "__main__":
    print("Running pendulum dynamics test.")
    start = np.array([0,0,-64 * math.pi / 385,2.0])

    dyn = BicycleModel()
    trajx, _ = getTraj(dyn, start)

    # print(trajx[:,0])
    print(trajx.shape)

    plt.plot(trajx[:,0], trajx[:,1])
    # plt.show()
    # exit()

    Q = np.eye(4) * 1.
    R = np.eye(2) * 1.

    tp = 0
    tp_dist = 0
    xn = start
    un = np.array([0.,0.])
    unshape = un.shape

    follow_traj = [start]
    follow_u = []
    while tp < track_len:
        print(tp)
        tp = get_closest_point(trajx, xn,0.7)
        A = dyn.A(xn, un)
        B = dyn.B(xn, un)
        K = lqr(A, B, Q, R)
        un = -np.array(K @ (xn - trajx[tp])).reshape(2)
        # un[1] = 0.
        follow_u.append(un)

        for i in range(finer_dt):
            xn = xn + dt * dyn.f(xn, un) / float(finer_dt)
            follow_traj.append(xn)

        tp += 1


    follow_traj = np.array(follow_traj)
    cdist_tot = 0.
    num = 0
    for i in range(len(follow_traj)):
        # if i >= len(trajx) or i >= len(follow_traj):
        #     break
        # num = i + 1
        cp = get_closest_point(trajx, follow_traj[i])
        cdist = np.linalg.norm(trajx[cp,:2] - follow_traj[i,:2])
        cdist_tot += cdist
    
    print(cdist_tot)
    print(cdist_tot / float(len(follow_traj)))


    follow_traj = np.array(follow_traj)
    plt.plot(follow_traj[:,0], follow_traj[:,1])
    plt.show()
