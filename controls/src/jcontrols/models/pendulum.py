#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import math

class PendulumModel():
    m = 1
    g = 9.8
    l = 1

def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def pendulum_f(x, u, model):
    o_shape = x.shape
    x_l = np.array(x).reshape(2)
    u_l = np.array(u).reshape(1)

    x_l[0] = wrap_pi(x_l[0])
    # accel = model.g * math.sin(x_l[0]) / model.l + u_l[0] / (model.m * model.l * model.l)
    # if accel > 0:
    return np.array([
        x_l[1],
        model.g * math.sin(x_l[0]) / model.l + u_l[0] / (model.m * model.l * model.l) - 0.2 * x_l[1]
    ]).reshape(o_shape)

if __name__ == "__main__":
    print("Running pendulum dynamics test.")
    model = PendulumModel()

    x = np.array([math.pi/4, 0])
    for i in range(5000):
        print("%3.2f, %s"%(i * 0.1, str(x)))
        x = x + pendulum_f(x, 0, model) * 0.01
        x[0] = wrap_pi(x[0])