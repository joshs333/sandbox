#!/usr/bin/env python3
import matplotlib.pyplot as plt
import sympy as sp
import math

def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

if __name__ == "__main__":
    print("Running pendulum dynamics test.")
    l_f, l_r, dt = sp.symbols('l_f l_r dt')
    x, y, theta, v, delta = sp.symbols('x y theta v delta')
    v_dot, delta_dot = sp.symbols('v_dot delta_dot')
    beta = sp.atan(l_r * sp.tan(theta) / (l_r + l_f))
    R = (l_r + l_f) / (sp.tan(delta) * sp.cos(beta))


    state = sp.Matrix([x, y, theta, v, delta])
    control = sp.Matrix([v_dot, delta_dot])
    dynamics = sp.Matrix([
        x + dt * v * sp.cos(beta + theta),
        y + dt * v * sp.sin(beta + theta),
        theta + dt * v / R,
        v + dt * v_dot,
        delta + dt * delta_dot,
    ]).applyfunc(sp.simplify)

    sp.pprint(dynamics)
    sp.pprint(dynamics.diff(state).applyfunc(sp.simplify))
