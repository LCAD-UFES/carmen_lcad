
import sys
import time
from scipy.optimize import minimize
import numpy as np


def apply_poly(x, params):
    y = 0.
    for i in range(len(params)):
        y += params[i] * (x ** i)
    return np.tanh(y)


def simulate(params):
    dt = 0.1
    max_v = 10.0
    max_phi = 0.488692  # 28 degrees
    L = 2.625
    nsteps = 30

    n = int(len(params) / 2)

    vparams = params[:n]
    pparams = params[n:]

    x = y = th = 0.0
    cmds, poses = [], []

    for i in range(nsteps):
        t = float(i) / (nsteps - 1)
        v = max_v * apply_poly(t, vparams)
        phi = max_phi * apply_poly(t, pparams)

        dx = dt * v * np.cos(th)
        dy = dt * v * np.sin(th)
        dth = dt * (v / L) * np.tan(phi)

        x = x + dx
        y = y + dy
        th = th + dth

        cmds.append([v, phi])
        poses.append([x, y, th])

    return np.array(cmds), np.array(poses)


def objective(params, goal, goal_v):
    cmds, poses = simulate(params)

    loss = 0.

    for i in range(1, len(poses)):
        # loss += (cmds[i][1] - cmds[i-1][1]) ** 2
        loss += (poses[i][0] - poses[i-1][0]) ** 2 + (poses[i][1] - poses[i-1][1]) ** 2

    loss += (goal[0] - poses[-1][0]) ** 2 + (goal[1] - poses[-1][1]) ** 2
    # loss += ((goal_v - cmds[-1][0]) ** 2)
    # loss += np.sum(np.square(cmds[:, 1]))

    return loss


class ScipyPolyPlanner:
    def __init__(self):
        self.init_sol = np.ones(10)

    def forward(self, goal, goal_v):
        options = {'disp': True, 'maxiter': 10000}

        res = minimize(fun=objective, x0=self.init_sol,
                       args=(goal, goal_v),
                       method='BFGS', options=options)

        self.init_sol = res.x
        cmds, poses = simulate(res.x)

        return cmds, poses

