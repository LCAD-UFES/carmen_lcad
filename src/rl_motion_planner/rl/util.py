
import numpy as np


def dist(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def normalize_theta(theta):
    if theta >= -np.pi and theta < np.pi:
        return theta

    multiplier = np.floor(theta / (2 * np.pi))
    theta = theta - multiplier * 2 * np.pi

    if theta >= np.pi:
        theta -= 2 * np.pi
    if theta < -np.pi:
        theta += 2 * np.pi
    return theta


def rotate(x, y, angle):
    rot_x = np.cos(angle) * x - np.sin(angle) * y
    rot_y = np.sin(angle) * x + np.cos(angle) * y
    return rot_x, rot_y


class Transform2d:
    def __init__(self, x=0., y=0., th=0.):
        self.x = x
        self.y = y
        self.th = th

    def transform(self, t):
        result = Transform2d(self.x, self.y, self.th)
        result.transform_inline(t)
        return result

    def transform_inline(self, t):
        rx, ry = rotate(t.x, t.y, self.th)
        self.x = self.x + rx
        self.y = self.y + ry
        self.th = normalize_theta(self.th + t.th)

    def inverse(self):
        result = Transform2d()
        result.x, result.y = rotate(-self.x, -self.y, -self.th)
        result.th = -self.th
        return result

    def __repr__(self):
        return '[%f, %f, %f]' % (self.x, self.y, self.th)


def relative_pose(x, y):
    a = Transform2d(y[0] - x[0], y[1] - x[1], y[2])
    b = Transform2d(0., 0., x[2])
    c = b.inverse().transform(a)
    return [c.x, c.y, c.th]


def ackerman_motion_model(pose, v, phi, dt, L=2.625):
    new_pose = np.copy(pose)
    new_pose[0] += v * dt * np.cos(pose[2])
    new_pose[1] += v * dt * np.sin(pose[2])
    new_pose[2] += v * dt * np.tan(phi) / L
    new_pose[2] = normalize_theta(new_pose[2])
    return new_pose