"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 5.0  # maximum road width [m]
D_ROAD_W = 0.2  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAXT = 10.0  # max prediction time [m]
MINT = 9.9  # min prediction time [m]
TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 4.0  # robot radius [m]

# cost weights
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 1.0
KLON = 1.0

show_animation = True


class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):

    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Loongitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2

                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob, robot_radius):

    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= robot_radius**2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob, robot_radius):

    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob, robot_radius):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob, robot_radius):

    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob, robot_radius)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath, fplist

def frenet_planning(x, y, s0, c_speed, c_d, c_d_d, c_d_dd, ob, robot_radius):
    csp = cubic_spline_planner.Spline2D(x, y)
 
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    if ob.size > 0:
        fplist = check_paths(fplist, ob, robot_radius)
 
    return fplist

def frenet_find_optimal(fplist):
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def test(rddf, ob, c_speed, displacement, vel_dif, acc_dif):
    ob = np.array(ob)
    rddf = np.array(rddf)
    _, rddf_idx = np.unique(rddf, return_index=True, axis=0)
    rddf = rddf[np.sort(rddf_idx)]

    fplist = frenet_planning(rddf[:,0], rddf[:,1], 0.0, c_speed, displacement, vel_dif, acc_dif, ob, 3.0)
    
    print("[Python]",len(fplist)," Paths found")
    if len(fplist) == 0:
        print("[Python]Cant find any path, Reducing safety distance")
        fplist = frenet_planning(rddf[:,0], rddf[:,1], 0.0, c_speed, displacement, vel_dif, acc_dif, ob,2.0)
        if len(fplist) == 0:
            print("[Python]Cant find any path at all")
            return []

    path = frenet_find_optimal(fplist)

#     plt.cla()
#     plt.plot(rddf[:,0], rddf[:,1])
#     if ob.size > 0:
#         plt.plot(ob[:, 0], ob[:, 1], "xk")
#     for fp in fplist:
#         plt.plot(fp.x[1:], fp.y[1:], "-g")
#     plt.plot(path.x[1:], path.y[1:], "-or")
#     plt.plot(path.x[1], path.y[1], "vc")
#     plt.xlim(path.x[1] - 50.0, path.x[1] + 50.0)
#     plt.ylim(path.y[1] - 50.0, path.y[1] + 50.0)
#     plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
#     plt.grid(True)
#     plt.pause(0.000001)

    path_x = np.array(path.x).reshape(len(path.x),1)
    path_y = np.array(path.y).reshape(len(path.y),1)
    path_yaw = np.array(path.yaw).reshape(len(path.yaw),1)
    return np.concatenate((path_x, path_y, path_yaw), axis=1).tolist()
  
def main():
    print(__file__ + " start!!")

    # way points
    wx = [7757856.715485,
        7757857.202057,
        7757857.691403,
        7757858.176561,
        7757858.641998,
        7757859.120141,
        7757859.578607,
        7757860.046205,
        7757860.526738,
        7757860.993184,
        7757861.476614,
        7757861.940667,
        7757862.431454,
        7757862.914915,
        7757863.391341,
        7757863.881642,
        7757864.38697 ,
        7757864.882983,
        7757865.369365,
        7757865.855209,
        7757866.341678,
        7757866.833314,
        7757867.316039,
        7757867.813749,
        7757868.309463,
        7757868.793431,
        7757869.281193,
        7757869.763361,
        7757870.267778,
        7757870.76416 ,
        7757871.247802,
        7757871.738862,
        7757872.225228,
        7757872.712088,
        7757873.183084,
        7757873.658   ,
        7757874.134747,
        7757874.619511,
        7757875.095298,
        7757875.588255,
        7757876.070013,
        7757876.54883 ,
        7757877.040554,
        7757877.50224 ,
        7757877.989228,
        7757878.461122,
        7757878.933451,
        7757879.393716,
        7757879.860559,
        7757880.311198,
        7757880.776983,
        7757881.244484,
        7757881.709763,
        7757882.186818,
        7757882.651253,
        7757883.125022,
        7757883.590231,
        7757884.065787,
        7757884.54129 ,
        7757885.012503,
        7757885.498437,
        7757885.960524,
        7757886.427333,
        7757886.902455,
        7757887.367675,
        7757887.845191,
        7757888.316694,
        7757888.784801,
        7757889.273165,
        7757889.725679,
        7757890.183984,
        7757890.643046,
        7757891.095541,
        7757891.539832,
        7757891.984405,
        7757892.417148,
        7757892.852183,
        7757893.294124,
        7757893.734256,
        7757894.185788,
        7757894.621364,
        7757895.076869,
        7757895.521903,
        7757895.95145 ,
        7757896.398642,
        7757896.835914,
        7757897.286456,
        7757897.729071,
        7757898.146237,
        7757898.581578,
        7757899.014575,
        7757899.4351  ,
        7757899.875863,
        7757900.297491,
        7757900.732673,
        7757901.167781,
        7757901.603488,
        7757902.480295,
        7757902.909934,
        7757903.334994]
    wy = [-363557.008953,
        -363557.441027,
        -363557.874753,
        -363558.300826,
        -363558.717449,
        -363559.137666,
        -363559.54928 ,
        -363559.969115,
        -363560.396507,
        -363560.816421,
        -363561.252516,
        -363561.669707,
        -363562.104877,
        -363562.526962,
        -363562.942575,
        -363563.369782,
        -363563.806298,
        -363564.237731,
        -363564.658918,
        -363565.080957,
        -363565.502758,
        -363565.930722,
        -363566.347587,
        -363566.779835,
        -363567.213121,
        -363567.634178,
        -363568.060982,
        -363568.484123,
        -363568.925858,
        -363569.360365,
        -363569.780712,
        -363570.211448,
        -363570.636761,
        -363571.063517,
        -363571.477206,
        -363571.894695,
        -363572.313252,
        -363572.735786,
        -363573.153468,
        -363573.582441,
        -363574.008129,
        -363574.431786,
        -363574.860905,
        -363575.265254,
        -363575.694045,
        -363576.100742,
        -363576.509687,
        -363576.914456,
        -363577.334161,
        -363577.734859,
        -363578.146738,
        -363578.565613,
        -363578.98074 ,
        -363579.411963,
        -363579.82703 ,
        -363580.250133,
        -363580.672308,
        -363581.095159,
        -363581.516137,
        -363581.928317,
        -363582.350938,
        -363582.758177,
        -363583.167323,
        -363583.582943,
        -363583.992002,
        -363584.412807,
        -363584.828518,
        -363585.242448,
        -363585.663807,
        -363586.060032,
        -363586.46145 ,
        -363586.856758,
        -363587.24726 ,
        -363587.629226,
        -363588.019634,
        -363588.400627,
        -363588.78073 ,
        -363589.16841 ,
        -363589.55597 ,
        -363589.954382,
        -363590.337107,
        -363590.743692,
        -363591.139662,
        -363591.530654,
        -363591.929604,
        -363592.317521,
        -363592.714269,
        -363593.100206,
        -363593.466845,
        -363593.852631,
        -363594.236252,
        -363594.60999 ,
        -363594.998994,
        -363595.372861,
        -363595.755979,
        -363596.144723,
        -363596.53159 ,
        -363597.305421,
        -363597.683794,
        -363598.056808]
        # obstacle li
    ob = np.array([[7757888.05712317, -363586.05494902]])
    tx, ty, tyaw, tc , csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current latral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 30.0  # animation area length [m]

    for i in range(500):

        fplist = frenet_planning(
            wx, wy, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        path = frenet_find_optimal(fplist)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:
            plt.cla()
            plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            for fp in fplist:
                plt.plot(fp.x[1:], fp.y[1:], "-g")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()