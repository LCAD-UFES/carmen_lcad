
import os
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
from rl.util import relative_pose, Transform2d
from gradmpp.rddf_tf_planners import MotionPlanner
from rl.envs import CarmenSimEnv
plt.ion()
plt.show()

def read_rddf(rddf):
    carmen_path = os.environ['CARMEN_HOME']
    # rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
    rddf_path = carmen_path + '/data/rndf/' + rddf
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf


def dist(pose, goal):
    return np.sqrt((pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2)


def prepare_rddf(pose, rddf):
    ps, vs = [], []
    
    for i in range(len(rddf)):
        vs.append(10.)
        ps.append(relative_pose(pose, rddf[i]))

    vs[0] = pose[3]
    max_acceleration = 1.0
    total_time = 0
    dts, cdts = [], []
    
    for i in range(1, len(rddf)):
        d = dist(rddf[i], rddf[i-1])
        # from torricelli equation
        a = (vs[i] ** 2 - vs[i - 1] ** 2) / (2 * d)
        
        if a > max_acceleration:
            vs[i] = (vs[i - 1] ** 2 + 2 * max_acceleration * d) ** 0.5
            a = max_acceleration
    
        dt = d / ((vs[i] + vs[i-1]) / 2.)
        total_time += dt
        dts.append(dt)
        cdts.append(total_time)

    return np.array(ps[1:]), np.array(dts), np.array(cdts)


n_steps = 50
step_size = 2
planner = MotionPlanner(l2_weight=0.)

params = {'fix_initial_position': True,
          'allow_negative_commands': False,
          'train': False,
          'use_latency': False,
          'use_acceleration': False,
          'use_spline': False,
          'view': False, 
          'goal_achievement_dist': np.inf,
          'vel_achievement_dist': np.inf,
          'n_steps_episode': -1}

carmen = CarmenSimEnv(params)

init_pos_id = 450
carmen.reset()

while True:
    
    pose = carmen.sim.pose()
    rddf = carmen.sim.rddf_forward()
    
    rddf_p, rddf_dt, rddf_cumdt = prepare_rddf(pose, rddf)

    mmin = np.min(rddf_p[:, :2], axis=0)
    mmax = np.max(rddf_p[:, :2], axis=0)
    diff = mmax - mmin
    max_diff = np.max(diff)

    init = time.time()

    l = np.inf
    n_iters = 0
    while l > 2.0 and n_iters < 100:
        cmds, poses, vc, pc, l = planner.forward(rddf_p, rddf_dt, rddf_cumdt, reinit=False)
        n_iters += 1 
    
    print('n_iters:', n_iters)
    print('forward time: ', time.time() - init)
    print('v coeffs:', vc)
    print('phi coeffs:', pc)

    cmds = np.array(cmds)
    poses = np.array(poses)

    print('RDDF:')
    for i in range(len(rddf_p)):
        print('t: %.2f x: %.2f y: %.2f th: %.2f' % 
              (rddf_cumdt[i], rddf_p[i, 0], rddf_p[i, 1], rddf_p[i, 2]))
    print()

    print('RESULT:')
    for c, p in zip(cmds, poses):
        print('c: %.2f %.2f p: %.2f %.2f %.2f' % 
              (c[0], c[1], p[0], p[1], p[2]))
    print()

    #plt.xlim(mmin[0], mmin[0] + max_diff)
    #plt.ylim(mmin[1] - max_diff / 2., mmin[1] + max_diff / 2.)
    plt.figure('paths')
    plt.clf()
    plt.plot(rddf_p[:, 0], rddf_p[:, 1], 'ob')
    p1 = plt.plot(rddf_p[:, 0], rddf_p[:, 1], '-b')

    plt.plot(poses[:, 0], poses[:, 1], 'or')
    p2 = plt.plot(poses[:, 0], poses[:, 1], '-r')
    plt.legend((None, 'desired', None, 'achieved'))

    plt.figure('velocity')
    plt.clf()
    plt.title('Velocity')
    plt.plot(rddf_cumdt, cmds[:, 0], '-g')
    plt.ylim(-13., 13.)

    plt.figure('phi')
    plt.clf()
    plt.title('Phi')
    plt.plot(rddf_cumdt, np.rad2deg(cmds[:, 1]), '-g')
    plt.ylim(-30.0, 30.0)

    for i in range(len(poses)):
        t1 = Transform2d(pose[0], pose[1], pose[2])
        t2 = Transform2d(poses[i][0], poses[i][1], poses[i][2])
        t3 = t1.transform(t2)
        poses[i] = [t3.x, t3.y, t3.th]
    
    print(poses)
    
    poses = np.array(poses).astype(np.float64)

    plt.show()
    plt.pause(0.001)
    
    carmen.sim.draw_occupancy_map()
    carmen.sim.draw_poses(rddf, 0, 200, 200)
    carmen.sim.draw_poses(poses, 0, 0, 255)
    carmen.sim.draw_pose(pose[0], pose[1], pose[2], 0, 0, 0)
    carmen.sim.show(1)
    
    v = ((cmds[0][0] + pose[3]) / 2.0) / 10.
    p = ((cmds[0][1] + pose[4]) / 2.0) / np.deg2rad(28.)
    
    carmen.step(np.array([v, p]))

