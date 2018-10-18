
import os
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
from rl.util import relative_pose, Transform2d
from gradmpp.rddf_tf_planners import MotionPlanner
from rl.envs import CarmenSimEnv
import panel.pycarmen_panel as pycarmen_panel

plt.ion()
plt.show()


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
    rddf[0] = pose
    
    for i in range(1, len(rddf)):
        d = dist(rddf[i], rddf[i-1])
        # from torricelli equation
        a = (vs[i] ** 2 - vs[i - 1] ** 2) / (2 * d)
        
        if a > max_acceleration:
            vs[i] = (vs[i - 1] ** 2 + 2 * max_acceleration * d) ** 0.5
            a = max_acceleration
    
        if a != 0:
            dt = (vs[i] - vs[i - 1]) / a
        elif np.abs(vs[i]) > 0:
            dt = d / np.abs(vs[i])
        else:
            dt = 0.1
        
        total_time += dt
        dts.append(dt)
        cdts.append(total_time)

    return np.array(ps[1:]), np.array(dts), np.array(cdts), np.array(vs[1:])


n_steps = 50
step_size = 2
planner = MotionPlanner(l2_weight=0., use_acceleration=False)
panel = pycarmen_panel.CarPanel()

params = {'fix_initial_position': True,
          'allow_negative_commands': False,
          'train': False,
          'use_latency': True,
          'use_acceleration': False,
          'use_spline': False,
          'view': False, 
          'goal_achievement_dist': np.inf,
          'vel_achievement_dist': np.inf,
          'n_steps_episode': -1}

carmen = CarmenSimEnv(params)

init_pos_id = 450

carmen.sim.set_initial_pose(100)
carmen.reset()

while True:
    
    pose = carmen.sim.pose()
    rddf = carmen.sim.rddf_forward()
    
    rddf_p, rddf_dt, rddf_cumdt, des_vs = prepare_rddf(pose, list(rddf))

    mmin = np.min(rddf_p[:, :2], axis=0)
    mmax = np.max(rddf_p[:, :2], axis=0)
    diff = mmax - mmin
    max_diff = np.max(diff)

    init = time.time()

    l = [np.inf]
    n_iters = 0
    while np.mean(l) > 0.5 and n_iters < 100:
        cmds, poses, vc, pc, l, eff = planner.forward(rddf_p, rddf_dt, rddf_cumdt, reinit=False,
                                                      desired_v=des_vs, pose=pose)
        n_iters += 1 

    v = (pose[3] + ((cmds[0][0] - pose[3]) * (0.15 / rddf_cumdt[0]))) / 10.
    p = (pose[4] + ((cmds[0][1] - pose[4]) * (0.15 / rddf_cumdt[0]))) / np.deg2rad(28.)
    # 
    # v = cmds[0][0] / 10.
    # p = cmds[0][1] / np.deg2rad(28.)

    sim_cmd = np.array([v, p])
    
    print('n_iters:', n_iters)
    print('forward time: ', time.time() - init)
    print('v coeffs:', vc)
    print('phi coeffs:', pc)

    cmds = np.array(cmds)
    poses = np.array(poses)
    eff = np.array(eff)

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

    """
    #plt.xlim(mmin[0], mmin[0] + max_diff)
    #plt.ylim(mmin[1] - max_diff / 2., mmin[1] + max_diff / 2.)
    plt.figure('paths')
    plt.clf()
    plt.plot(rddf_p[:, 0], rddf_p[:, 1], 'ob')
    p1 = plt.plot(rddf_p[:, 0], rddf_p[:, 1], '-b')
    plt.plot(poses[:, 0], poses[:, 1], 'or')
    p2 = plt.plot(poses[:, 0], poses[:, 1], '-r')
    plt.legend(('', 'desired', '', 'achieved'))

    plt.figure('velocity')
    plt.clf()
    plt.title('Velocity')
    plt.plot(rddf_cumdt, des_vs, '-c')
    plt.plot(rddf_cumdt, cmds[:, 0], '-g')
    plt.legend(('desired', 'command'))
    plt.ylim(-13., 13.)

    plt.figure('phi')
    plt.clf()
    plt.title('Phi')
    plt.plot(rddf_cumdt, np.rad2deg(cmds[:, 1]), '-g')
    plt.ylim(-30.0, 30.0)

    plt.figure('acc')
    plt.clf()
    plt.title('Acceleration')
    plt.plot(rddf_cumdt, eff[:, 0], '-g')
    plt.ylim(-1.0, 1.0)

    plt.figure('phi_v')
    plt.clf()
    plt.title('Phi Velocity')
    plt.plot(rddf_cumdt, np.rad2deg(eff[:, 1]), '-g')
    plt.ylim(-30.0, 30.0)

    plt.show()
    plt.pause(0.001)
    """

    for i in range(len(poses)):
        t1 = Transform2d(pose[0], pose[1], pose[2])
        t2 = Transform2d(poses[i][0], poses[i][1], poses[i][2])
        t3 = t1.transform(t2)
        poses[i] = [t3.x, t3.y, t3.th]
    
    print(poses)
    
    poses = np.array(poses).astype(np.float64)

    panel.draw(pose[3], pose[4], carmen.sim_t)
    carmen.sim.draw_occupancy_map()
    carmen.sim.draw_poses(rddf, 0, 255, 0)
    carmen.sim.draw_poses(poses, 0, 0, 255)
    carmen.sim.draw_pose(pose[0], pose[1], pose[2], 0, 0, 0)
    carmen.sim.show(2)

    _, _, info = carmen.step(sim_cmd)
    
    if (info['hit_obstacle']):
        print()
        print('** COLLISION DETECTED! **')
        print()
        carmen.sim.show(-1)

