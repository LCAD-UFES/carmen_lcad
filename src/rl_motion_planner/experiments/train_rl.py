
import os
import cv2
import time
import numpy as np
import carmen_comm.carmen_comm as carmen
from rl.util import Transform2d, normalize_theta


def read_rddf(rddf):
    carmen_path = os.environ['CARMEN_HOME']
    rddf_path = carmen_path + '/data/rndf/' + rddf
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf


def read_config():
    carmen_path = os.environ['CARMEN_HOME']
    config_path = carmen_path + '/src/rl_motion_planner/data/config.txt'

    config = dict([l.rstrip().rsplit(' ') for l in open(config_path, 'r').readlines()])
    for key, value in config.items():
        config[key] = float(value)

    return config


def subsample_rddf(rddfs, px, py, pth, n_rddf_poses):
    n = int(len(rddfs) / 3)
    x, y, th = px, py, pth
    subsampled_rddfs = []

    for i in range(n):
        p = 3 * i
        nx, ny, nth = rddfs[p], rddfs[p + 1], rddfs[p + 2]
        d = ((nx - x) ** 2 + (ny - y) ** 2) ** .5
        if d > 1.0:
            x, y, th = nx, ny, nth
            subsampled_rddfs += [x, y, th]

    nmax = 3 * int(n_rddf_poses)

    if len(subsampled_rddfs) > nmax:
        subsampled_rddfs = subsampled_rddfs[:nmax]
    else:
        while len(subsampled_rddfs) < nmax:
            subsampled_rddfs += [px, py, pth]

    return subsampled_rddfs


def subsample_laser(laser, laser_subsample):
    i = 0
    ls = int(laser_subsample)
    subsampled_laser = []
    while i < len(laser):
        subsampled_laser.append(np.min(laser[i:(i + ls)]))
        i += ls
    return subsampled_laser


def assemble_state(pose, goal, v, phi, goal_v, goal_phi, readings, rddfs, config):
    pose.x -= config['x_offset']
    pose.y -= config['y_offset']
    goal.x -= config['x_offset']
    goal.y -= config['y_offset']

    inverted_pose = pose.inverse()
    goal = inverted_pose.transform(goal)

    g = [goal.x / config['max_gx'],
         goal.y / config['max_gy'],
         goal.th]

    v = [v / config['max_v'],
         phi / config['max_phi'],
         goal_v / config['max_v'],
         goal_phi / config['max_phi']]

    readings = subsample_laser(readings, config['laser_subsample'])

    r = [reading / config['max_range'] for reading in readings]

    rddf_poses = []
    rddfs = subsample_rddf(rddfs, pose.x, pose.y, pose.th, config['n_rddf_poses'])
    n = int(len(rddfs) / 3)

    for i in range(n):
        p = i * 3
        rddf_pose = Transform2d(x = rddfs[p] - config['x_offset'],
                                y = rddfs[p + 1] - config['y_offset'],
                                th = rddfs[p + 2])
        rddf_pose = inverted_pose.transform(rddf_pose)
        rddf_pose = [rddf_pose.x / config['max_gx'],
                     rddf_pose.y / config['max_gy'],
                     rddf_pose.th]
        rddf_poses += rddf_pose

    state = np.array(g + v + r + rddf_poses)
    return state


def compute_state(pose_data, goal_data, laser, rddf_data, config):
    pose = Transform2d(pose_data[0], pose_data[1], pose_data[2])
    v, phi = pose_data[3], pose_data[4]

    goal = Transform2d(goal_data[0], goal_data[1], goal_data[2])
    goal_v, goal_phi = goal_data[3], goal_data[4]

    state = assemble_state(pose, goal, v, phi, goal_v, goal_phi, laser, rddf_data, config)
    return state


########################################################
# Policy
########################################################
# Todo


########################################################
# Main
########################################################

if __name__ == "__main__":
    rddf = read_rddf('rddf-voltadaufes-20170809.txt')
    config = read_config()
    carmen.init()

    n_collisions = 0
    goal_achievement_dist = 0.0
    max_episode_size = 500
    max_steps_stopped = 20
    n_episodes = 0

    episodes = []

    while True:
        init_pos_id = np.random.randint(len(rddf))
        init_pos = rddf[init_pos_id]

        carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
        time_since_last_print = time.time()

        episodes.append([])

        n_steps_v_zero = 0
        dist_to_goal = np.inf

        while (not carmen.hit_obstacle()) and \
                (len(episodes[-1]) < max_episode_size) and \
                (n_steps_v_zero < max_steps_stopped) and \
                (dist_to_goal > goal_achievement_dist):

            carmen.handle_messages()
            pose_data = carmen.read_pose()
            goal_data = carmen.read_goal()
            laser_data = carmen.read_laser()
            rddf_data = carmen.read_rddf()
            commands = carmen.read_commands()

            if len(goal_data) == 0:
                print('Invalid goal data. Waiting for valid data.')
                continue

            state = compute_state(pose_data, goal_data, laser_data, rddf_data, config)

            ####################################3
            # Forward
            ####################################3
            nn_v, nn_phi, nn_dt = 0.0, 0.0, 0.1
            vs = [nn_v] * 5
            phis = [nn_phi] * 5
            ts = [nn_dt] * 5
            ####################################3

            # """
            # Janela para permitir terminar episodios antes da hora.
            cv2.imshow('img', np.zeros((200, 200)))
            key = cv2.waitKey(1)

            if key == ord('b'):
                print('Forcing end of episode. n_steps_v_zero:', n_steps_v_zero)
                break
            # """

            if abs(pose_data[3]) <= 1e-5: n_steps_v_zero += 1
            else: n_steps_v_zero = 0

            ####################################
            # Adidicionar dados do episodio na lista
            ####################################
            # Todo.

            if time.time() - time_since_last_print > 2.0:
                print('Report')
                time_since_last_print = time.time()

            carmen.publish_command(vs, phis, ts, True)

        print("Episode done.")
        carmen.publish_stop_command()

        collided = False
        n_episodes += 1
        if carmen.hit_obstacle():
            collided = True
            n_collisions += 1

        ######################################3
        # Train and save policy
        ######################################3


        ######################################3
        # Print report
        ######################################3
        print('Report')
