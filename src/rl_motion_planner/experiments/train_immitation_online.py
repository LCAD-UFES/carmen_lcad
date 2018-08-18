
"""
filipe   19157 20.2  1.3 149324 105668 pts/7   S+   00:31   5:33 ./map_server -map_path ../data/map_ida_guarapari-20170403-2 -map_x 7757721.8 -map_y -363569.5 -block_map on -lanemap_incoming_message_type 0
filipe   19162  0.9  0.0  40544  6252 pts/7    S+   00:31   0:14 ./rddf_play ../data/rndf/rddf-voltadaufes-20170809.txt
filipe   19163  6.4  0.2  46256 19496 pts/7    S+   00:31   1:46 ./behavior_selector
filipe   19164  5.7  0.1  25356 15072 pts/7    S+   00:31   1:35 ./obstacle_avoider
"""

import os
import cv2
import time
import math
import numpy as np
from rl.policy import Policy
import carmen_comm.carmen_comm as carmen
from rl.util import Transform2d, normalize_theta

# ###############################################################################
# Copied from rl.env. TODO: improve.
# ###############################################################################

def read_rddf(rddf):
    carmen_path = os.environ['CARMEN_HOME']
    # rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
    rddf_path = carmen_path + '/data/rndf/' + rddf
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf

# ###############################################################################

def compute_state(policy, pose_data, goal_data, laser, rddf_data):
    pose = Transform2d(pose_data[0], pose_data[1], pose_data[2])
    v, phi = pose_data[3], pose_data[4]
    
    goal = Transform2d(goal_data[0], goal_data[1], goal_data[2])
    goal_v, goal_phi = goal_data[3], goal_data[4]

    state = policy.assemble_state(pose, goal, v, phi, goal_v, goal_phi, laser, rddf_data)    
    return state


def generate_plan(policy, pose_data, n_commands=20, dt=0.2):
    x, y, th = pose_data[0], pose_data[1], pose_data[2]
    v, phi = pose_data[3], pose_data[4]

    carmen.simulation_reset(x, y, th, v, phi)
    
    vs = []
    phis = []
    ts = []
    
    base_time = time.time()
    
    i = 0
    while (i < n_commands) and (not carmen.simulation_hit_obstacle()):
        pose_data = carmen.simulation_read_pose()
        goal_data = carmen.simulation_read_goal()
        laser = carmen.simulation_read_laser()

        if len(goal_data) == 0:
            print('Invalid goal data during simulation.')
            break

        # print('Simulation step:', i, 'Pose:', pose_data, 'Goal:', goal_data)
        # print('Laser[:20]:', laser[:20])
        # print()

        v, phi = compute_command(policy, pose_data, goal_data, laser)
        
        vs.append(v)
        phis.append(phi)
        ts.append(dt)
        
        carmen.simulation_step(v, phi, dt)
        i += 1

    # print('vs:', vs)
    # print('phis:', phis)
    # print('ts:', ts)

    return vs, phis, ts


def find_next_command(plan, plan_time):
    if plan is None or plan_time is None:
        return 0., 0., False, 0.
    else:
        t = time.time() - plan_time
        n = int(len(plan) / 4)
        cum_cmd_t = 0
        for i in range(n):
            p = i * 3
            v = plan[p]
            phi = plan[p + 1]
            cmd_t = plan[p + 2]
            cum_cmd_t += cmd_t
            if t < cmd_t:
                return v, phi, True, cmd_t
        return 0., 0., False, 0.  


def simple_planner(pose_data, goal_data, laser_data, planning_horizon):
    # if close to obstacle or close to the goal, brake and move the steering wheel to 0.0
    n = int(len(laser_data) / 2)
    avg_dist_ahead = np.mean(laser_data[(n - 2) : (n + 2)])
    if avg_dist_ahead < 1.0:
        v = 0.
        phi = 0.
    # else, try to achieve the goal velocity and move in direction to the goal.
    else:
        v = goal_data[3]
        
        # th = normalize_theta(goal_data[2] - pose_data[2])
        goal_t = Transform2d(goal_data[0] - pose_data[0], goal_data[1] - pose_data[1], goal_data[2])
        pose_t = Transform2d(0., 0., pose_data[2])
        goal_t = pose_t.inverse().transform(goal_t)
        th = math.atan2(goal_t.y, goal_t.x)

        if goal_t.x < 1.0:
            phi = 0.
        else:
            L = 2.625
            phi = math.atan((th * L) / ((pose_data[3] + 1e-4) * planning_horizon))
            phi = np.clip(phi, -0.55, 0.55)
     
    return v, phi 


rddf = read_rddf('rddf-voltadaufes-20170809.txt')
carmen.init()

n_collisions = 0

policy = Policy(input_shape=[98], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-3, nonlin='tanh',
                single_thread=False, n_hidden_layers=3,
                continuous_std=1e-7)

# policy.load('data/model_online_immitation_learning.ckpt')

goal_achievement_dist = 0.0
max_episode_size = 500
max_steps_stopped = 20
n_episodes = 0

while True:
    init_pos_id = np.random.randint(len(rddf))
    init_pos = rddf[init_pos_id]
    # init_pos = rddf[int(len(rddf) / 2)]
    # init_pos = rddf[0]
    # init_pos = rddf[-1000]

    # goal_id = init_pos_id + 30
    # goal_data = rddf[goal_id]
    # goal_v = goal_data[3] * np.random.random()
    # goal_phi = goal_data[4] * np.random.random()
    # print('desired v:', goal_v, 'desired_phi:', goal_phi)

    """
    carmen.publish_goal_list([goal_data[0]], [goal_data[1]],
        [goal_data[2]], 
        [goal_v], 
        [goal_phi], 
        time.time());
    """
    
    carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
    time_since_last_print = time.time()

    episode = []
    # plan = None
    # plan_time = None
    n_steps_v_zero = 0
    algo = 'nn' # 'mpp' if n_episodes % 2 == 0 else 'nn'

    """    
    goal_data = carmen.read_goal()
    if len(goal_data) <= 0:
        continue
    """    
    dist_to_goal = np.inf # ((goal_data[0] - init_pos[0]) ** 2 + (goal_data[1] - init_pos[1]) ** 2) ** .5
    # pdg = dist_to_goal
    # pdv = abs(init_pos[3] - goal_v)
    episode_rl = {'states': [], 'actions': [], 'rewards': []}
    cmd_v = cmd_phi = cmd_dt = 0.0 

    while (not carmen.hit_obstacle()) and \
          (len(episode) < max_episode_size) and \
          (len(episode_rl['states']) < max_episode_size) and \
          (n_steps_v_zero < max_steps_stopped) and \
          (dist_to_goal > goal_achievement_dist):
        carmen.handle_messages()
        pose_data = carmen.read_pose()
        goal_data = carmen.read_goal()
        laser_data = carmen.read_laser()
        rddf_data = carmen.read_rddf()
        commands = carmen.read_commands()

        dist_to_goal = np.inf # ((goal_data[0] - pose_data[0]) ** 2 + (goal_data[1] - pose_data[1]) ** 2) ** .5 

        if len(goal_data) == 0:
            print('Invalid goal data. Waiting for valid data.')
            continue

        state = compute_state(policy, pose_data, goal_data, laser_data, rddf_data)
        nn_v, nn_phi = policy.forward(state)
        nn_dt = 0.1 

        # """
        cv2.imshow('img', np.zeros((200, 200)))
        key = cv2.waitKey(1)
        
        if key == ord('b'):
            print('Forcing end of episode. n_steps_v_zero:', n_steps_v_zero)
            break
        # """
        
        """        
        if len(commands) > 0 and commands[0] > 0.01:
            plan = np.copy(commands)
            plan_time = time.time()

        cmd_v, cmd_phi, dt, valid = find_next_command(plan, plan_time)
        """
        if abs(pose_data[3]) <= 1e-5:
            n_steps_v_zero += 1
        else:
            n_steps_v_zero = 0

        cmd_v, cmd_phi = simple_planner(pose_data, goal_data, laser_data, 1.0)
        cmd_dt = 0.1
        # print(cmd_v, cmd_phi, cmd_dt)

        # """
        # cmd_v, cmd_phi, cmd_dt = commands[0], commands[1], commands[2] 
        episode.append([state, [cmd_v, cmd_phi]])
        # """
        
        if algo == 'mpp':
            vs = [cmd_v] * 5
            phis = [cmd_phi] * 5
            ts = [cmd_dt] * 5
        else:
            vs = [nn_v] * 5
            phis = [nn_phi] * 5
            ts = [nn_dt] * 5

        # """
        if time.time() - time_since_last_print > 2.0:
            print('Algorithm: %s NSteps: %d PlannerCommand: %.2f %.2f NNCommand: %.2f %.2f' % (algo, len(episode), cmd_v, cmd_phi, nn_v, nn_phi))
            # print('Critic:', policy.sess.run(policy.critic, feed_dict={policy.state: [state]}))
            time_since_last_print = time.time()
        # """
        
        # init = time.time()
        # vs, phis, ts = generate_plan(policy, pose_data)
        # print('Time to generate plan: %.4lf' % (time.time() - init))

        # dv = abs(pose_data[3] - goal_v)
        # pdg = dist_to_goal
        # rw = (pdg - dist_to_goal) + (pdv - dv)
        # pdg = dist_to_goal
        # pdv = dv 
        
        # episode_rl['states'].append(state)
        # episode_rl['actions'].append([v, phi])
        # episode_rl['rewards'].append(rw)
        
        carmen.publish_command(vs, phis, ts, True) # , pose_data[0], pose_data[1], pose_data[2])
        """
        carmen.publish_goal_list([goal_data[0]], [goal_data[1]],
            [goal_data[2]], [goal_v], [goal_phi], time.time());
        """
        
    print("Episode done.")
    carmen.publish_stop_command()

    collided = False
    n_episodes += 1
    if carmen.hit_obstacle(): # len(episode) < max_episode_size and n_steps_v_zero < max_steps_stopped and dist_to_goal > goal_achievement_dist:
        collided = True
        n_collisions += 1

    """
    if len(episode_rl['rewards']) > 1:
        if dist_to_goal < goal_achievement_dist:
            episode_rl['rewards'][-1] += 10.0
        elif collided:
            episode_rl['rewards'][-1] -= 10.0
        loss = policy.train(episode_rl)
    """

    loss = policy.train_immitation_learning_episode(episode, max(1000, len(episode)) * 1, 256)
    policy.save('data/model_online_immitation_learning.ckpt')

    print('Episode:', n_episodes, 'algo:', algo, 
          'n_collisions:', n_collisions, 
          'Episode size:', len(episode), 
          'loss:', loss, 'Collided:', collided)
