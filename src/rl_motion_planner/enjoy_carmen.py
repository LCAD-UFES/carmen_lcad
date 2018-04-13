
import os
# import cv2
import time
import numpy as np
from rl.policy import Policy
import carmen_comm.carmen_comm as carmen
from rl.util import Transform2d


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


def filter_v(cmd_v, current_v, dt, laser_data):
    """
    Regulate the maximum acceleration.
    """
    max_allowed_acceleration = 1.0
    
    if abs(cmd_v - current_v) > dt * max_allowed_acceleration:
        new_v = current_v + (1. if cmd_v > current_v else -1.) * dt * max_allowed_acceleration
        cmd_v = new_v

    """
    Reduce speed when close to obstacles. It's not working well currently. 
    """
    """
    min_allowed_dist_to_obstacle = 6.0
    min_dist_to_obstacle_before_braking = 20.0

    n = int(len(laser_data) / 2)
    avg_dist_obstacle_ahead = np.mean(laser_data[(n - 2) : (n + 2)])
    print('avg_dist_obstacle_ahead:', avg_dist_obstacle_ahead)

    if avg_dist_obstacle_ahead < min_allowed_dist_to_obstacle and cmd_v > 0:
        cmd_v = 0.
        print('cmd_v set to zero.')
    elif avg_dist_obstacle_ahead < min_dist_to_obstacle_before_braking and cmd_v > 0:
        tan_alpha = current_v / avg_dist_obstacle_ahead - 1.
        pred_dist_after_cmd = dt * current_v
        cmd_v = (pred_dist_after_cmd - 1.) * tan_alpha
        if cmd_v < 0:
            cmd_v = 0.
        print('reducing speed cmd_v:', cmd_v)
    """
    
    return cmd_v


def generate_plan(policy, pose_data, n_commands=30, dt=0.1):
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
        laser_data = carmen.simulation_read_laser()

        if len(goal_data) == 0:
            break

        state = compute_state(policy, pose_data, goal_data, laser_data, None)
        nn_v, nn_phi = policy.forward(state)
        
        """
        By using v as input we assume that the car won't be able to achieve the desired velocity. 
        It is a conservative approach.
        Alternativelly, We could also be less conservative and use pose_data[3] (the predicted   
        velocity) instead of v. This is equivalent to assume the car will achieve the 
        desired velocity. 
        """
        nn_v = filter_v(nn_v, pose_data[3], dt, laser_data)

        vs.append(nn_v)
        phis.append(nn_phi)
        ts.append(dt)
        
        carmen.simulation_step(nn_v, nn_phi, dt)
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


rddf = read_rddf('rddf-voltadaufes-20170809.txt')

carmen.init()

n_collisions = 0

policy = Policy(input_shape=[98], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-3, nonlin='tanh',
                single_thread=False, n_hidden_layers=3,
                continuous_std=1e-7)

# policy.load('data/model_immitation.ckpt')
carmen_path = os.environ['CARMEN_HOME']
policy.load(carmen_path + '/src/rl_motion_planner/data/model_online_immitation_learning.ckpt')

goal_achievement_dist = 1.0
max_steps_stopped = 20
n_episodes = 0

while True:
    # init_pos_id = np.random.randint(len(rddf))
    # init_pos = rddf[init_pos_id]
    # init_pos = rddf[int(len(rddf) / 2)]
    # init_pos = rddf[0]
    # init_pos = rddf[-1000]
    init_pos = [0., 0., 0., 0., 0.]
    
    # carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
    carmen.reset_without_initial_pose()
    # print('Initialized!')
    
    time_since_last_print = time.time()

    # plan = None
    # plan_time = None
    n_steps_v_zero = 0
    episode_size = 0

    last_pos = init_pos[0], init_pos[1]
    travelled_dist = 0.
    
    while (not carmen.hit_obstacle()) and \
          (n_steps_v_zero < max_steps_stopped):
        carmen.handle_messages()
        pose_data = carmen.read_pose()

        if False:
            laser = carmen.read_laser()
            rddf_data = carmen.read_rddf()
            goal_data = carmen.read_goal()
            
            if len(goal_data) == 0:
                print('Empty goal list. Reinitializing.')
                break

            state = compute_state(policy, pose_data, goal_data, laser, rddf_data)
            nn_v, nn_phi = policy.forward(state)
            nn_dt = 0.1 

            nn_v = filter_v(nn_v, pose_data[3], nn_dt, laser_data)
        
            vs = [nn_v] * 5
            phis = [nn_phi] * 5
            ts = [nn_dt] * 5
        else:
            vs, phis, ts = generate_plan(policy, pose_data, n_commands=30, dt=0.2)
            
#             cv2.imshow('img', np.zeros((200, 200)))
#             c = cv2.waitKey(1)
#             
#             if c == ord('w'):
#                 vs = [policy.config['max_v']] * len(ts) 
#             elif c == ord('s'):
#                 vs = [-policy.config['max_v']] * len(ts) 
#             elif c == ord('a'):
#                 phis = [-policy.config['max_phi']] * len(ts)
#             elif c == ord('d'):
#                 phis = [policy.config['max_phi']] * len(ts)
#             
#         travelled_dist += ((last_pos[0] - pose_data[0]) ** 2 + (last_pos[1] - pose_data[1]) ** 2) ** .5
#         last_pos = pose_data[0], pose_data[1] 
        
        if len(vs) > 0:
            carmen.publish_command(vs, phis, ts, True, pose_data[0], pose_data[1], pose_data[2])
        else:
            print('Ignoring empty command list!')

        episode_size += 1
        
    # print("Episode done.")
    carmen.publish_stop_command()

    n_episodes += 1

    collided = False
    if carmen.hit_obstacle(): 
        collided = True
        n_collisions += 1

#     print('Episode:', n_episodes, 
#           'Collided:', collided, 
#           'n_collisions:', n_collisions, 
#           'Episode size:', episode_size,
#           'Travelled dist:', travelled_dist)
