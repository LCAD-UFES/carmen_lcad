
import os
import cv2
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
        laser = carmen.simulation_read_laser()

        if len(goal_data) == 0:
            print('Invalid goal data during simulation.')
            break

        state = compute_state(policy, pose_data, goal_data, laser, None)
        nn_v, nn_phi = policy.forward(state)

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
policy.load('data/model_online_immitation_learning.ckpt')

goal_achievement_dist = 1.0
max_steps_stopped = 20
n_episodes = 0

while True:
    # init_pos_id = np.random.randint(len(rddf))
    # init_pos = rddf[init_pos_id]
    # init_pos = rddf[int(len(rddf) / 2)]
    init_pos = rddf[0]
    # init_pos = rddf[-1000]
    
    carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
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
        goal_data = carmen.read_goal()
        laser = carmen.read_laser()
        rddf_data = carmen.read_rddf()

        if len(goal_data) == 0:
            print('Invalid goal data. Waiting for valid data. Reinitializing environment.')
            break

        if False:
            state = compute_state(policy, pose_data, goal_data, laser, rddf_data)
            nn_v, nn_phi = policy.forward(state)
            nn_dt = 0.1 
        
            vs = [nn_v] * 5
            phis = [nn_phi] * 5
            ts = [nn_dt] * 5

        else:
            vs, phis, ts = generate_plan(policy, pose_data, n_commands=30, dt=0.2)
            
        travelled_dist += ((last_pos[0] - pose_data[0]) ** 2 + (last_pos[1] - pose_data[1]) ** 2) ** .5
        last_pos = pose_data[0], pose_data[1] 
        
        carmen.publish_command(vs, phis, ts, True)
        episode_size += 1
        
    print("Episode done.")
    carmen.publish_stop_command()

    n_episodes += 1

    collided = False
    if carmen.hit_obstacle(): 
        collided = True
        n_collisions += 1

    print('Episode:', n_episodes, 
          'Collided:', collided, 
          'n_collisions:', n_collisions, 
          'Episode size:', episode_size,
          'Travelled dist:', travelled_dist)
