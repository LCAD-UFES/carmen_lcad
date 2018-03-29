
import os
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


rddf = read_rddf('rddf-voltadaufes-20170809.txt')

carmen.init()

n_collisions = 0

policy = Policy(input_shape=[98], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=4,
                continuous_std=1e-7)

policy.load('data/model_immitation.ckpt')

max_episode_size = np.inf
max_steps_stopped = np.inf 
n_episodes = 0

while True:
    # init_pos = rddf[np.random.randint(len(rddf))]
    # init_pos = rddf[int(len(rddf) / 2)]
    # init_pos = rddf[0]
    init_pos = rddf[-1000]

    carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
    time_since_last_print = time.time()

    episode = []
    plan = None
    plan_time = None
    n_steps_v_zero = 0
    algo = 'nn' # 'mpp' if n_episodes % 2 == 0 else 'nn' 
    
    while (not carmen.hit_obstacle()) and (len(episode) < max_episode_size) and (n_steps_v_zero < max_steps_stopped):
        carmen.handle_messages()
        pose_data = carmen.read_pose()
        goal_data = carmen.read_goal()
        laser = carmen.read_laser()
        rddf_data = carmen.read_rddf()
        commands = carmen.read_commands()

        if len(goal_data) == 0:
            print('Invalid goal data. Waiting for valid data.')
            continue

        state = compute_state(policy, pose_data, goal_data, laser, rddf_data)
        v, phi = policy.forward(state)

        if len(commands) > 0 and commands[0] > 0.01:
            plan = np.copy(commands)
            plan_time = time.time()

        cmd_v, cmd_phi, dt, valid = find_next_command(plan, plan_time)

        if (algo == 'nn' and abs(v) <= 0.0001) or (algo == 'mpp' and (abs(cmd_v) <= 0.0001 or valid is False)):
            n_steps_v_zero += 1
        else:
            n_steps_v_zero = 0

        if algo == 'mpp' and not valid:
            continue

        episode.append([state, [cmd_v, cmd_phi]])

        if algo == 'mpp':
            vs = [cmd_v] * 5
            phis = [cmd_phi] * 5
            ts = [dt] * 5
        else:
            vs = [v] * 20
            phis = [phi] * 20
            ts = [0.1] * 20

        if time.time() - time_since_last_print > 2.0:
            # """
            if len(commands) > 0:
                print('%s %.2f %.2f %.2f %.2f' % (algo, commands[0], commands[1], v, phi))
            else:
                print('Empty command.')
            # """
            
            time_since_last_print = time.time()
        
        # init = time.time()
        # vs, phis, ts = generate_plan(policy, pose_data)
        # print('Time to generate plan: %.4lf' % (time.time() - init))
        
        carmen.publish_command(vs, phis, ts, True) # , pose_data[0], pose_data[1], pose_data[2])

    print("Episode done.")

    carmen.publish_stop_command()
    continue 

    loss = 0.
    if len(episode) > 0:
        loss = policy.train_immitation_learning_episode(episode, 300, 256)

    n_episodes += 1
    if len(episode) < max_episode_size and n_steps_v_zero < max_steps_stopped:
        n_collisions += 1

    print('Episode:', n_episodes, 'n_collisions:', n_collisions, 'Episode size:', len(episode), 'loss:', loss)

