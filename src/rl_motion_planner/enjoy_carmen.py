
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

policy = Policy(input_shape=[368], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=5,
                continuous_std=1e-7)

policy.load('data/model_immitation.ckpt')

rddf = read_rddf('rddf_ida_guarapari-20170403.txt')

carmen.init()
n_collisions = 0


def compute_command(policy, pose_data, goal_data, laser):
    pose = Transform2d(pose_data[0], pose_data[1], pose_data[2])
    v, phi = pose_data[3], pose_data[4]
    
    goal = Transform2d(goal_data[0], goal_data[1], goal_data[2])
    goal_v, goal_phi = goal_data[3], goal_data[4]

    state = policy.assemble_state(pose, goal, v, phi, goal_v, goal_phi, laser)
    v, phi = policy.forward(state)

    return v, phi


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


while True:
    # init_pos = rddf[np.random.randint(len(rddf))]
    # init_pos = rddf[int(len(rddf) / 2)]
    init_pos = rddf[0]
    # init_pos = rddf[7800]

    carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
    
    while not carmen.hit_obstacle():
        pose_data = carmen.read_pose()
        goal_data = carmen.read_goal()
        laser = carmen.read_laser()

        if len(goal_data) == 0:
            print('Invalid goal data. Waiting for valid data.')
            continue

        # v, phi = compute_command(policy, pose_data, goal_data, laser)
        # vs = [v]
        # phis = [phi]
        # ts = [t]
        init = time.time()
        vs, phis, ts = generate_plan(policy, pose_data)
        print('Time to generate plan: %.4lf' % (time.time() - init))
        
        carmen.publish_command(vs, phis, ts) # , pose_data[0], pose_data[1], pose_data[2])
        carmen.handle_messages()
        
    n_collisions += 1
    print('n_collisions:', n_collisions)

