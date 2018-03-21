
import os
import numpy as np
from rl.policy import Policy
from rl.util import Transform2d
import carmen_comm.carmen_comm as carmen

# ###############################################################################
# Copied from rl.env. TODO: improve.
# ###############################################################################

def read_rddf():
    carmen_path = os.environ['CARMEN_HOME']
    # rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
    rddf_path = carmen_path + '/data/rndf/rddf_ida_guarapari-20170403.txt'
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf

# ###############################################################################

policy = Policy(input_shape=[368], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=5,
                continuous_std=1e-7)

policy.load('data/model_immitation.ckpt')

rddf = read_rddf()
carmen.env_init()
n_collisions = 0

while True:

    init_pos = rddf[np.random.randint(len(rddf))]
    # init_pos = rddf[int(len(rddf) / 2)]
    # init_pos = rddf[0]
    # init_pos = rddf[7800]

    carmen.env_reset(init_pos[0], init_pos[1], init_pos[2])
    
    while not carmen.env_done():
        state = carmen.read_state()
        goal_data = carmen.read_goal()
        readings = carmen.read_laser()

        pose = Transform2d(state[0], state[1], state[2])
        v, phi = state[3], state[4]
        
        goal = Transform2d(goal_data[0], goal_data[1], goal_data[2])
        goal_v, goal_phi = goal_data[3], goal_data[4]
        
        state = policy.assemble_state(pose, goal, v, phi, goal_v, goal_phi, readings)
        cmd = policy.forward(state)
        
        carmen.env_step(cmd[0], cmd[1])

    n_collisions += 1
    carmen.env_step(0.0, 0.0)

    print('n_collisions:', n_collisions)

