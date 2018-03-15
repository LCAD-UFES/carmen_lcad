
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
    rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf

def goal_in_car_ref(goal, car_info):
    p = Transform2d(0., 0., car_info[2])
    g = Transform2d(goal[0] - car_info[0], goal[1] - car_info[1], goal[2])
    g = p.inverse().transform(g)
    return [g.x, g.y, g.th]

max_v = 10.049866
max_phi = 0.254299

# ###############################################################################

policy = Policy(input_shape=[5], 
                n_outputs=2, 
                hidden_size=64, learning_rate=1e-3, nonlin='elu',
                single_thread=False, n_hidden_layers=3,
                continuous_std=1e-5)

policy.load('data/model_immitation.ckpt')

rddf = read_rddf()
carmen.env_init()

while True:
    init_pos = rddf[np.random.randint(len(rddf))]
    carmen.env_reset(init_pos[0], init_pos[1], init_pos[2])
    
    while not carmen.env_done():
        state = carmen.read_state()
        goal = carmen.read_goal()
        goal = goal_in_car_ref(goal, state)
        
        cmd = policy.forward(goal + [state[-2], state[-1]])
        carmen.env_step(cmd[0] * max_v, cmd[1] * max_phi)
