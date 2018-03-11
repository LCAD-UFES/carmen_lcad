
import os
import sys
import numpy as np
from gym import Env, spaces
import carmen_comm.carmen_comm as carmen
from gym.envs.registration import register


class CarmenSimEnv(Env):
    metadata = {'render.modes': ['human']}
    
    def read_rddf(self):
        carmen_path = os.environ['CARMEN_HOME']
        rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
        rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
        return rddf
    
    def __init__(self):
        carmen.env_init()
        self.rddf = self.read_rddf()
        
        state = self.reset()
        self.observation_space = spaces.Box(-np.inf, np.inf, len(state))
        self.action_space = spaces.Box(-1., 1., 1)
    
    def step(self, action):
        action = np.clip(action, -1., 1.)
        state = carmen.env_step(3.0, action[0] * np.deg2rad(28.), 
                                self.goal[0], self.goal[1], self.goal[2],
                                self.goal[3], self.goal[4])
        
        dist_goal = abs(state[0]) + abs(state[1])
        diff_theta = abs(state[2])
        diff_vel = abs(state[3] - state[4])

        rw = -(dist_goal + diff_theta + diff_vel) / 2000.0
        done = carmen.env_done()

        return np.array(state), rw, done, None

    def reset(self):
        init_pos_id = np.random.randint(len(self.rddf) - 30.0)
        goal_id = init_pos_id + 30
        
        init_pos = self.rddf[init_pos_id]
        self.goal = self.rddf[goal_id]
        
        state = carmen.env_reset(init_pos[0], init_pos[1], init_pos[2],
                                self.goal[0], self.goal[1], self.goal[2],
                                self.goal[3], self.goal[4])
        
        return np.array(state)
    
    def render(self, mode='human', close=False):
        pass


def register_carmen_sim():
    register(
        id='CarmenSim-v0',
        entry_point='rl.env:CarmenSimEnv',
        max_episode_steps=1000
    )

