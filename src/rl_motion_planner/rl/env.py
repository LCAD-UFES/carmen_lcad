
import numpy as np
from gym import Env, spaces
import carmen_comm.carmen_comm as carmen
from gym.envs.registration import register


class CarmenSimEnv(Env):
    metadata = {'render.modes': ['human']}
    
    def carmen_reset(self):
        return carmen.env_reset(7757732.33, -363558.89, 0.651)
    
    def __init__(self):
        carmen.env_init()
        state = self.carmen_reset()
        
        self.observation_space = spaces.Box(-np.inf, np.inf, len(state))
        self.action_space = spaces.Box(-1., 1., 1)
    
    def step(self, action):
        action = np.clip(action, -1., 1.)
        state = carmen.env_step(3.0, action[0] * np.deg2rad(28.))
        
        dist_goal = abs(state[0]) + abs(state[1])
        diff_theta = abs(state[2])
        diff_vel = abs(state[3] - state[4])

        rw = -(dist_goal + diff_theta + diff_vel) / 2000.0
        done = False

        return np.array(state), rw, done, None
    
    def reset(self):
        return np.array(self.carmen_reset())
    
    def render(self, mode='human', close=False):
        pass


def register_carmen_sim():
    register(
        id='CarmenSim-v0',
        entry_point='rl.env:CarmenSimEnv',
        max_episode_steps=1000
    )

