
import os
import sys
import numpy as np
from gym import Env, spaces
import carmen_comm.carmen_comm as carmen
from gym.envs.registration import register
from rl.util import Transform2d


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
    
    def dist_to_goal(self, car_info):
        return ((car_info[0] - self.goal[0]) ** 2 + (car_info[1] - self.goal[1]) ** 2) ** 0.5
    
    def goal_in_car_ref(self, car_info):
        p = Transform2d(car_info[0] - car_info[0], car_info[1] - car_info[1], car_info[2])
        g = Transform2d(self.goal[0] - car_info[0], self.goal[1] - car_info[1], self.goal[2])
        g = p.inverse().transform(g)
        return [g.x, g.y, g.th]
    
    def step(self, action):
        action = np.clip(action, -1., 1.)
        car_info = carmen.env_step(5.0, action[0] * np.deg2rad(28.), 
                                  self.goal[0], self.goal[1], self.goal[2],
                                  self.goal[3], self.goal[4])
        
        d_goal = self.dist_to_goal(car_info)
        rw = (self.prev_dist_to_goal - d_goal) / 10.0
        self.prev_dist_to_goal = d_goal
        
        # done = carmen.env_done()        
        # if done:
            # rw -= 2.0
        done = True if d_goal < 1.0 else False

        g = np.array(self.goal_in_car_ref(car_info))

        return g, rw, done, None

    def reset(self):
        init_pos_id = np.random.randint(len(self.rddf) - 20.0)
        goal_id = init_pos_id + 20
        
        init_pos = self.rddf[init_pos_id]
        self.goal = self.rddf[goal_id]
        
        car_info = carmen.env_reset(init_pos[0], init_pos[1], init_pos[2],
                                    self.goal[0], self.goal[1], self.goal[2],
                                    self.goal[3], self.goal[4])
        
        self.prev_dist_to_goal = self.dist_to_goal(car_info)
        
        return np.array(self.goal_in_car_ref(car_info))
    
    def render(self, mode='human', close=False):
        pass


def register_carmen_sim():
    register(
        id='CarmenSim-v0',
        entry_point='rl.env:CarmenSimEnv',
        max_episode_steps=1000
    )

