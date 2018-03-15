
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
        # values from the dataset used for immitation learning.
        self.max_v = 10.049866
        self.max_phi = 0.254299

        carmen.env_init()
        self.rddf = self.read_rddf()
        
        state = self.reset()
        self.observation_space = spaces.Box(-np.inf, np.inf, len(state))
        self.action_space = spaces.Box(-1., 1., 2)
    
    def dist_to_goal(self, car_info):
        return ((car_info[0] - self.goal[0]) ** 2 + (car_info[1] - self.goal[1]) ** 2) ** 0.5
    
    def goal_in_car_ref(self, car_info):
        p = Transform2d(0., 0., car_info[2])
        g = Transform2d(self.goal[0] - car_info[0], self.goal[1] - car_info[1], self.goal[2])
        g = p.inverse().transform(g)
        return [g.x, g.y, g.th]
    
    def step(self, action):
        action = np.clip(action, -1., 1.)
        
        car_info = carmen.env_step(action[0] * self.max_v, action[1] * self.max_phi, 
                                  self.goal[0], self.goal[1], self.goal[2],
                                  self.goal[3], self.goal[4])
        
        d_goal = self.dist_to_goal(car_info)
        
        # rw = -d_goal / 100.0 
        rw = (self.prev_dist_to_goal - d_goal) / 10.0
        # rw = 0.0
        
        self.prev_dist_to_goal = d_goal
        
        collided = carmen.env_done()        
        if collided:
            rw -= 2.0
            
        goal_reached = d_goal < 1.0
        done = True if goal_reached or collided else False

        if goal_reached:
            rw += 1.0

        g = self.goal_in_car_ref(car_info)

        self.previous_commands.append(action[0])
        self.previous_commands.append(action[1])
        
        self.previous_commands.pop(0)
        self.previous_commands.pop(0)
        
        v, phi = car_info[-2] / self.max_v, car_info[-1] / self.max_phi
        # state = np.array(g + self.previous_commands + [v, phi])
        state = np.copy(g + [v, phi])
        # print('state: ', state, 'rw:', rw)
        
        return state, rw, done, None

    def reset(self):
        init_pos_id = np.random.randint(len(self.rddf) - 20.0)
        goal_id = init_pos_id + 20
        
        init_pos = self.rddf[init_pos_id]
        self.goal = self.rddf[goal_id]
        
        car_info = carmen.env_reset(init_pos[0], init_pos[1], init_pos[2],
                                    self.goal[0], self.goal[1], self.goal[2],
                                    self.goal[3], self.goal[4])
        
        self.prev_dist_to_goal = self.dist_to_goal(car_info)

        g = self.goal_in_car_ref(car_info)
        
        self.previous_commands = [0.0 for _ in range(20)]

        v, phi = car_info[-2] / self.max_v, car_info[-1] / self.max_phi
        # state = np.concatenate([g, self.previous_commands, [v, phi]])
        state = np.copy(g + [v, phi])
        
        return state
    
    def render(self, mode='human', close=False):
        pass


def register_carmen_sim():
    register(
        id='CarmenSim-v0',
        entry_point='rl.env:CarmenSimEnv',
        max_episode_steps=100
    )

