
import time
import os
import numpy as np
import cv2
from pycarmen import CarmenComm, CarmenSim, CarPanel
from rl.util import dist, ackerman_motion_model, draw_rectangle, normalize_theta, find_nearest_pose

from scipy.interpolate import CubicSpline
#import matplotlib.pyplot as plt
#plt.ion()
#plt.show()
from rl.envs import AbstractEnv
from rl.goal_samplers import RddfGoalSampler


def read_rddf(name):
    carmen_path = os.environ['CARMEN_HOME']
    rddf_path = carmen_path + '/data/rndf/' + name
    return np.array([[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()])


class Carmen(AbstractEnv):
    def __init__(self, params, min_steps_to_initial_goal=20, max_steps_to_initial_goal=20, shift_step=1, 
                 max_speed_forward=10., max_speed_backward=-10., sim_dt=0.05, 
                 goal_achievement_dist=0.5, velocity_achievement_dist=-1, 
                 max_acceleration=1.0, max_phi_velocity=np.deg2rad(5.), mode='online'):
        
        self.use_truepos = True
        self.enjoy_mode = not params['train']
        self.mode = mode
        self.params = params
        
        self.rddf = read_rddf(params['rddf'])
        goal_sampler = RddfGoalSampler(self.rddf, 
                                       params['allow_goals_behind'], 
                                       params['fix_goal'], 
                                       min_steps_to_initial_goal, 
                                       max_steps_to_initial_goal, 
                                       shift_step)
        
        super().__init__(params, goal_sampler)
        self._init_carmen()

        self.sim_dt = sim_dt
        self.max_speed_forward = max_speed_forward
        self.max_speed_backward = max_speed_backward
        self.max_phi = np.deg2rad(28.)
        
        self.goal_achievement_dist = goal_achievement_dist
        self.velocity_achievement_dist = velocity_achievement_dist

        # only used if params['use_acceleration'] is True
        self.max_acceleration_v = max_acceleration  # m/s^2
        self.max_velocity_phi = max_phi_velocity  # rad/s 
        self.use_acceleration = params['use_acceleration']

        if params['view'] and self.mode != 'online':
            self.panel = CarPanel()

    def seed(self, seed):
        super().seed(seed)
        if self.mode != 'online':
            self.sim.set_seed(seed)

    def goal_reached(self, state, goal):
        achieved_goal = dist(state['pose'], goal) < self.goal_achievement_dist
        
        vel_is_correct = True
        if self.velocity_achievement_dist > 0:
            vel_is_correct = np.abs(state['pose'][3] - goal[3]) < self.velocity_achievement_dist

        return (achieved_goal and vel_is_correct)

    def finalize(self):
        if self.mode == 'online':
            carmen.publish_stop_command()

    def _init_carmen(self):
        if self.mode == 'online':
            print('Connecting to carmen')
            CarmenComm.init()
            assert CarmenComm.config_rear_laser_is_active()
        else:
            self.sim = CarmenSim(self.params['fix_initial_position'],
                         self.use_truepos, 
                         self.params['allow_negative_velocity'], 
                         self.enjoy_mode,
                         self.params['use_latency'],
                         self.params['rddf'],
                         self.params['map'])
    
    def _state(self):
        if self.mode == 'online':
            CarmenComm.handle_messages()
            laser = CarmenComm.read_laser()
            pose = CarmenComm.read_truepos()
        else:
            laser = self.sim.laser()
            pose = self.sim.pose()

        pose = np.array(pose)        
        if pose[3] > 0:
            pose[3] /= self.max_speed_forward
        else:
            pose[3] /= self.max_speed_backward
        pose[4] /= self.max_phi    
        
        state = {
            'pose': pose,
            'laser': np.array(laser).reshape(len(laser), 1),
        }
        
        return state

    def _hit_obstacle(self):
        if self.mode == 'online':
            return CarmenComm.hit_obstacle()
        else:
            return self.sim.hit_obstacle()

    def _reset_state(self):
        if self.mode == 'online':
            init_pos = self.rddf[np.random.randint(len(self.rddf))]
            CarmenComm.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])
        else:
            self.sim.reset()
        
        self.v = 0
        self.phi = 0
        
        self.state = self._state()
        self.hit_obstacle = self._hit_obstacle()

    def _done(self):
        if self.hit_obstacle:
            if self.mode == 'online':
                CarmenComm.publish_stop_command()
            return True
        return super()._done()
        
    def _info(self):
        info = super()._info()
        info['hit_obstacle'] = self.hit_obstacle
        return info

    def _view(self):
        if self.mode != 'online':
            p = self.sim.pose()
            g = self.goal
            
            self.sim.draw_occupancy_map()
            self.sim.draw_pose(p[0], p[1], p[2], 0, 0, 0)
            self.sim.draw_pose(g[0], g[1], g[2], 0, 255, 0)
            self.sim.show()
            
            sim_t = self.n_steps * self.sim_dt
            self.panel.draw(p[3], p[4], sim_t)

    def _update_state(self, cmd):
        if self.mode == 'online':
            CarmenComm.publish_goal_list([self.goal[0]], 
                                            [self.goal[1]], 
                                            [self.goal[2]], 
                                            [self.goal[3]], 
                                            [0.0], 
                                            time.time())

        if self.use_acceleration:
            self.v += cmd[0] * self.max_acceleration_v * self.sim_dt
            self.phi += cmd[1] * self.max_velocity_phi * self.sim_dt
        
        else:
            if cmd[0] > 0: 
                d_v = cmd[0] * self.max_speed_forward
            else: 
                d_v = cmd[0] * np.abs(self.max_speed_backward)
            
            d_phi = cmd[1] * self.max_phi
            self.v += d_v 
            self.phi = normalize_theta(self.phi + d_phi)   

        self.v = np.clip(self.v, a_min=self.max_speed_backward, a_max=self.max_speed_forward)
        self.phi = np.clip(self.phi, a_min=-self.max_phi, a_max=self.max_phi)

        if self.mode == 'online':
            CarmenComm.publish_command([self.v] * 10, [self.phi] * 10, [self.sim_dt] * 10, True)
        else:
            self.sim.step(self.v, self.phi, self.sim_dt)
        
        self.state = self._state()
        self.hit_obstacle = self._hit_obstacle()
