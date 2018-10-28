
import time 
import numpy as np


class AbstractEnv:
    def __init__(self, params, goal_sampler):
        self.view_active = params['view']
        self.view_sleep = params['view_sleep']
        self.max_steps = params['max_steps']
        self.goal_sampler = goal_sampler
    
    def reset(self):
        self.n_steps = 0
        self._reset_state()
        self.goal = self.goal_sampler.reset(self.state)
        return [self.state, self.goal]
    
    def step(self, cmd):
        self.n_steps += 1
        self._update_state(cmd)
        self.success = self.goal_reached(self.state, self.goal)
        self.goal = self.goal_sampler.update(self.state)
        return [self.state, self.goal, self._done(), self._info()]
    
    def view(self):
        if self.view_active:
            self._view()
            time.sleep(self.view_sleep)

    def _done(self):
        if self.n_steps >= self.max_steps:
            return True
        if self.success:
            return True
        return False
        
    def _info(self):
        return {'success': self.success, 'hit_obstacle': False}

    def seed(self, seed):
        np.random.seed(seed)

    def goal_reached(self, state, goal):
        return False

    def finalize(self):
        pass
    
    def _view(self):
        pass

    def _update_state(self, cmd):
        pass

    def _reset_state(self):
        self.state = {'pose': None, 'laser': None}
        raise NotImplementedError()
    
    
    
    