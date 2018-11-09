
import numpy as np
from rl.util import dist, normalize_theta
from abc import abstractstaticmethod


class RewardGeneratorInterface:
    def reward(self, episode_size, obs, goal, achieved, is_final):
        # return a float
        raise NotImplementedError()


class RewardSparse(RewardGeneratorInterface):
    def __init__(self, punish_failure=True):
        self.punish_failure = punish_failure
    
    def reward(self, episode_size, obs, goal, achieved, is_final):
        if is_final:
            if achieved:
                return 1.0
            elif self.punish_failure:
                return -1.0
        return 0.0
        

class RewardSparseEfficiency(RewardGeneratorInterface):
    def __init__(self, max_steps, punish_failure=True):
        self.punish_failure = punish_failure
        self.max_steps = max_steps
    
    def reward(self, episode_size, obs, goal, achieved, is_final):
        if is_final:
            if achieved:
                return (self.max_steps - episode_size) / (self.max_steps - 1)
            elif self.punish_failure:
                return -1.
        return 0.


class RewardStepPunishment(RewardGeneratorInterface):
    def __init__(self, max_steps):
        self.max_steps = max_steps

    def reward(self, episode_size, obs, goal, achieved, is_final):
        return (-1.0 / self.max_steps)


class RewardDistanceBased(RewardGeneratorInterface):
    def __init__(self, dist_weight=1.0, th_weight=0., velocity_weight=0.):
        self.thw = th_weight
        self.vw = velocity_weight
        self.dw = dist_weight
        
    def reward(self, episode_size, obs, goal, achieved, is_final):
        p = obs['pose']
        d = self.dw * dist(p, goal)
        t = self.thw * np.abs(normalize_theta(p[2] - goal[2]))
        v = self.vw * np.abs(p[3] - goal[3])
        return -(d + t + v)


class RewardGroup(RewardGeneratorInterface):
    def __init__(self, list_reward_generators):
        self.generators = list_reward_generators
    
    def reward(self, episode_size, obs, goal, achieved, is_final):
        assert n_steps_to_goal >= 1
        r = 0.
        for g in self.generators:
            r += g.reward(episode_size, obs, goal, achieved, is_final)
        return r


class RewardFactory:
    @staticmethod
    def _create_one(name, params):
        if name == 'sparse':
            r_generator = RewardSparse(params['punish_failure'])
        elif name == 'sparse_efficiency':
            r_generator = RewardSparseEfficiency(params['max_steps'], params['punish_failure'])
        elif name == 'step':
            r_generator = RewardStepPunishment(params['max_steps'])
        elif name == 'distance':
            r_generator = RewardDistanceBased(params['dist_weight'], 
                                              params['th_weight'], 
                                              params['velocity_weight'])
        else:
            raise Exception('Reward type "%s" not defined.' % name)
        return r_generator
    
    @staticmethod
    def create(params):
        name = params['reward_type']
        if ',' in name:
            names = name.rsplit(',')
            generators = [RewardFactory._create_one(n, params) for n in names]
            return RewardGroup(generators)
        else:
            return RewardFactory._create_one(name, params)

