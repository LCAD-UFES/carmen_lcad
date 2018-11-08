
import numpy as np
from rl.util import normalize_theta, find_nearest_pose


class AbstractGoalSampler:
    def reset(self, state):
        raise NotImplementedError()
    
    def update(self, state):
        raise NotImplementedError()

    def increase_difficulty(self):
        pass
    
    def decrease_difficulty(self):
        pass

    def level(self):
        return 0


class UniformSampler(AbstractGoalSampler):
    def __init__(self, n_dims, limits=None):
        assert n_dims >= 1
        self.n_dims = n_dims
        
        if limits is not None:
            assert len(limits) == n_dims
            self.limits = limits
        else:
            self.limits = [[-1e8, 1e8]] * self.n_dims
    
    def reset(self, state):
        self.goal = np.array([low + np.random.random() * (high - low) for low, high in self.limits])
        return self.goal
    
    def update(self, state):
        return self.goal


class GaussianSampler(AbstractGoalSampler):
    def __init__(self, mean, stddev, clip_stddev=False):
        self.mean = mean
        self.stddev = stddev
        self.clip_stddev = clip_stddev
         
    def reset(self, state):
        sample = np.random.randn(self.mean.size) * self.stddev

        if clip_stddev:
            sample = np.clip(sample, -stddev, stddev)
        
        self.goal = self.mean + sample 
        return self.goal
    
    def update(self, state):
        return self.goal


class StepSampler(AbstractGoalSampler):
    def __init__(self, n_dims, min_delta, max_delta, delta_step, distribution='uniform'):
        self.delta = min_delta
        self.init_delta = min_delta
        self.delta_step = delta_step
        self.max_delta = max_delta
        
        if distribution == 'uniform':
            self.sampler = UniformSampler(n_dims, [[-1, 1]] * n_dims)
        elif distribution == 'gaussian':
            # by setting the stddev to 1/3, 90% of the data will be in [-1, 1].
            stddev = np.array([1.] * n_dims) / 3.
            mean = np.zeros(n_dims)
            self.sampler = GaussianSampler(mean, stddev)
        else:
            msg = "Invalid distribution '%s' not implemented in PositionSampler."
            msg = msg % distribution
            raise Exception(msg)

    def _apply_delta(self, x):
        if self.max_delta == self.init_delta:
            return x
        else:
            return x * self.delta
    
    def reset(self, state):
        return self._apply_delta(self.sampler.reset(state))
    
    def update(self, state):
        return self._apply_delta(self.sampler.update(state))

    def increase_difficulty(self):
        self.delta += self.delta_step
        if self.delta > self.max_delta:
            self.delta = self.max_delta
    
    def decrease_difficulty(self):
        self.delta -= self.delta_step
        if self.delta < self.init_delta:
            self.delta = self.init_delta

    def level(self):
        if self.max_delta == self.init_delta:
            return 1.0
        else:
            return self.delta # (self.delta - self.init_delta) / (self.max_delta - self.init_delta)


class PoseSampler(AbstractGoalSampler):
    def __init__(self, n_dims, 
                 p_min_delta, p_max_delta, p_delta_step, 
                 o_min_delta, o_max_delta, o_delta_step, 
                 distribution='uniform'):
        # TODO: compute number of orientation components in higher dims
        if n_dims == 2:
            self.p_dims, self.o_dims = 2, 1
        elif n_dims == 3:
            self.p_dims, self.o_dims = 3, 3
        else:
            raise Exception('PoseSampler only supports 2d and 3d poses.')
        
        # position and orientation samplers
        self.p_sampler = StepSampler(self.p_dims, p_min_delta, p_max_delta, p_delta_step, distribution)
        self.o_sampler = StepSampler(self.o_dims, o_min_delta, o_max_delta, o_delta_step, distribution)
    
    def build_pose(self, state, p, o):
        pose = np.concatenate([p, o, [0]], axis=0)
        for i in range(self.p_dims, self.p_dims + self.o_dims):
            pose[i] = normalize_theta(pose[i])
        return pose
    
    def reset(self, state):
        p = self.p_sampler.reset(state)
        o = self.o_sampler.reset(state)
        return self.build_pose(state, p, o)
    
    def update(self, state):
        p = self.p_sampler.update(state)
        o = self.o_sampler.update(state)
        return self.build_pose(state, p, o)

    def increase_difficulty(self):
        self.p_sampler.increase_difficulty()
        self.o_sampler.increase_difficulty()
    
    def decrease_difficulty(self):
        self.p_sampler.decrease_difficulty()
        self.o_sampler.decrease_difficulty()

    def level(self):
        return self.p_sampler.level()


class RddfGoalSampler(AbstractGoalSampler):
    def __init__(self, rddf, allow_goals_behind, fix_goal=True, min_shift=1, max_shift=20, shift_step=1):
        self.rddf = rddf
        self.fix_goal = fix_goal
        self.min_shift = min_shift
        self.max_shift = max_shift
        self.shift_step = shift_step
        self.shift = min_shift
    
    def reset(self, state):
        print('pose:', state['pose'])
        id, _ = find_nearest_pose(state['pose'], self.rddf)
        self._set_goal_id(id)
        goal = self.rddf[self.goal_id]
        goal[3:] = 0.
        return goal[:4] 
    
    def update(self, state):
        if not self.fix_goal:
            # Assume the vehicle do not move too much between time steps, 
            # and just search in the neighborhood of the goal for efficiency.
            # As the returned position is relative to the sublist, 
            # we have to add 'st' to have the actual position in the full list.
            st = max(self.goal_id-20, 0)
            end = min(self.goal_id+20, len(self.rddf))
            id, _ = find_nearest_pose(state['pose'], self.rddf[st:end])
            id += st
            self._set_goal_id(id)
        goal = self.rddf[self.goal_id]
        goal[3:] = 0.
        return goal[:4] 

    def increase_difficulty(self):
        self.shift += self.shift_step
        if self.shift_step > self.max_shift:
            self.shift_step = self.max_shift
    
    def decrease_difficulty(self):
        self.shift -= self.shift_step
        if self.shift < self.min_shift:
            self.shift = self.min_shift

    def level(self):
        if self.max_shift == self.min_shift:
            return 1.0
        else:
            return self.shift # (self.shift - self.min_shift) / (self.max_shift - self.min_shift) 
    
    def _set_goal_id(self, id):
        self.goal_id = id + self.shift
        if self.goal_id >= len(self.rddf): 
            self.goal_id = len(self.rddf) - 1
    