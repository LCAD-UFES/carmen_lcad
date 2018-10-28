
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
plt.ion()
plt.show()


class ActionTransformerInterface:
    def transform(self, cmd, observation, id=0):
        raise NotImplementedError()


class IdentityTransformer(ActionTransformerInterface):
    def transform(self, cmd, observation, id=0):
        return cmd


class GaussianTransformer(ActionTransformerInterface):
    def __init__(self, std):
        self.std = std
    def transform(self, cmd, observation, id=0):
        return cmd + np.random.randn(len(cmd)) * self.std 


class EpsRandomTransformer(ActionTransformerInterface):
    def __init__(self, rate):
        self.rate = rate
    def transform(self, cmd, observation, id=0):
        if np.random.random() < self.rate:
            return np.random.uniform(-1.0, 1.0, len(cmd))
        return cmd


class ClipTransformer(ActionTransformerInterface):
    def __init__(self, a_min, a_max):
        self.min = a_min
        self.max = a_max
    def transform(self, cmd, observation, id=0):
        return np.clip(a=cmd, a_min=self.min, a_max=self.max)


# move to utils
class Plot:
    Figure_ids = dict()
    Figure_count = 0
    
    @staticmethod
    def plot(name, x, y, x_detailed, y_detailed, cmd_x, cmd_y):
        if name in Plot.Figure_ids.keys():
            figid = Plot.Figure_ids[name] 
        else:
            Plot.Figure_count += 1
            Plot.Figure_ids[name] = Plot.Figure_count
            figid = Plot.Figure_count
        
        plt.figure(num=figid, clear=True)
        plt.title(name)
        plt.ylim(-1.2, 1.2)
        plt.plot(x, y, 'o')
        plt.plot([cmd_x], [cmd_y], 'or')
        plt.plot(x_detailed, y_detailed, '-b')
        plt.draw()
        plt.pause(0.001)


class CubicSplineTransformer(ActionTransformerInterface):
    def __init__(self, spline_command_t, view_spline):
        self.spline_command_t = spline_command_t
        self.view_spline = view_spline
        self.x = np.arange(start=0., stop=5.5, step=5./3.)
        self.x_detailed = np.arange(start=0., stop=5.1, step=0.1)
        self.plot_starting_figure=10

    def transform(self, cmd, observation, id=0):
        assert len(cmd) % 3 == 0
        n_commands = int(len(cmd) / 3)
        out_cmd = np.zeros(n_commands)
        base = [observation['pose'][3], observation['pose'][4]] # TODO: here we assume there are at most two commands...
        for i in range(n_commands):
            poinx = cmd[3*i : 3*(i+1)]
            y = [base[i], poinx[0], poinx[1], poinx[2]]
            spl = CubicSpline(self.x, y)
            out_cmd[i] = spl(self.spline_command_t)

            if self.view_spline:
                y_detailed = spl(self.x_detailed)
                Plot.plot("cubic_%d_%d" % (id, i), self.x, y, self.x_detailed, y_detailed, self.spline_command_t, out_cmd[i])

        return out_cmd


class LinearSplineTransformer(ActionTransformerInterface):
    def __init__(self, spline_command_t, view_spline):
        self.spline_command_t = spline_command_t
        self.view_spline = view_spline
        # 1.66 is the first point of the cubic spline
        self.x = self.x_detailed = [0., 1.66]

    def transform(self, cmd, observation, id=0):
        n_commands = int(len(cmd))
        out_cmd = np.zeros(n_commands)
        base = [observation['pose'][3], observation['pose'][4]] # TODO: here we assume there are at most two commands...
        for i in range(n_commands):
            c = (cmd[i] - base[i]) * (self.spline_command_t / 1.66) + base[i]
            y = y_detailed = [base[i], cmd[i]]
            out_cmd[i] = c

            if self.view_spline:
                Plot.plot("linear_%d_%d" % (id, i), self.x, y, self.x_detailed, y_detailed, self.spline_command_t, out_cmd[i])

        return out_cmd


class ActionTransformerFactory:
    @staticmethod
    def create(name, params=None):
        if name == 'identity':
            return IdentityTransformer()
        elif name == 'gaussian':
            return GaussianTransformer(std=params['noise_std'])
        elif name == 'random':
            return EpsRandomTransformer(rate=params['random_eps'])
        elif name == 'clip':
            return ClipTransformer(a_min=params['action_clip_min'], a_max=params['action_clip_max'])
        elif name == 'cubic_spline':
            return CubicSplineTransformer(params['spline_command_t'], params['view_spline'])
        elif name == 'linear_spline':
            return LinearSplineTransformer(params['spline_command_t'], params['view_spline'])
        else:
            raise Exception("ActionTransformer '%s' not implemented!" % name)


class ActionTransformerGroup(ActionTransformerInterface):
    def __init__(self, transformers, param_list):
        is_multidim = type(transformers[0]) == list

        if not is_multidim:
            transformers = [transformers]
            param_list = [param_list]
        
        self.queues = []
        for i in range(len(transformers)):
            name_list = transformers[i]
            params = param_list[i]
            q = [] 
            for name in name_list:
                q.append(ActionTransformerFactory.create(name, params))
            self.queues.append(q)

    def transform(self, cmd, observation):
        splitted = np.split(np.array(cmd), len(self.queues))

        transformed = []
        
        for i in range(len(splitted)):
            c = splitted[i]
            q = self.queues[i]
            for t in q:
                c = t.transform(c, observation, id=i)
            transformed.append(c)
        
        return np.concatenate(transformed)


if __name__ == "__main__":
    cmd = [-1., 1.]
    cmd_spline = [1., -1., 1., 1., -1., 1.]
    obs = [0., 0., 0., 0., 0.] 

    print('Identity:', ActionTransformerFactory.create('identity').transform(cmd, obs))
    print('GaussianNoise:', ActionTransformerFactory.create('gaussian', {'noise_std': 0.1}).transform(cmd, obs))
    print('Random:', ActionTransformerFactory.create('random', {'random_eps': 1.0}).transform(cmd, obs))
    print('Clip:', ActionTransformerFactory.create('clip', {'action_clip_min': -1.0, 'action_clip_max': 1.0}).transform(cmd, obs))
    print('Linear:', ActionTransformerFactory.create('linear_spline', {'spline_command_t': 0.2, 'view_spline': True}).transform(cmd, obs))
    print('Cubic:', ActionTransformerFactory.create('cubic_spline', {'spline_command_t': 0.2, 'view_spline': True}).transform(cmd_spline, obs))
    print('Group(Gaussian -> Clip)', ActionTransformerGroup(['gaussian', 'clip'], {'noise_std': 0.1, 'action_clip_min': 0., 'action_clip_max': 1.0}).transform(cmd, obs))

    params = [{'noise_std': 0.1, 'action_clip_min': 0., 'action_clip_max': 2.0}, 
              {'noise_std': 0.1, 'action_clip_min': -2., 'action_clip_max': 2.0}]
    
    print('Group(Gaussian -> Clip[0, 2], Gaussian -> Clip[-2, 2])', ActionTransformerGroup([['gaussian', 'clip'], 
                                                                                            ['gaussian', 'clip']], params).transform(cmd, obs))

    plt.waitforbuttonpress()

