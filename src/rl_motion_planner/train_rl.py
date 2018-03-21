
import sys
import numpy as np
import gym

import rl
from rl.runner import rollout
from rl.policy import Policy
import carmen_comm.carmen_comm as carmen


def main():
    env = gym.make('CarmenSim-v0')
    
    policy = Policy(input_shape=[368], 
                    n_outputs=2, 
                    hidden_size=64, learning_rate=1e-4, nonlin='elu',
                    single_thread=False, n_hidden_layers=3,
                    continuous_std=1e-7)

    policy.load('data/model_immitation.ckpt')
    
    returns = []
    
    for episode_id in range(5000):
        episode = rollout(env, policy)
        policy.train(episode)
        returns.append(episode['return'])
        print('Episode:', episode_id, 'Return:', episode['return'], 'Mean30:', np.mean(returns[-30:]), 
              'n_transitions:', len(episode['states']), 'final obs:', episode['states'][-1])
        sys.stdout.flush()

if __name__ == "__main__":
    main()
