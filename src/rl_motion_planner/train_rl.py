
import numpy as np
import gym

import rl
from rl.runner import rollout
from rl.policy import PolicyPG
import carmen_comm.carmen_comm as carmen


def main():
    env = gym.make('CarmenSim-v0')
    
    policy = PolicyPG(env.observation_space.shape, 
                      env.action_space.shape[0], 
                      hidden_size=32, 
                      n_hidden_layers=2, 
                      learning_rate=1e-3, 
                      nonlin='elu',
                      single_thread=True, 
                      continuous_std=0.01, 
                      continuous_action_max=1.0)
    
    returns = []
    
    for episode_id in range(5000):
        episode = rollout(env, policy)
        policy.train(episode)
        returns.append(episode['return'])
        print('Episode:', episode_id, 'Return:', episode['return'], 'Mean30:', np.mean(returns[-30:]))


if __name__ == "__main__":
    main()
