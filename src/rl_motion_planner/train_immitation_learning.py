
import gym
import rl
from rl.runner import rollout
from rl.policy import Policy

env = gym.make('CarmenSim-v0')

policy = Policy(input_shape=env.observation_space.shape, 
                n_outputs=env.action_space.shape[0], 
                hidden_size=64, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=3,
                continuous_std=1e-7)

policy.train_immitation_learning(dataset_path='data/dataset_commands.txt', 
                                 n_epochs=20000, 
                                 n_batches_by_epoch=500, 
                                 batch_size=128)

print('Immitation learning done.')

policy.save('data/model_immitation_long.ckpt')

"""
for episode_id in range(50):
    episode = rollout(env, policy)
    print('Episode:', episode_id, 'Return:', episode['return'],  'n_transitions:', len(episode['states']))
"""
