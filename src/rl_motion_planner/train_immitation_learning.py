
import gym
import rl
from rl.runner import rollout
from rl.policy import Policy

policy = Policy(input_shape=[5], 
                n_outputs=2, 
                hidden_size=64, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=3,
                continuous_std=1e-7)

policy.train_immitation_learning(dataset_path='data/dataset_commands.txt', 
                                 n_epochs=20000, 
                                 n_batches_by_epoch=500, 
                                 batch_size=128)

print('Immitation learning done.')

policy.save('data/model_immitation_long.ckpt')

