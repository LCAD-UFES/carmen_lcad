
import gym
import rl
from rl.runner import rollout
from rl.policy import Policy

policy = Policy(input_shape=[368], 
                n_outputs=2, 
                hidden_size=128, learning_rate=1e-4, nonlin='elu',
                single_thread=False, n_hidden_layers=5,
                continuous_std=1e-7)

policy.train_immitation_learning(dataset_path='data/dataset_merged.txt', 
                                 n_epochs=1000, 
                                 n_batches_by_epoch=500, 
                                 batch_size=128,
                                 checkpoint_path='data/model_immitation.ckpt')

print('Immitation learning done.')

