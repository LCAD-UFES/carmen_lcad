
import os
import time
import numpy as np
import carmen_comm.carmen_comm as carmen
from rl.util import Transform2d
from rl.ddpg import DDPG
from rl.util import relative_pose


carmen.init()

n_laser_readings = 361 * 2

params = {
    # env
    'env': 'carmen',
    'model': 'simple',
    'n_steps_episode': 300,
    'goal_achievement_dist': 1.0,
    'vel_achievement_dist': 0.5,
    'view': True,
    'rddf': 'rddf-voltadaufes-20170809.txt',
    'fix_initial_position': False,
    # net
    'n_hidden_neurons': 128,
    'n_hidden_layers': 1,
    'soft_update_rate': 0.75,
    'use_conv_layer': False,
    'activation_fn': 'elu',
    'allow_negative_commands': True,
    # training
    'n_rollouts': 1,
    'n_batches': 50,
    'batch_size': 256,
    'use_her': True,
    'her_rate': 1.0,
    'n_test_rollouts': 0.5,
    'replay_memory_capacity': 500,  # episodes
    # exploration
    'random_eps': 0.1,  # percentage of time a random action is taken
    'noise_eps': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
}

params['gamma'] = 1. - 1. / params['n_steps_episode']

policy = DDPG(params, n_laser_readings=n_laser_readings)
policy.load('results/345/policy_19300.ckpt')

while True:
    carmen.reset_without_initial_pose()

    while not carmen.hit_obstacle():
        carmen.handle_messages()

        pose_data = carmen.read_pose()
        laser = carmen.read_laser()
        laser = np.reshape(laser, (np.size(laser), 1))
        goal = carmen.read_goal()
        
        obs = {'laser': laser, 'pose': pose_data}
        g = relative_pose(obs['pose'], goal)
        
        cmd, q = policy.get_actions(obs, g + [goal[3]], noise_eps=0., random_eps=0., use_target_net=False)

        v = cmd[0] * 10.0
        phi = cmd[1] * np.deg2rad(28.)
        carmen.publish_command([v] * 10, [phi] * 10, [0.1] * 10, True)
        
    carmen.publish_stop_command()

