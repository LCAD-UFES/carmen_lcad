"""
###############################################################################################
TODO: Os caras da OpenAI treinaram no final de cada episodio. Tentar treinar a cada passo?
TODO: Adicionar codigo para gerar graficos.
TODO: Adicionar verificacao de erros (e.g., NANs no tensorflow).
TODO: Adicionar goals com diferentes velocidades e diferentes velocidades inciais. Levar em conta 
    o maximo que o carro consegue mudar a velocidade dada a distancia do ponto para o goal.
TODO: Checar se nao estao acontecendo copias que podem causar bugs.
TODO: Checar se os codigos de amostragem de batches podem ser mais eficientes.
###############################################################################################
"""

import os
import sys
import random
import time
import numpy as np
import copy

import tensorflow as tf

from sacred import Experiment
from sacred.observers import FileStorageObserver

from rl.ddpg import DDPG
from rl.util import relative_pose, numlist2str
from rl.envs import *
from rl.rewards import RewardFactory
from rl.replay_buffer import ReplayBuffer
from rl.action_transformers import ActionTransformerGroup


results_dir = 'results/'
ex = Experiment('rl_motion_planner')
ex.observers.append(FileStorageObserver.create(results_dir))


def print_report_and_view(transitions, obs, goal, g, cmd, q, env, view_active, view_sleep):
    if len(transitions) % 25 == 0:
        print('Step:', len(transitions),
              'Pose:', numlist2str(obs['pose']), 
              'Goal:', numlist2str(goal),
              'RelativeGoal:', numlist2str(g), 
              'Cmd:', numlist2str(cmd),
              'Q:', q)

    if view_active:
        env.view()
        time.sleep(view_sleep)


def run_one_episode(policy, update_goal, env, view_active, view_sleep, 
                    frame_skip, exploration_transformers, additional_transformers, 
                    use_target_net=False):
    obs, goal = env.reset()

    # even if the environment changes the goal internally, 
    # we keep using the first goal (except if update_goal is True) .
    goal = copy.deepcopy(goal)

    transitions = []
    done = False
 
    while not done:
        # we have to make a deep copy before interacting with the environment because if the env returns references
        # (instead of a copy) to the observation, internal changes in the step method would cause changes to the variable.
        obs = copy.deepcopy(obs)

        g = relative_pose(obs['pose'], goal)
        cmd, q = policy.get_actions(obs, g, use_target_net)
        exploration_cmd = exploration_transformers.transform(cmd, obs)
        final_command = additional_transformers.transform(exploration_cmd, obs)
        
        for _ in range(frame_skip):
            new_obs, new_goal, done, info = env.step(final_command)
        
        print_report_and_view(transitions, obs, goal, 
                              g, cmd, q, env, 
                              view_active, 
                              view_sleep)

        transitions.append([obs, exploration_cmd])

        obs = new_obs
        if update_goal:
            goal = copy.deepcopy(new_goal)

    env.finalize()
    
    episode = {
        'transitions': transitions,
        'goal': goal,
        'success': info['success'],
        'hit_obstacle': info['hit_obstacle'],
        'level': env.goal_sampler.level(),
    }

    return episode


def run_episodes(policy, update_goal, env, n_rollouts,  
                 view_active, view_sleep, frame_skip, 
                 exploration_transformers, additional_transformers, 
                 use_target_net=False):
    episodes = []
    
    while len(episodes) < n_rollouts:
        episode = run_one_episode(policy, update_goal, env, view_active, view_sleep, 
                                  frame_skip, exploration_transformers,
                                  additional_transformers, 
                                  use_target_net=use_target_net)
        
        # TODO: Check if the robot started out of a valid map.
        if len(episode['transitions']) == 1 and not episode['success']:
            print("Episode starting in invalid pose. Trying again.")
            continue

        episodes.append(episode)

    return episodes


# Only for debugging purposes
def print_episode(episode):
    print('-----------------------------------------')
    
    print('Goal:', episode['goal'], 
          'Success:', episode['success'], 
          'Hit_obstacle:', episode['hit_obstacle'])
    
    i = 0
    for transition in episode['transitions']:
        obs = transition[0]
        cmd = transition[1]
        rw = transition[2]
        
        print(i, end=' ')
        print(obs['pose'], cmd, rw, '\n')
        i += 1

    print('-----------------------------------------')
    input('Pressione enter para continuar:')


def create_env(params):
    name = params['env']
    if name == 'simple': 
        return EmptyRoomSimple(params)
    elif name == 'ackerman':
        return EmptyRoomAckerman(params)
    elif name == 'carmen_offline': 
        return Carmen(params, mode='offline', sim_dt=params['sim_dt'])
    elif name == 'carmen_online': 
        return Carmen(params, mode='online', sim_dt=params['sim_dt'])
    else: 
        raise Exception("Env '{}' not implemented.".format(name))


def set_seeds(env, seed):
    # Note: it is not possible to make the results reproducible when using
    # Carmen in online mode (with message passing).
    tf.set_random_seed(seed)
    np.random.seed(seed)
    random.seed(seed)
    env.seed(seed)


def create_policy(params, env):
    # read an state and goal to read their 
    # dimensions when creating the neural net.
    state, goal = env.reset()
    
    # create the policy
    policy = DDPG(params, state, goal)
    
    # load a pre-trained network if available
    checkpoint = params['checkpoint']
    if len(checkpoint) > 0:
        policy.load(checkpoint)
    
    return policy


def create_exploration_transformers(params):
    p_v = copy.deepcopy(params)
    if not p_v['allow_negative_velocity']:
        p_v['action_clip_min'] = 0.
    
    filters = [params['v_filters'], params['phi_filters']]
    params = [p_v, params]
    return ActionTransformerGroup(filters, params)


def create_additional_transformers(params):
    p_v = copy.deepcopy(params)
    if not p_v['allow_negative_velocity']:
        p_v['action_clip_min'] = 0.

    all_params = [p_v, params]

    if params['spline_type'] == 'linear':
        filters = [['linear_spline', 'clip'], ['linear_spline', 'clip']]
        t = ActionTransformerGroup(filters, all_params) 
    elif params['spline_type'] == 'cubic':
        filters = [['cubic_spline', 'clip'], ['cubic_spline', 'clip']]
        t = ActionTransformerGroup(filters, all_params) 
    else:
        filters = [['identity'], ['identity']]
        t = ActionTransformerGroup(filters, all_params)

    return t


def add_to_queue(queue, value, max_size):
    queue.append(value)
    if len(queue) > max_size: 
        queue.pop(0)


def update_rewards(episode, reward_generator):
    ep_return = 0.
    
    transitions = episode['transitions']
    success = episode['success']
    goal = episode['goal']
    n = len(transitions)
    
    for i in range(n):
        t = transitions[i]
        is_final = (i == (n - 1))
        r = reward_generator.reward(n, 
                                   obs=t[0], 
                                   goal=goal, 
                                   achieved=success,
                                   is_final=is_final)
        t.append(r)
        ep_return += r
        
    episode['return'] = ep_return


def compute_rewards_and_statistics(episodes, statistics, reward_generator):
    for e in episodes:
        add_to_queue(statistics['successes_lastk'], e['success'], max_size=30)
        add_to_queue(statistics['collisions_lastk'], e['hit_obstacle'], max_size=30)    
        add_to_queue(statistics['level_lastk'], e['level'], max_size=30)
        update_rewards(e, reward_generator)
        
    mean_success_lastk = np.mean(statistics['successes_lastk'])
    return mean_success_lastk 


def train(params, policy, episodes, replay_buffer):
    training_time = 0.

    if params['train']:
        replay_buffer.add(episodes)

        init_t = time.time()
        
        if len(replay_buffer) > params['min_buffer_size']:
            
            for batch_id in range(params['n_batches']):
                print_report = False
                if batch_id % 25 == 0:
                    print_report = True 

                policy.train(replay_buffer.get_batch(), print_report=print_report)

            policy.update_target_net()
            
        training_time = time.time() - init_t

    return training_time


def save_policy(policy, epoch, save_interval, mean_success, best_success_rate, experiment_dir):
    # save the policy from time to time
    if epoch % save_interval == 0:
        periodic_policy_path = os.path.join(experiment_dir, 'policy_%d.ckpt' % epoch)
        policy.save(periodic_policy_path)

    # save the policy if it improved.
    if mean_success >= best_success_rate:
        best_policy_path = os.path.join(experiment_dir, 'policy_best.ckpt')
        policy.save(best_policy_path)
        best_success_rate = mean_success
        
    return best_success_rate


def print_report(epoch, n_rollouts, episodes, statistics, mean_success, train_time, experiment_time):
    # Print episodes (debug)
    # for e in episodes:
        # print_episode(e)

    curr_time = time.time()
    
    print('\n** Episode', epoch * n_rollouts,
          'Return:', episodes[0]['return'],
          'Success:', episodes[0]['success'],
          'Collision:', episodes[0]['hit_obstacle'],
          'SuccessAvg30:', mean_success,
          'Level:', statistics['level_lastk'][-1],
          'CollisionsAvg30:', np.mean(statistics['collisions_lastk']),
          'AvgEpSize:', np.mean([len(e['transitions']) for e in episodes]),
          'Training time:', train_time,
          'Overall time:',  experiment_time,
          end='\n\n')
    
    sys.stdout.flush()


@ex.config
def config():
    seed = 1
    n_epochs = 100000
    save_interval = 20

    params = {
        # experiment
        'use_gpu': True,
        'checkpoint': '',
        'train': True,
        
        # env
        'env': 'simple',
        'max_steps': 20,
        'rddf': 'rddf-voltadaufes-20170809.txt',
        'map': "map_ida_guarapari-20170403-2",
        'use_latency': False,
        'allow_negative_velocity': True,  # CHECK!!
        'allow_goals_behind': True,  # CHECK!!
        'fix_goal': True, # CHECK!
        'fix_initial_position': False,
        # to sum several rewards, use their names separated by commas
        'reward_type': 'sparse',
        'frames_by_step': 1,
        'sim_dt': 0.1,  # CHECK IF USED!!
        'commands_are_derivatives': False,  # TODO!

        # GUI
        'view': True,
        'view_sleep': 0,
        'view_spline': True,
        
        # net
        'n_critic_hidden_neurons': 64,
        'n_actor_hidden_neurons': 64,
        'n_critic_hidden_layers': 1,
        'n_actor_hidden_layers': 1,
        'soft_update_rate': 0.95,
        'use_conv_layer': False,
        'activation_fn': 'elu',
        'critic_lr': 1e-3,
        'actor_lr': 1e-3,
        'laser_normalization_weight': 1.0,
        'position_normalization_weight': 1.0,
        'v_normalization_weight': 1.0,
        'angle_normalization_weight': 1.0,
        'critic_training_method': 'td',  # td or mc   # TODO MC!!  
        
        # tricks
        'actor_gradient_clipping': 0.0,
        'critic_gradient_clipping': 0.0,
        'use_layer_norm': True,        # TODO!!
        'l2_regularization_critic': 0.0,
        'l2_regularization_actor': 0.0,
        'l2_cmd': 0.0,
        
        # training
        'n_rollouts': 1,
        'n_batches': 50,
        'batch_size': 256,
        'use_her': True,  # CHECK!!
        'use_sparse': True,  # CHECK!!
        'her_rate': 1.0,
        'replay_memory_capacity': 500,  # episodes
        'min_buffer_size': 10,
        'use_curriculum': True,  # TODO!!
        
        # Commands
        'n_commands': 2,
        'random_eps': 0.1,  # percentage of times a random action is taken
        'noise_std': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
        'spline_command_t': 0.15,
        'spline_type': None,  # None, linear, or cubic.  # TODO!!
        'v_filters': ['random', 'gaussian', 'clip'],
        'phi_filters': ['random', 'gaussian', 'clip'],
        'action_clip_max': 1.0,
        'action_clip_min': -1.0,
        
        # Reward generator params
        'punish_failure': True,
        'dist_weight': 1. / 100., 
        'th_weight': 0.0, 
        'velocity_weight': 0.0   # THERE IS A PARAMETER WITH ALMOST THE SAME NAME!!
    }

    params['gamma'] = 1. - 1. / params['max_steps']


@ex.named_config
def simple():
    params = {}
    params['env'] = 'simple'
    params['n_critic_hidden_neurons'] = 8
    params['n_actor_hidden_neurons'] = 64
    params['use_conv_layer'] = False
    params['max_steps'] = 20
    params['soft_update_rate'] = 0.0
    params['activation_fn'] = 'elu'
    params['actor_lr'] = 1e-3
    params['critic_lr'] = 1e-3
    params['actor_gradient_clipping'] = 0.0
    params['critic_gradient_clipping'] = 0.0
    params['l2_regularization_critic'] = 0.0
    params['l2_regularization_actor'] = 0.0
    params['l2_cmd'] = 0.0
    params['gamma'] = 1. - 1. / params['max_steps']
    params['spline_type'] = None # 'linear'
    params['spline_command_t'] = 0.5
    params['use_curriculum'] = True


@ex.named_config
def ackerman():
    params = {}
    params['env'] = 'ackerman'
    params['n_critic_hidden_neurons'] = 32
    params['n_actor_hidden_neurons'] = 64
    params['use_conv_layer'] = False
    params['max_steps'] = 100
    params['soft_update_rate'] = 0.
    params['activation_fn'] = 'elu'
    params['actor_lr'] = 1e-4
    params['critic_lr'] = 1e-3
    params['actor_gradient_clipping'] = 0.0
    params['critic_gradient_clipping'] = 0.0
    params['l2_regularization_critic'] = 0.0
    params['l2_regularization_actor'] = 0.0
    params['l2_cmd'] = 0.0
    params['gamma'] = 1. - 1. / params['max_steps']
    params['spline_type'] = None # 'linear'
    params['spline_command_t'] = 0.5
    params['use_curriculum'] = True
    params['her_rate'] = 1.
    params['n_batches'] = 100
    params['batch_size'] = 64


@ex.named_config
def carmen_offline():
    params = {}
    params['env'] = 'carmen_offline'
    params['n_critic_hidden_neurons'] = 128
    params['n_actor_hidden_neurons'] = 64
    params['use_conv_layer'] = False
    params['max_steps'] = 100
    params['soft_update_rate'] = 0.95
    params['activation_fn'] = 'elu'
    params['actor_lr'] = 1e-4
    params['critic_lr'] = 1e-4
    params['actor_gradient_clipping'] = 0.0
    params['critic_gradient_clipping'] = 0.0
    params['l2_regularization_critic'] = 0.0
    params['l2_regularization_actor'] = 0.01
    params['l2_cmd'] = 0.0
    params['spline_type'] = None 
    params['spline_command_t'] = 0.15
    params['laser_normalization_weight'] = 1.0 / 30.
    params['position_normalization_weight'] = 1.0 / 10.
    params['v_normalization_weight'] = 1.0 / 10.
    params['angle_normalization_weight'] = 2.0
    params['gamma'] = 1. - 1. / params['max_steps']
    params['her_rate'] = 0.


@ex.named_config
def exploit():
    params = {}
    params['train'] = False
    params['view_sleep'] = 0.03
    params['random_eps'] = 0
    params['noise_std'] = 0


@ex.automain
def main(seed, n_epochs, save_interval, params):
    init_experiment = time.time()
    
    env = create_env(params)
    set_seeds(env, seed)
    reward_generator = RewardFactory.create(params)
    policy = create_policy(params, env)
    exploration_transformers = create_exploration_transformers(params)
    additional_transformers = create_additional_transformers(params)
    replay_buffer = ReplayBuffer(params, reward_generator)
    experiment_dir = os.path.join(results_dir, str(ex.current_run._id))

    n_rollouts = params['n_rollouts']
    update_goal = True if params['env'] == 'carmen_online' else False 

    best_success_rate = -1
    statistics = {
        'successes_lastk': [],
        'collisions_lastk': [],
        'level_lastk': [],
    }
    
    last_difficulty_update = 0

    for epoch in range(n_epochs):
        episodes = run_episodes(policy, update_goal, env, n_rollouts, 
                                params['view'], params['view_sleep'], 
                                params['frames_by_step'],
                                exploration_transformers,
                                additional_transformers)

        mean_success_lastk = compute_rewards_and_statistics(episodes, statistics, reward_generator)
        
        train_time = train(params, policy, episodes, replay_buffer)
        
        best_success_rate = save_policy(policy, epoch, save_interval, 
                                        mean_success_lastk, best_success_rate,
                                        experiment_dir)
        
        print_report(epoch, n_rollouts, episodes, statistics, 
                     mean_success_lastk, train_time, time.time() - init_experiment)

        if (len(statistics['successes_lastk']) >= 10) and ((epoch - last_difficulty_update) > 10):
            if mean_success_lastk >= 0.5:
                env.goal_sampler.increase_difficulty()
                last_difficulty_update = epoch
            elif mean_success_lastk <= 0.5:
                env.goal_sampler.decrease_difficulty()
                last_difficulty_update = epoch

            
