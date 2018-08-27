
import os
import sys
import random
import time
import numpy as np

import tensorflow as tf

from sacred import Experiment
from sacred.observers import FileStorageObserver

import carmen_comm.carmen_comm as carmen
from rl.ddpg import DDPG
from rl.util import relative_pose


results_dir = 'results/'
ex = Experiment('rl_motion_planner')
ex.observers.append(FileStorageObserver.create(results_dir))


def dist(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def print_evaluation_report(episodes, n_successes, n_collisions):
    # TODO.
    pass


def save_policy(policy, path):
    # TODO
    pass


def read_state():
    carmen.handle_messages()

    laser = carmen.read_laser()
    laser = np.clip(laser, 0.0, 30.0).reshape(len(laser), 1)
    laser = (laser / 30.0) * 2.0 - 1.0

    state = {
        'pose': carmen.read_pose(),
        'laser': laser,
    }

    return state


def reset(rddf):
    carmen.publish_stop_command()

    init_pos_id = np.random.randint(50, len(rddf) - 50)
    init_pos = rddf[init_pos_id]

    carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])

    forw_or_back = np.random.randint(2) * 2 - 1
    goal_id = init_pos_id + 20 # np.random.randint(10, 50) * forw_or_back
    goal = rddf[goal_id]

    carmen.publish_goal_list([goal[0]], [goal[1]], [goal[2]], [goal[3]], [0.0], time.time())

    return read_state(), goal[:4]


def step(cmd, goal, n_steps, params):
    carmen.publish_goal_list([goal[0]], [goal[1]], [goal[2]], [goal[3]], [0.0], time.time())

    v = cmd[0] * 10.0
    phi = cmd[1] * np.deg2rad(28.0)
    carmen.publish_command([v] * 10, [phi] * 10, [0.1] * 10, True)

    state = read_state()

    achieved_goal = dist(state['pose'], goal) < params['goal_achievement_dist']
    vel_is_correct = np.abs(state['pose'][3] - goal[3]) < params['vel_achievement_dist']

    hit_obstacle = carmen.hit_obstacle()
    starved = n_steps >= params['n_steps_episode']
    success = achieved_goal # and vel_is_correct

    done = success or hit_obstacle or starved
    info = {'success': success, 'hit_obstacle': hit_obstacle, 'starved': starved}

    return read_state(), done, info


def update_rewards(params, episode, info):
    rw = -1.0

    if info['success']:
        rw = float(params['n_steps_episode'] - len(episode)) / float(params['n_steps_episode'])
    elif info['hit_obstacle']:
        rw = -1.0

    rw /= len(episode)

    for transition in episode:
        transition[2] = rw


def generate_rollouts(policy, n_rollouts, params, rddf, exploit, use_target_net=False):
    # generate episodes
    episodes = []
    n_successes = 0
    n_collisions = 0

    while len(episodes) < n_rollouts:
        obs, goal = reset(rddf)
        print('Episode', len(episodes), 'pose:', obs['pose'], 'goal:', goal)
        episode = []
        done = False
        info = None

        while not done:
            g = relative_pose(obs['pose'], goal)

            cmd, q = policy.get_actions(obs, g + [goal[3]], noise_eps=params['noise_eps'] if not exploit else 0.,
                                        random_eps=params['random_eps'] if not exploit else 0.,
                                        use_target_net=use_target_net)

            if len(episode) % 20 == 0:
                print('Step', len(episode), 'Cmd:', cmd, 'Q:', q)

            new_obs, done, info = step(cmd, goal, len(episode), params)

            g_after = relative_pose(new_obs['pose'], goal)
            rw = (g[0] ** 2 + g[1] ** 2) - (g_after[0] ** 2 + g_after[1] ** 2)

            episode.append([obs, cmd, rw, goal])
            obs = new_obs

        carmen.publish_stop_command()
        if len(episode) == 0:
            continue

        print('done. info:', info)
        # update_rewards(params, episode, info)
        episodes.append(episode)

        if info['success']: n_successes += 1
        elif info['hit_obstacle']: n_collisions += 1

    return episodes, n_successes, n_collisions


def read_rddf(rddf):
    carmen_path = os.environ['CARMEN_HOME']
    rddf_path = carmen_path + '/data/rndf/' + rddf
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf


def launch(params, n_epochs, seed, policy_save_interval, clip_return, path_rddf, save_policies=True):
    """
    ###############################################################################################
    TODO: Os caras da OpenAI treinaram no final de cada episodio. Tentar treinar a cada passo?
    TODO: Adicionar codigo para gerar graficos.
    TODO: Adicionar verificacao de erros (e.g., NANs no tensorflow).
    TODO: Adicionar goals com diferentes velocidades e diferentes velocidades inciais. Levar em conta o maximo que o
        TODO: carro consegue mudar a velocidade dada a distancia do ponto para o goal.
    TODO: Colocar goal na referencia do carro (inclusive no HER), e normalizar dados de entrada (laser e velocidades).
    TODO: Checar se nao estao acontecendo copias que podem causar bugs.
    TODO: Checar se o codigo da amostragem de batches podem ser mais eficientes.
    ###############################################################################################
    """
    # It is not possible to set the seeds used by all carmen modules from here.
    tf.set_random_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    carmen.init()
    rddf = read_rddf(path_rddf)

    state, goal = reset(rddf)
    n_laser_readings = len(state['laser'])

    policy = DDPG(params, n_laser_readings=n_laser_readings)

    # Path to some log files
    experiment_dir = os.path.join(results_dir, str(ex.current_run._id))
    latest_policy_path = os.path.join(experiment_dir, 'policy_latest.pkl')
    best_policy_path = os.path.join(experiment_dir, 'policy_best.pkl')
    periodic_policy_path = os.path.join(experiment_dir, 'policy_{}.pkl')

    best_success_rate = -1

    for epoch in range(n_epochs):
        print("Training...")
        # train
        episodes, n_successes, n_collisions = generate_rollouts(policy, params['n_train_rollouts'],
                                                                params, rddf=rddf, exploit=False)
        policy.store_episode(episodes)
        for b in range(params['n_batches']):
            cl, pl = policy.train()
            if b % 50 == 0:
                print('Training batch', b, 'Critic loss:', cl, 'Policy loss:', pl)

        print('Updating target net.')
        policy.update_target_net()

        # test
        if params['n_test_rollouts'] > 0:
            print('Testing...')
            episodes, n_successes, n_collisions = generate_rollouts(policy, params['n_test_rollouts'],
                                                                    params, rddf=rddf, exploit=True)
            print_evaluation_report(episodes, n_successes, n_collisions)
            success_rate = float(n_successes) / float(len(episodes))
            collision_rate = float(n_collisions) / float(len(episodes))
            print('Evaluation success rate:', success_rate, 'collision_rate:', collision_rate)

            # save the policy if it's better than the previous ones
            if success_rate >= best_success_rate:
                best_success_rate = success_rate
                print('New best success rate: {}. Saving policy to {} ...'.format(best_success_rate, best_policy_path))
                save_policy(policy, best_policy_path)
                save_policy(policy, latest_policy_path)

            if policy_save_interval > 0 and epoch % policy_save_interval == 0 and save_policies:
                policy_path = periodic_policy_path.format(epoch)
                print('Saving periodic policy to {} ...'.format(policy_path))
                save_policy(policy, policy_path)


@ex.config
def config():
    seed = 1
    n_epochs = 100000

    # the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.
    policy_save_interval = 0

    # whether or not returns should be clipped (default = 1)
    clip_return = 1

    path_rddf = 'rddf-voltadaufes-20170809.txt'

    params = {
        # env
        'n_steps_episode': 200,
        'goal_achievement_dist': 1.0,
        'vel_achievement_dist': 0.5,
        # training
        'n_train_rollouts': 1,  # per epoch
        'n_batches': 400,  # training batches per cycle
        'batch_size': 256,  # per mpi thread, measured in transitions and reduced to even multiple of chunk_length.
        'n_test_rollouts': 0,  # number of test rollouts per epoch, each consists of rollout_batch_size rollouts
        # exploration
        'random_eps': 0.0,  # percentage of time a random action is taken
        'noise_eps': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
    }

    params['gamma'] = 1. - 1. / params['n_steps_episode']
    params['clip_return'] = (1. / (1. - params['gamma'])) if clip_return else np.inf


@ex.automain
def main(seed, n_epochs, policy_save_interval, clip_return, path_rddf, params):
    launch(params, n_epochs, seed, policy_save_interval, clip_return, path_rddf)
