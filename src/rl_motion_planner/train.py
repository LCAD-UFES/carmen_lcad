
import os
import sys
import random
import time
import numpy as np

import tensorflow as tf

from sacred import Experiment
from sacred.observers import FileStorageObserver

from rl.ddpg import DDPG
from rl.util import relative_pose, dist
from rl.envs import SimpleEnv, CarmenEnv


results_dir = 'results/'
ex = Experiment('rl_motion_planner')
ex.observers.append(FileStorageObserver.create(results_dir))


def update_rewards(params, episode, info):
    rw = -1.0

    if info['success']:
        rw = float(params['n_steps_episode'] - len(episode)) / float(params['n_steps_episode'])
        print("updated rewards:", rw)
    elif info['hit_obstacle']:
        rw = -1.0

    rw /= len(episode)
    print('rw:', rw)

    for transition in episode:
        transition[2] = rw


def generate_rollouts(policy, env, n_rollouts, params, exploit, use_her, use_target_net=False):
    # generate episodes
    episodes = []
    n_successes = 0
    n_collisions = 0
    q0 = 0.

    while len(episodes) < n_rollouts:
        obs, goal = env.reset()
        episode = []
        done = False
        info = None

        g = relative_pose(obs['pose'], goal)
        _, q0 = policy.get_actions(obs, g + [goal[3]], noise_eps=0.,
                                   random_eps=0.,
                                   use_target_net=False)

        while not done:
            g = relative_pose(obs['pose'], goal)

            cmd, q = policy.get_actions(obs, g + [goal[3]], noise_eps=params['noise_eps'] if not exploit else 0.,
                                        random_eps=params['random_eps'] if not exploit else 0.,
                                        use_target_net=use_target_net)

            # if len(episode) % 20 == 0:
                # print('Step', len(episode), 'Cmd:', cmd, 'Q:', q)

            new_obs, done, info = env.step(cmd)

            if params['view']:
                env.view()

            g_after = relative_pose(new_obs['pose'], goal)
            rw = ((g[0] ** 2 + g[1] ** 2) - (g_after[0] ** 2 + g_after[1] ** 2)) / 100.0

            episode.append([obs, cmd, rw, goal])
            obs = new_obs

        env.finalize()
        if len(episode) == 0:
            continue

        if params['use_her']:
            update_rewards(params, episode, info)

        episodes.append(episode)

        if info['success']: n_successes += 1
        elif info['hit_obstacle']: n_collisions += 1

    return episodes, n_successes, n_collisions, q0


def launch(params, n_epochs, seed, policy_save_interval, checkpoint):
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

    print('Connecting to carmen')
    if params['env'] == 'simple': env = SimpleEnv(params)
    elif params['env'] == 'carmen': env = CarmenEnv(params)
    else: raise Exception("Env '{}' not implemented.".format(params['env']))

    state, goal = env.reset()
    n_laser_readings = len(state['laser'])

    policy = DDPG(params, n_laser_readings=n_laser_readings, use_her=params['use_her'])
    if len(checkpoint) > 0:
        policy.load(checkpoint)

    # Path to some log files
    experiment_dir = os.path.join(results_dir, str(ex.current_run._id))
    latest_policy_path = os.path.join(experiment_dir, 'policy_latest.ckpt')
    best_policy_path = os.path.join(experiment_dir, 'policy_best.ckpt')
    periodic_policy_path = os.path.join(experiment_dir, 'policy_{}.ckpt')

    best_success_rate = -1

    last_success_flags = []
    last_collision_flags = []

    # Training loop
    for epoch in range(n_epochs):
        # rollouts
        episodes, n_successes, n_collisions, q0 = generate_rollouts(policy, env, params['n_train_rollouts'],
                                                                params, exploit=False,
                                                                use_her=params['use_her'])

        # report
        last_success_flags.append(n_successes / float(len(episodes)))
        last_collision_flags.append(n_collisions / float(len(episodes)))

        if len(last_success_flags) > 30: last_success_flags.pop(0)
        if len(last_collision_flags) > 30: last_collision_flags.pop(0)

        last_episode = episodes[-1]

        print('** Episode', epoch * params['n_train_rollouts'],
              'Return:', np.sum([t[2] for t in last_episode]),
              'n_successes:', n_successes,
              'n_collisions:', n_collisions,
              'DIST:', dist(last_episode[-1][0]['pose'], last_episode[-1][-1]),
              'SampleQ:', q0,
              'SuccessAvg30:', np.mean(last_success_flags),
              'CollisionsAvg30:', np.mean(last_collision_flags),
              )

        sys.stdout.flush()

        # Train
        policy.store_episode(episodes)

        if len(policy.buffer.stack) > 1:
            for b in range(params['n_batches']):
                cl, pl = policy.train()
                # if b % 50 == 0:
                    # print('Training batch', b, 'Critic loss:', cl, 'Policy loss:', pl)

            # Save
            if epoch % 20 == 0:
                policy_path = periodic_policy_path.format(epoch)
                policy.save(policy_path)

            policy.update_target_net()

        # test
        if params['n_test_rollouts'] > 0:
            print('Testing...')
            episodes, n_successes, n_collisions, _ = generate_rollouts(policy, env, params['n_test_rollouts'],
                                                                       params, exploit=True,
                                                                       use_her=params['use_her'])
            print_evaluation_report(episodes, n_successes, n_collisions)
            success_rate = float(n_successes) / float(len(episodes))
            collision_rate = float(n_collisions) / float(len(episodes))
            print('Evaluation success rate:', success_rate, 'collision_rate:', collision_rate)

            # save the policy if it's better than the previous ones
            if success_rate >= best_success_rate:
                best_success_rate = success_rate
                print('New best success rate: {}. Saving policy to {} ...'.format(best_success_rate, best_policy_path))
                policy.save(best_policy_path)
                policy.save(latest_policy_path)

            if policy_save_interval > 0 and epoch % policy_save_interval == 0 and save_policies:
                policy_path = periodic_policy_path.format(epoch)
                print('Saving periodic policy to {} ...'.format(policy_path))
                policy.save(policy_path)


@ex.config
def config():
    seed = 1
    n_epochs = 100000

    # the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.
    policy_save_interval = 0

    params = {
        'rddf': 'rddf-voltadaufes-20170809.txt',
        'n_hidden_neurons': 64,
        'n_hidden_layers': 1,
        'use_conv_layer': True,
        'activation_fn': 'elu',
        'allow_negative_commands': False,
        'use_her': True,
        # env
        'env': 'simple',
        'model': 'ackerman',
        'n_steps_episode': 200,
        'goal_achievement_dist': 1.0,
        'vel_achievement_dist': 0.5,
        'view': False,
        # training
        'n_train_rollouts': 1,  # per epoch
        'n_batches': 50,  # training batches per cycle
        'batch_size': 256,  # per mpi thread, measured in transitions and reduced to even multiple of chunk_length.
        'n_test_rollouts': 0,  # number of test rollouts per epoch, each consists of rollout_batch_size rollouts
        # exploration
        'random_eps': 0.0,  # percentage of time a random action is taken
        'noise_eps': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
    }

    params['gamma'] = 1. - 1. / params['n_steps_episode']

    checkpoint = ''


@ex.automain
def main(seed, n_epochs, policy_save_interval, params, checkpoint):
    launch(params, n_epochs, seed, policy_save_interval, checkpoint)
