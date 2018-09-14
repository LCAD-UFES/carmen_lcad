
import os
import cv2
import sys
import random
import time
import numpy as np

import tensorflow as tf

from sacred import Experiment
from sacred.observers import FileStorageObserver

from rl.ddpg import DDPG
from rl.util import relative_pose, dist, draw_rectangle
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


def view_data(obs, g):
    # TODO!
    return
    view = np.zeros((500, 500, 3)) + 255

    mult = view.shape[0] / 2.0
    mult -= 0.1 * mult

    # read from params file
    init = -np.pi / 2.
    resolution = np.pi / float(len(obs['laser']))

    angle = init

    for range in obs['laser']:
        range = (range[0] + 1.0) / 2.0
        range *= (30. / 50.)

        x = range * mult * np.cos(angle)
        y = range * mult * np.sin(angle)

        px = int(x + view.shape[1] / 2.0)
        py = view.shape[0] - int(y + view.shape[0] / 2.0) - 1

        cv2.circle(view, (px, py), 2, (0, 0, 0), -1)
        angle += resolution

    cv2.circle(view, (view.shape[0] // 2, view.shape[1] // 2), 2, (0, 0, 255), -1)
    # draw_rectangle(img, pose, height, width, zoom)
    draw_rectangle(view, (0., 0., 0.), 2.0, 4.0, 4)

    # PAREI AQUI!!
    g[0] *= mult
    g[1] *= mult
    g[1] = view.shape[0] - g[1] - 1

    px = int(g[0] + view.shape[1] / 2.0)
    py = int(g[1] + view.shape[0] / 2.0)

    cv2.circle(view, (px, py), 2, (0, 255, 0), -1)
    draw_rectangle(view, (g[0] * mult, g[1] * mult, g[2]), 2.0, 4.0, 4, color=(0, 255, 0))

    cv2.imshow("input_data", view)
    cv2.waitKey(1)


def generate_rollouts(policy, env, n_rollouts, params, exploit, use_target_net=False):
    # generate episodes
    episodes = []
    n_successes = 0
    n_collisions = 0
    q0 = 0.

    while len(episodes) < n_rollouts:
        obs, goal = env.reset()

        g = relative_pose(obs['pose'], goal)
        _, q0 = policy.get_actions(obs, g + [goal[3]], noise_eps=0.,
                                   random_eps=0.,
                                   use_target_net=False)

        obs, goal = env.reset()
        episode = []
        done = False
        info = None

        while not done:
            g = relative_pose(obs['pose'], goal)

            cmd, q = policy.get_actions(obs, g + [goal[3]], noise_eps=params['noise_eps'] if not exploit else 0.,
                                        random_eps=params['random_eps'] if not exploit else 0.,
                                        use_target_net=use_target_net)

            new_obs, done, info = env.step(cmd)

            if params['env'] == 'carmen' and params['view']:
                view_data(obs, g)
            elif params['view']:
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

    if params['env'] == 'simple': env = SimpleEnv(params)
    elif params['env'] == 'carmen': env = CarmenEnv(params)
    else: raise Exception("Env '{}' not implemented.".format(params['env']))

    state, goal = env.reset()
    n_laser_readings = len(state['laser'])

    policy = DDPG(params, n_laser_readings=n_laser_readings)
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
        episodes, n_successes, n_collisions, q0 = generate_rollouts(policy, env, params['n_rollouts'],
                                                                    params, exploit=False)

        # report
        last_success_flags.append(n_successes / float(len(episodes)))
        last_collision_flags.append(n_collisions / float(len(episodes)))

        if len(last_success_flags) > 30: last_success_flags.pop(0)
        if len(last_collision_flags) > 30: last_collision_flags.pop(0)

        last_episode = episodes[-1]

        print('** Episode', epoch * params['n_rollouts'],
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
                policy.train()

            # Save
            if epoch % 20 == 0:
                policy_path = periodic_policy_path.format(epoch)
                policy.save(policy_path)

            policy.update_target_net()


@ex.config
def config():
    seed = 1
    n_epochs = 100000

    # the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.
    policy_save_interval = 0

    params = {
        # env
        'env': 'simple',
        'model': 'simple',
        'n_steps_episode': 100,
        'goal_achievement_dist': 0.5,
        'vel_achievement_dist': 0.5,
        'view': True,
        'rddf': 'rddf-voltadaufes-20170809.txt',
        # net
        'n_hidden_neurons': 32,
        'n_hidden_layers': 1,
        'use_conv_layer': False,
        'activation_fn': 'elu',
        'allow_negative_commands': True,
        # training
        'n_rollouts': 1,
        'n_batches': 50,
        'batch_size': 256,
        'use_her': True,
        'her_rate': 1.0,
        'n_test_rollouts': 0,
        'replay_memory_capacity': 500,  # episodes
        # exploration
        'random_eps': 0.05,  # percentage of time a random action is taken
        'noise_eps': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
    }

    params['gamma'] = 1. - 1. / params['n_steps_episode']

    checkpoint = ''


@ex.automain
def main(seed, n_epochs, policy_save_interval, params, checkpoint):
    launch(params, n_epochs, seed, policy_save_interval, checkpoint)
