
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
from rl.envs import SimpleEnv, CarmenEnv, CarmenSimEnv

import matplotlib.pyplot as plt


results_dir = 'results/'
ex = Experiment('rl_motion_planner')
ex.observers.append(FileStorageObserver.create(results_dir))

"""
plt.ion()
plt.show()
"""

def update_rewards(params, episode, info):
    rw = -1.

    if info['success']:
        rw = (float(params['n_steps_episode'] + 1. - len(episode))) / float(params['n_steps_episode'])
        print("updated rewards:", rw)
    elif info['hit_obstacle']:
        rw = -1.0

    rw /= len(episode)
    print('rw:', rw)

    for transition in episode:
        transition[2] = rw


def view_data(obs, g, rear_laser_is_active, goal_achievemnt_dist):
    pixels_by_meter = 5.
    viewer_size_in_meters = 150.
    viewer_size_in_pixels = int(viewer_size_in_meters * pixels_by_meter)
    distance_between_front_and_rear_axles = 2.625
    distance_between_rear_wheels = 1.535
    distance_between_rear_car_and_rear_wheels = 0.96
    distance_between_front_car_and_front_wheels = 0.85

    car_angle = obs['pose'][2]
    laser = np.copy(obs['laser'])

    """
    plt.clf()
    plt.plot(range(361), laser[:361], '-b')
    plt.plot(range(361, len(laser)), laser[361:], '-r')
    plt.draw()
    plt.pause(0.001)
    """

    view = np.zeros((viewer_size_in_pixels, viewer_size_in_pixels, 3)) + 255

    # TODO: read from params file
    init = -np.pi / 2.

    if rear_laser_is_active:
        resolution = np.pi / float(len(laser) / 2.)
    else:
        resolution = np.pi / float(len(laser))

    angle = init

    color = (0, 0, 0)
    n = 0

    for distance in laser:
        distance = distance[0]

        x = pixels_by_meter * distance * np.cos(angle + car_angle)
        y = pixels_by_meter * distance * np.sin(angle + car_angle)

        x += view.shape[1] / 2.
        y += view.shape[0] / 2.

        angle += resolution
        n += 1

        # range max
        if abs(distance) >= 30.:
            continue

        if n > len(laser) / 2:
            color = (0, 0, 255)

        cv2.circle(view, (int(x), int(y)), 2, color, -1)

    cv2.circle(view, (view.shape[0] // 2, view.shape[1] // 2), 2, (0, 0, 255), -1)

    # draw_rectangle(img, pose, height, width, zoom)
    car_length = pixels_by_meter * (distance_between_front_and_rear_axles + distance_between_rear_car_and_rear_wheels + distance_between_front_car_and_front_wheels)
    car_width = pixels_by_meter * distance_between_rear_wheels

    # draw car in the origin
    center_to_rear_axis = ((car_length / 2.) - distance_between_rear_car_and_rear_wheels * pixels_by_meter)
    shift_x = center_to_rear_axis * np.cos(obs['pose'][2])
    shift_y = center_to_rear_axis * np.sin(obs['pose'][2])

    rectangle_pose = (shift_x, shift_y, obs['pose'][2])
    draw_rectangle(view, rectangle_pose, height=car_width, width=car_length)

    # draw goal
    from rl.util import Transform2d
    tr_g = Transform2d(g[0], g[1], g[2])
    tr_r = Transform2d(0., 0., obs['pose'][2])
    tr_g = tr_r.transform(tr_g)
    g = np.array([tr_g.x, tr_g.y, tr_g.th])

    # car_length - shft
    x = pixels_by_meter * g[0]
    y = pixels_by_meter * g[1]
    th = g[2]

    radius = int(goal_achievemnt_dist * pixels_by_meter)
    cv2.circle(view, (int(x + view.shape[1] / 2.), int(y + view.shape[0] / 2.)), 2, (0, 128, 0), -1)
    cv2.circle(view, (int(x + view.shape[1] / 2.), int(y + view.shape[0] / 2.)), radius, (0, 128, 0), 1)

    x += center_to_rear_axis * np.cos(th)
    y += center_to_rear_axis * np.sin(th)

    draw_rectangle(view, (x, y, th), height=car_width, width=car_length, color=(0, 128, 0))

    cv2.imshow("input_data", np.flip(view, axis=0))
    cv2.waitKey(1)


def generate_rollouts(policy, env, n_rollouts, params, exploit, use_target_net=False):
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

        i = 0
        while not done:
            g = relative_pose(obs['pose'], goal)

            if params['env'] == 'carmen' and params['view']:
                view_data(obs, g, rear_laser_is_active=env.rear_laser_is_active(),
                          goal_achievemnt_dist=params['goal_achievement_dist'])
                env.view()

            cmd, q = policy.get_actions(obs, g + [goal[3]], noise_eps=params['noise_eps'] if not exploit else 0.,
                                        random_eps=params['random_eps'] if not exploit else 0.,
                                        use_target_net=use_target_net)

            if len(episode) == 0:
                q0 = q

            for _ in range(1):
                new_obs, done, info = env.step(cmd)

            if params['env'] == 'simple' and params['view']:
                env.view()

            # g_after = relative_pose(new_obs['pose'], goal)
            # rw = 0.01 if not info['hit_obstacle'] else -1.0
            rw = (dist(obs['pose'], goal) - dist(goal, new_obs['pose'])) / 10.0
            # rw = -dist(goal, obs['pose']) / 1000.0

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


def print_episode(episode):
    print('-----------------------------------------')
    i = 0
    for transition in episode:
        obs, cmd, rw, goal = transition
        print(i, end=' ')
        for r in obs['laser']:
            print('%.2f' % r[0], end=' ')
        print(obs['pose'], cmd, rw, goal, '\n')
        i += 1
    print('-----------------------------------------')
    input('Pressione enter para continuar:')


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
    elif params['env'] == 'carmen': env = CarmenSimEnv(params)
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

        # print_episode(last_episode)

        print('** Episode', epoch * params['n_rollouts'],
              'Return:', np.sum([t[2] for t in last_episode]),
              'n_successes:', n_successes,
              'n_collisions:', n_collisions,
              'DIST:', dist(last_episode[-1][0]['pose'], last_episode[-1][-1]),
              'SampleQ:', q0,
              'SuccessAvg30:', np.mean(last_success_flags),
              'CollisionsAvg30:', np.mean(last_collision_flags),
              'EpSize:', len(last_episode)
              )

        sys.stdout.flush()

        # Train
        policy.store_episode(episodes)

        if len(policy.buffer.stack) > 0:
            for b in range(params['n_batches']):
                c_loss, p_loss, target_next_q, predicted_q, rew, main_q_policy = policy.train()
                #"""
                if b % 10 == 0:
                    print('Batch', b, 'CriticLoss:', c_loss, 'PolicyLoss:', p_loss,
                          'target_next_q predicted_q:\n', np.concatenate([rew[:5],
                                                                          target_next_q[:5],
                                                                          rew[:5] + target_next_q[:5],
                                                                          predicted_q[:5],
                                                                          main_q_policy[:5]], axis=1))
                #"""

            policy.update_target_net()

            # Save
            if epoch % 20 == 0:
                policy_path = periodic_policy_path.format(epoch)
                policy.save(policy_path)


@ex.config
def config():
    seed = 1
    n_epochs = 100000

    # the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.
    policy_save_interval = 0

    params = {
        # env
        'env': 'carmen',
        'model': 'simple',
        'n_steps_episode': 100,
        'goal_achievement_dist': 1.0,
        'vel_achievement_dist': 0.5,
        'view': True,
        'rddf': 'rddf-voltadaufes-20170809.txt',
        # net
        'n_hidden_neurons': 64,
        'n_hidden_layers': 1,
        'soft_update_rate': 0.75,
        'use_conv_layer': False,
        'activation_fn': 'leaky_relu',
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
        'random_eps': 0.1,  # percentage of time a random action is taken
        'noise_eps': 0.1,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
    }

    params['gamma'] = 1. - 1. / params['n_steps_episode']

    checkpoint = ''


@ex.automain
def main(seed, n_epochs, policy_save_interval, params, checkpoint):
    launch(params, n_epochs, seed, policy_save_interval, checkpoint)
