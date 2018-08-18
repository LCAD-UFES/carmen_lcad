
import os
import sys

import numpy as np
import json

from sacred import Experiment
from sacred.observers import FileStorageObserver

import rl.config as config
from rl.rollout import RolloutWorker


results_dir = 'results/'
ex = Experiment('rl_motion_planner')
ex.observers.append(FileStorageObserver.create(results_dir))


def print_evaluation_report():
    # TODO.
    pass


def save_policy(policy, path):
    # TODO
    pass


def reset():
    # TODO
    pass


def step(cmd):
    # TODO
    pass


def generate_rollouts(policy, params, exploit, use_target_net=False):
    # generate episodes
    episodes = []
    n_successes = 0

    for t in range(params['n_episodes_per_cycle']):
        obs, goal = reset()
        episode = []
        done = False

        while not done and len(episode) < params['max_steps']:
            policy_output = policy.get_actions(obs, goal,
                                               noise_eps=params['noise_eps'] if not exploit else 0.,
                                               random_eps=params['random_eps'] if not exploit else 0.,
                                               use_target_net=use_target_net)

            new_obs, rw, done, info = step(policy_output)
            episode.append([obs, policy_output, rw, goal])
            obs = new_obs

        n_successes += 1 if success else 0

    return episodes, n_successes


def launch(hparams, n_epochs, seed, policy_save_interval, clip_return, save_policies=True):
    """
    ###############################################################################################
    TODO: Os caras da OpenAI treinaram no final de cada episodio. Tentar treinar a cada passo?
    TODO: Adicionar codigo para gerar graficos.
    TODO: Adicionar verificacao de erros (e.g., NANs no tensorflow).
    TODO: Adicionar goals com diferentes velocidades e diferentes velocidades inciais. Levar em conta o maximo que o
        carro consegue mudar a velocidade dada a distancia do ponto para o goal.
    ###############################################################################################
    """
    # It is not possible to set the seeds used by all carmen modules from here.
    tf.set_random_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    policy = DDPG(params, reuse=False)


    # Path to some log files
    experiment_dir = os.path.join(results_dir, str(ex.current_run._id))
    latest_policy_path = os.path.join(experiment_dir, 'policy_latest.pkl')
    best_policy_path = os.path.join(experiment_dir, 'policy_best.pkl')
    periodic_policy_path = os.path.join(experiment_dir, 'policy_{}.pkl')

    print("Training...")
    best_success_rate = -1

    for epoch in range(n_epochs):

        # train
        rollout_worker.clear_history()
        for _ in range(n_cycles):
            episode = rollout_worker.generate_rollouts()
            policy.store_episode(episode)
            for _ in range(n_batches):
                policy.train()
            policy.update_target_net()

        # test
        evaluator.clear_history()
        for _ in range(n_test_rollouts):
            evaluator.generate_rollouts()
        print_evaluation_report()
        success_rate = 0.0  # TODO.

        # save the policy if it's better than the previous ones
        if success_rate >= best_success_rate:
            best_success_rate = success_rate
            print('New best success rate: {}. Saving policy to {} ...'.format(best_success_rate, best_policy_path))
            save_policy(eval_policy, best_policy_path)
            save_policy(eval_policy, latest_policy_path)

        if policy_save_interval > 0 and epoch % policy_save_interval == 0 and save_policies:
            policy_path = periodic_policy_path.format(epoch)
            print('Saving periodic policy to {} ...'.format(policy_path))
            save_policy(eval_policy, policy_path)


@ex.config
def config():
    seed = 1
    n_epochs = 100000

    # the interval with which policy pickles are saved. If set to 0, only the best and latest policy will be pickled.
    policy_save_interval = 0

    # whether or not returns should be clipped (default = 1)
    clip_return = 1

    rddf = 'rddf-voltadaufes-20170809.txt'

    hparams = {
        # env
        'n_steps_episode': 1000,
        # ddpg
        'layers': 3,  # number of layers in the critic/actor networks
        'hidden': 256,  # number of neurons in each hidden layers
        'network_class': 'baselines.her.actor_critic:ActorCritic',
        'Q_lr': 0.001,  # critic learning rate
        'pi_lr': 0.001,  # actor learning rate
        'buffer_size': int(1E6),  # for experience replay
        'polyak': 0.95,  # polyak averaging coefficient
        'action_l2': 1.0,  # quadratic penalty on actions (before rescaling by max_u)
        'clip_obs': 200.,
        'scope': 'ddpg',  # can be tweaked for testing
        'relative_goals': False,
        # training
        'n_cycles': 50,  # per epoch
        'rollout_batch_size': 2,  # per mpi thread
        'n_batches': 40,  # training batches per cycle
        'batch_size': 256,  # per mpi thread, measured in transitions and reduced to even multiple of chunk_length.
        'n_test_rollouts': 10,  # number of test rollouts per epoch, each consists of rollout_batch_size rollouts
        'test_with_polyak': False,  # run test episodes with the target network
        # exploration
        'random_eps': 0.3,  # percentage of time a random action is taken
        'noise_eps': 0.2,  # std of gaussian noise added to not-completely-random actions as a percentage of max_u
        # HER
        'replay_strategy': 'future',  # supported modes: future, none
        'replay_k': 4,  # number of additional goals used for replay, only used if off_policy_data=future
        # normalization
        'norm_eps': 0.01,  # epsilon used for observation normalization
        'norm_clip': 5,  # normalized observations are cropped to this values
    }

    hparams['gamma'] = 1. - 1. / kwargs['n_steps_episode']
    hparams['clip_return'] = (1. / (1. - gamma)) if clip_return else np.inf


@ex.automain
def main(seed, n_epochs, policy_save_interval, clip_return, hparams):
    launch(hparams, n_epochs, seed, policy_save_interval, clip_return)
