
from collections import OrderedDict

import numpy as np
import tensorflow as tf
from tensorflow.contrib.staging import StagingArea

from baselines import logger
from baselines.her.util import (
    import_function, store_args, flatten_grads, transitions_in_episode_batch)
from baselines.her.replay_buffer import ReplayBuffer
from baselines.common.mpi_adam import MpiAdam


class ActorCritic:
    def __init__(self):
        self.o_tf = inputs_tf['o']
        self.g_tf = inputs_tf['g']
        self.u_tf = inputs_tf['u']

        # Prepare inputs for actor and critic.
        o = self.o_stats.normalize(self.o_tf)
        g = self.g_stats.normalize(self.g_tf)
        input_pi = tf.concat(axis=1, values=[o, g])  # for actor

        # Networks.
        with tf.variable_scope('pi'):
            self.pi_tf = self.max_u * tf.tanh(nn(
                input_pi, [self.hidden] * self.layers + [self.dimu]))
        with tf.variable_scope('Q'):
            # for policy training
            input_Q = tf.concat(axis=1, values=[o, g, self.pi_tf / self.max_u])
            self.Q_pi_tf = nn(input_Q, [self.hidden] * self.layers + [1])
            # for critic training
            input_Q = tf.concat(axis=1, values=[o, g, self.u_tf / self.max_u])
            self._input_Q = input_Q  # exposed for tests
            self.Q_tf = nn(input_Q, [self.hidden] * self.layers + [1], reuse=True)


class DDPG(object):
    def __init__(self, params):
        # Create network. TODO.
        with tf.variable_scope("ddpg"):
            self._create_network()

        # Create replay buffer.
        self.buffer = ReplayBuffer(capacity=100000, n__trad=16, n_her=48)

    def _create_network(self):
        self.sess = tf.get_default_session()
        if self.sess is None:
            self.sess = tf.Session()

        # Networks
        with tf.variable_scope('main') as vs:
            self.main = ActorCritic()

        with tf.variable_scope('target') as vs:
            self.target = ActorCritic()

        assert len(self._vars("main")) == len(self._vars("target"))

        # Critic loss function
        target_Q = batch_tf['r'] + self.gamma * self.target.Q_pi_tf
        clipped_target_Q = tf.clip_by_value(target_Q, clip_value_min=-100, clip_value_max=100)
        self.Q_loss_tf = tf.reduce_mean(tf.square(tf.stop_gradient(clipped_target_Q) - self.main.Q_tf))

        # Policy loss function
        self.pi_loss_tf = -tf.reduce_mean(self.main.Q_pi_tf)
        # TODO: checar se o componente abaixo eh necessario e o que ele significa.
        self.pi_loss_tf += self.action_l2 * tf.reduce_mean(tf.square(self.main.pi_tf))

        # Optimizers
        # TODO: only train self._vars('main/Q')
        self.Q_adam = tf.train.AdamOptimizer(learning_rate=0.001)
        # TODO: only train self._vars('main/pi')
        self.pi_adam = tf.train.AdamOptimizer(learning_rate=0.0001)

        # Gradients
        # TODO: MAKE SURE THESE ARE EQUIVALENT!!!!!
        # TODO: CHECAR FUNCAO DE TREINO MAIS EMBAIXO!!!!!
        self.Q_update = self.Q_adam.minimize(loss=self.Q_loss_tf, var_list=self._vars('main/Q'))
        self.pi_update = self.pi_adam.minimize(loss=self.pi_loss_tf, var_list=self._vars('main/pi'))
        """
        Q_grads_tf = tf.gradients(self.Q_loss_tf, self._vars('main/Q'))
        pi_grads_tf = tf.gradients(self.pi_loss_tf, self._vars('main/pi'))
        assert len(self._vars('main/Q')) == len(Q_grads_tf)
        assert len(self._vars('main/pi')) == len(pi_grads_tf)
        self.Q_grads_vars_tf = zip(Q_grads_tf, self._vars('main/Q'))
        self.pi_grads_vars_tf = zip(pi_grads_tf, self._vars('main/pi'))
        self.Q_grad_tf = flatten_grads(grads=Q_grads_tf, var_list=self._vars('main/Q'))
        self.pi_grad_tf = flatten_grads(grads=pi_grads_tf, var_list=self._vars('main/pi'))
        """

        # Additional operations
        self.main_vars = self._vars('main/Q') + self._vars('main/pi')
        self.target_vars = self._vars('target/Q') + self._vars('target/pi')
        self.copy_main_to_target_op = list(
            map(lambda v: v[0].assign(v[1]), zip(self.target_vars, self.main_vars)))
        self.target_update_rate = 0.95  # rate used for soft update of the target net.
        self.update_target_net_op = list(
            map(lambda v: v[0].assign(self.target_update_rate * v[0] + (1. - self.target_update_rate) * v[1]),
                zip(self.target_vars, self.main_vars)))

        # Initialize all variables
        self.sess.run(tf.global_variables_initializer())

        # Update the target net to be equals to the main net
        self.sess.run(self.copy_main_to_target_op)

    def _vars(self, scope):
        res = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=self.scope + '/' + scope)
        assert len(res) > 0
        return res

    def update_target_net(self):
        self.sess.run(self.update_target_net_op)

    def _global_vars(self, scope):
        res = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=self.scope + '/' + scope)
        return res

    def _random_action(self, n):
        return np.random.uniform(low=-self.max_u, high=self.max_u, size=(n, self.dimu))

    def get_actions(self, o, g, noise_eps=0., random_eps=0., use_target_net=False, compute_Q=False):
        policy = self.target if use_target_net else self.main

        # values to compute
        vals = [policy.pi_tf]
        if compute_Q:
            vals += [policy.Q_pi_tf]

        # forward
        feed = {
            policy.o_tf: o.reshape(-1, self.dimo),
            policy.g_tf: g.reshape(-1, self.dimg),
            policy.u_tf: np.zeros((o.size // self.dimo, self.dimu), dtype=np.float32)
        }
        ret = self.sess.run(vals, feed_dict=feed)

        # action postprocessing
        u = ret[0]
        u += noise_eps * np.random.randn(*u.shape)  # gaussian noise
        u = np.clip(u, -1., 1.)

        # TODO: checar o que isto esta fazendo e se eh necessario.
        u += np.random.binomial(1, random_eps, u.shape[0]).reshape(-1, 1) * (self._random_action(u.shape[0]) - u)  # eps-greedy

        # TODO: wtf????? Estranho esse final...
        if u.shape[0] == 1:
            u = u[0]
        u = u.copy()
        ret[0] = u

        if len(ret) == 1:
            return ret[0]
        else:
            return ret

    def store_episode(self, episode_batch):
        self.buffer.store_episode(episode_batch)

    def get_current_buffer_size(self):
        return self.buffer.get_current_size()

    ####
    # TRAIN
    ####
    def train(self):
        self.batch_size = 64
        batch = self.buffer.sample(self.batch_size)
        # TODO: feed batch
        self.sess.run([self.Q_update, self.pi_update])

    """
    def train(self, stage=True):
        if stage:
            self.stage_batch()
        critic_loss, actor_loss, Q_grad, pi_grad = self._grads()
        self._update(Q_grad, pi_grad)
        return critic_loss, actor_loss

    def sample_batch(self):
        transitions = self.buffer.sample(self.batch_size)
        o, o_2, g = transitions['o'], transitions['o_2'], transitions['g']
        ag, ag_2 = transitions['ag'], transitions['ag_2']
        transitions['o'], transitions['g'] = self._preprocess_og(o, ag, g)
        transitions['o_2'], transitions['g_2'] = self._preprocess_og(o_2, ag_2, g)

        transitions_batch = [transitions[key] for key in self.stage_shapes.keys()]
        return transitions_batch

    def stage_batch(self, batch=None):
        if batch is None:
            batch = self.sample_batch()
        assert len(self.buffer_ph_tf) == len(batch)
        self.sess.run(self.stage_op, feed_dict=dict(zip(self.buffer_ph_tf, batch)))

    def _grads(self):
        # Avoid feed_dict here for performance!
        critic_loss, actor_loss, Q_grad, pi_grad = self.sess.run([
            self.Q_loss_tf,
            self.main.Q_pi_tf,
            self.Q_grad_tf,
            self.pi_grad_tf
        ])
        return critic_loss, actor_loss, Q_grad, pi_grad

    def _update(self, Q_grad, pi_grad):
        self.Q_adam.update(Q_grad, self.Q_lr)
        self.pi_adam.update(pi_grad, self.pi_lr)
    """
    ###
