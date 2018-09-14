
import numpy as np
import tensorflow as tf
from rl.replay_buffer import ReplayBuffer


class ActorCritic:
    def __init__(self, n_laser_readings, n_hidden_neurons, n_hidden_layers, use_conv_layer, activation_fn_name,
                 allow_negative_commands, laser_max_range=30.):
        self.action_size = 2

        # goal: (x, y, th, desired_v) - pose in car reference
        self.placeholder_goal = tf.placeholder(dtype=tf.float32, shape=[None, 4], name='placeholder_goal')
        # laser: (range0, range1, ...)
        self.placeholder_laser = tf.placeholder(dtype=tf.float32, shape=[None, n_laser_readings, 1], name='placeholder_laser')
        # state: (current_v) - current velocity
        self.placeholder_state = tf.placeholder(dtype=tf.float32, shape=[None, 1], name='placeholder_state')
        # performed action (for critic): (v, phi) - commands produced by the actor
        self.placeholder_action = tf.placeholder(dtype=tf.float32, shape=[None, self.action_size], name='placeholder_action')
        # reward (for training the critic)
        self.placeholder_reward = tf.placeholder(dtype=tf.float32, shape=[None, 1], name='placeholder_reward')
        # flag used when computing the target q: 1 if is final, 0 otherwise.
        self.placeholder_is_final = tf.placeholder(dtype=tf.float32, shape=[None, 1], name='placeholder_is_final')

        if activation_fn_name == 'leaky_relu': activation_fn = tf.nn.leaky_relu
        elif activation_fn_name == 'elu': activation_fn = tf.nn.elu
        else: raise Exception("Invalid non-linearity '{}'".format(activation_fn_name))

        if allow_negative_commands: cmd_activation_fn = tf.nn.tanh
        else: cmd_activation_fn = tf.nn.sigmoid

        with tf.variable_scope("preprocessing"):
            # normalize laser to be in [-1., 1.]
            norm_laser = tf.clip_by_value(self.placeholder_laser, 0., laser_max_range)
            norm_laser = (norm_laser / laser_max_range)
            # normalize goal
            norm_goal = self.placeholder_goal # / [10., 10., 1.0, 10.]
            # normalize state
            norm_state = self.placeholder_state # / 10.

            # laser pre-processing
            if use_conv_layer:
                laser_c1 = tf.layers.conv1d(norm_laser, filters=32, kernel_size=6, strides=3, padding='valid', activation=activation_fn)
                laser_c2 = tf.layers.conv1d(laser_c1, filters=32, kernel_size=6, strides=3, padding='valid', activation=activation_fn)
                laser_c3 = tf.layers.conv1d(laser_c2, filters=32, kernel_size=6, strides=3, padding='valid', activation=activation_fn)
                laser_fl = tf.layers.flatten(laser_c3)
                laser_fc = tf.layers.dense(laser_fl, units=n_hidden_neurons, activation=activation_fn)
            else:
                laser_fl = tf.layers.flatten(norm_laser)
                laser_fc = tf.layers.dense(laser_fl, units=n_hidden_neurons, activation=activation_fn)

            # goal, state, and action pre-processing
            goal_fc = tf.layers.dense(norm_goal, units=n_hidden_neurons, activation=activation_fn)
            state_fc = tf.layers.dense(norm_state, units=n_hidden_neurons, activation=activation_fn)

        # actor network
        with tf.variable_scope("policy"):
            actor_input = tf.concat(axis=-1, values=[state_fc, goal_fc, laser_fc])

            in_tensor = actor_input
            for _ in range(n_hidden_layers):
                in_tensor = tf.layers.dense(in_tensor, units=n_hidden_neurons, activation=activation_fn)

            self.actor_command = tf.layers.dense(in_tensor, units=self.action_size,
                                                 activation=cmd_activation_fn, name="actor_command")

        def action_preprocessing(cmd):
            return tf.layers.dense(cmd, units=n_hidden_neurons, activation=activation_fn)

        with tf.variable_scope("action_preprocessing"):
            input_action_fc = action_preprocessing(self.placeholder_action)

        with tf.variable_scope("action_preprocessing", reuse=True):
            computed_action_fc = action_preprocessing(self.actor_command)

        # critic network
        # TODO: esses caras estavam dentro dos variable scopes no codigo da openAI. Precisa? Eles nao tem variaveis...
        critic_input_for_policy_training = tf.concat(axis=-1, values=[actor_input, computed_action_fc])
        critic_input_for_critic_training = tf.concat(axis=-1, values=[actor_input, input_action_fc])

        def critic_net(critic_input):
            in_tensor = critic_input
            for _ in range(n_hidden_layers):
                in_tensor = tf.layers.dense(in_tensor, units=n_hidden_neurons, activation=activation_fn)

            critic_q = tf.layers.dense(in_tensor, units=1, activation=None)
            return critic_q

        with tf.variable_scope("critic"):
            self.q_from_policy = critic_net(critic_input_for_policy_training)

        with tf.variable_scope("critic", reuse=True):
            self.q_from_action_placeholder = critic_net(critic_input_for_critic_training)

        # Attribute names to the tensor to allow loading
        self.q_from_policy = tf.identity(self.q_from_policy, name="q_from_policy")
        self.q_from_action_placeholder = tf.identity(self.q_from_action_placeholder, name="q_from_action")


class DDPG(object):
    def __init__(self, params, n_laser_readings):
        self.gamma = params['gamma']

        if params['use_her']:
            if params['her_rate'] < 0 or params['her_rate'] > 1.0:
                raise Exception('The param "her_rate" should be in the interval [0., 1.]')

            n_trad = int(params['batch_size'] * (1. - params['her_rate']))
            n_her = int(params['batch_size'] * params['her_rate'])
        else:
            n_trad = params['batch_size']
            n_her = 0

        self.allow_negative_commands = params['allow_negative_commands']
        self._create_network(n_laser_readings, params)

        self.buffer = ReplayBuffer(capacity=params['replay_memory_capacity'],
                                   n_trad=n_trad, n_her=n_her,
                                   max_episode_size=params['n_steps_episode'])

        self.saver = tf.train.Saver()

    def _create_network(self, n_laser_readings, params):
        self.sess = tf.get_default_session()

        if self.sess is None:
            self.sess = tf.Session()

        # Networks
        with tf.variable_scope('main'):
            self.main = ActorCritic(n_laser_readings,
                             params['n_hidden_neurons'],
                             params['n_hidden_layers'],
                             params['use_conv_layer'],
                             params['activation_fn'],
                             params['allow_negative_commands'])

        with tf.variable_scope('target'):
            self.target = ActorCritic(n_laser_readings,
                             params['n_hidden_neurons'],
                             params['n_hidden_layers'],
                             params['use_conv_layer'],
                             params['activation_fn'],
                             params['allow_negative_commands'])

        assert len(self._vars("main")) == len(self._vars("target"))

        # Critic loss function (TODO: make sure this is correct!)
        discounted_next_q = self.gamma * self.target.q_from_policy * (1. - self.main.placeholder_is_final)
        target_Q = self.main.placeholder_reward + discounted_next_q
        clipped_target_Q = tf.clip_by_value(target_Q, clip_value_min=-100, clip_value_max=100)
        # The stop gradient prevents the target net from being trained.
        self.critic_loss = tf.reduce_mean(tf.square(tf.stop_gradient(clipped_target_Q) - self.main.q_from_action_placeholder))

        # Policy loss function (TODO: checar se essa loss esta certa. Ela parece diferente do DDPG do paper).
        self.policy_loss = -tf.reduce_mean(self.main.q_from_policy)
        # TODO: checar se o componente abaixo eh necessario e o que ele significa.
        # self.action_l2 = 0.0
        # self.policy_loss += self.action_l2 * tf.reduce_mean(tf.square(self.main.actor_command))

        # Training
        # TODO: MAKE SURE THESE ARE EQUIVALENT!!!!!
        # TODO: CHECAR FUNCAO DE TREINO MAIS EMBAIXO!!!!!
        v_critic = self._vars("main/preprocessing") + self._vars("main/action_preprocessing") + self._vars("main/critic")
        v_policy = self._vars("main/preprocessing") + self._vars("main/action_preprocessing") + self._vars("main/policy")
        self.critic_train = tf.train.AdamOptimizer(learning_rate=0.001).minimize(loss=self.critic_loss, var_list=v_critic)
        self.policy_train = tf.train.AdamOptimizer(learning_rate=0.0001).minimize(loss=self.policy_loss, var_list=v_policy)
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
        self.main_vars = self._vars('main/preprocessing') + self._vars("main/action_preprocessing") + self._vars('main/policy') + self._vars('main/critic')
        self.target_vars = self._vars('target/preprocessing') + self._vars("target/action_preprocessing") + self._vars('target/policy') + self._vars('target/critic')

        self.copy_main_to_target = list(map(lambda v: v[0].assign(v[1]), zip(self.target_vars, self.main_vars)))
        self.target_update_rate = 0.75  # rate used for soft update of the target net.
        self.soft_update_target_net = list(
            map(lambda v: v[0].assign(self.target_update_rate * v[0] + (1. - self.target_update_rate) * v[1]),
                zip(self.target_vars, self.main_vars)))

        # Initialize all variables
        self.sess.run(tf.global_variables_initializer())

        # Update the target net to be equals to the main net
        self.sess.run(self.copy_main_to_target)

    def _vars(self, scope):
        res = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope)
        assert len(res) > 0
        return res

    def update_target_net(self):
        self.sess.run(self.soft_update_target_net)

    def get_actions(self, obs, goal, noise_eps=0., random_eps=0., use_target_net=False):
        policy = self.target if use_target_net else self.main

        # values to compute
        vals = [policy.actor_command, policy.q_from_policy]

        # forward
        feed = {
            self.main.placeholder_laser: [obs['laser']],
            self.main.placeholder_state: [[obs['pose'][3]]],
            self.main.placeholder_goal: [goal],
        }

        ret = self.sess.run(vals, feed_dict=feed)

        # action postprocessing
        u = ret[0][0]
        u += noise_eps * np.random.randn(*u.shape)  # gaussian noise

        # eps greedy
        if np.random.random() < random_eps:
            if self.allow_negative_commands:
                u = np.random.uniform(-1.0, 1.0, len(u))
            else:
                u = np.array([np.random.uniform(0.0, 1.0), np.random.uniform(-1.0, 1.0)])

        if self.allow_negative_commands: u[0] = np.clip(u[0], -1.0, 1.0)
        else: u[0] = np.clip(u[0], 0.0, 1.0)

        return u, ret[1][0][0]

    def store_episode(self, episode_batch):
        self.buffer.add(episode_batch)

    def save(self, path):
        save_path = self.saver.save(self.sess, path)
        print("Model saved in path: %s" % save_path)

    def load(self, path):
        self.saver.restore(self.sess, path)

    ####
    # TRAIN
    ####
    def train(self):
        batch = self.buffer.sample()

        feed = {
            self.main.placeholder_goal: batch['goal'],
            self.main.placeholder_laser: batch['laser'],
            self.main.placeholder_state: batch['state'],
            self.main.placeholder_action: batch['act'],
            self.main.placeholder_reward: batch['rew'],
            self.main.placeholder_is_final: batch['is_final'],
            self.target.placeholder_goal: batch['goal'],
            self.target.placeholder_laser: batch['next_laser'],
            self.target.placeholder_state: batch['next_state'],
        }

        # TODO: Diferente do caso da OpenAI em que os gradients sao computados e aplicados separadamente, aqui os
        # TODO: gradients sao aplicados "ao mesmo tempo" nas variaveis de pre-processamento. Checar em qual ordem isso
        # TODO: eh realizado, e se nao da problema treinar os dois ao msm tempo.
        out = self.sess.run([self.critic_loss, self.policy_loss, self.critic_train, self.policy_train], feed_dict=feed)

        critic_loss = out[0]
        policy_loss = out[1]

        return critic_loss, policy_loss

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
