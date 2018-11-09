
import numpy as np
import tensorflow as tf
from rl.replay_buffer import ReplayBuffer
from rl.util import numlist2str


def get_activation(name):
    if name == 'leaky_relu': activation_fn = tf.nn.leaky_relu
    elif name == 'elu': activation_fn = tf.nn.elu
    elif name == 'tanh': activation_fn = tf.nn.tanh
    elif name == 'relu': activation_fn = tf.nn.relu
    else: raise Exception("Invalid non-linearity '{}'".format(name))
    return activation_fn


class ActorCritic:
    def __init__(self, n_laser_readings, agent_state_size, goal_size, n_commands,
                 params, allow_negative_commands):

        activation_fn = get_activation(params['activation_fn']) 
        self.v_activation_fn = tf.nn.tanh if allow_negative_commands else tf.nn.sigmoid

        use_conv_layer = params['use_conv_layer']
        if use_conv_layer and n_laser_readings < 100:
            use_conv_layer = False

        self._create_placeholders(n_laser_readings, agent_state_size, goal_size, n_commands)
        laser, goal, state = self._preprocess(params['laser_normalization_weight'], 
                                              params['position_normalization_weight'], 
                                              params['v_normalization_weight'], 
                                              params['angle_normalization_weight'])
        
        self._create_actor(laser, goal, state, n_commands, use_conv_layer, activation_fn, params['n_actor_hidden_layers'],params['n_actor_hidden_neurons'])
        self._create_critic(laser, goal, state, use_conv_layer, activation_fn, params['n_critic_hidden_layers'],params['n_critic_hidden_neurons'])


    def _create_placeholders(self, n_laser_readings, agent_state_size, goal_size, n_commands):
        # goal: (x, y, th, desired_v) - pose in car reference
        self.placeholder_goal = tf.placeholder(dtype=tf.float32, shape=[None, goal_size], name='placeholder_goal')
        # laser: (range0, range1, ...)
        self.placeholder_laser = tf.placeholder(dtype=tf.float32, shape=[None, n_laser_readings, 1], name='placeholder_laser')
        # state: (current_v) - current velocity
        self.placeholder_state = tf.placeholder(dtype=tf.float32, shape=[None, agent_state_size], name='placeholder_state')
        # performed action (for critic): (v, phi) - commands produced by the actor
        self.placeholder_action = tf.placeholder(dtype=tf.float32, shape=[None, n_commands], name='placeholder_action')
        # reward (for training the critic)
        self.placeholder_reward = tf.placeholder(dtype=tf.float32, shape=[None, 1], name='placeholder_reward')
        # flag used when computing the target q: 1 if is final, 0 otherwise.
        self.placeholder_is_final = tf.placeholder(dtype=tf.float32, shape=[None, 1], name='placeholder_is_final')

        
    def _preprocess(self, laser_normalization_weight, 
                    position_normalization_weight, 
                    v_normalization_weight, 
                    angle_normalization_weight):
        laser = self.placeholder_laser * laser_normalization_weight
        goal = self.placeholder_goal * [position_normalization_weight, 
                                        position_normalization_weight, 
                                        angle_normalization_weight, 
                                        v_normalization_weight]
        state = self.placeholder_state * [v_normalization_weight, 
                                          angle_normalization_weight]
        return laser, goal, state
    
    
    def _encoder(self, laser, goal, state, use_conv_layer, activation_fn, n_hidden_neurons):
        # laser pre-processing
        if use_conv_layer:
            laser_c1 = tf.layers.conv1d(laser, filters=32, kernel_size=6, strides=1, padding='same', activation=activation_fn)
            laser_p1 = tf.layers.max_pooling1d(laser_c1, pool_size=4, strides=4)
            laser_c2 = tf.layers.conv1d(laser_p1, filters=32, kernel_size=6, strides=1, padding='same', activation=activation_fn)
            laser_p2 = tf.layers.max_pooling1d(laser_c2, pool_size=4, strides=4)
            laser_c3 = tf.layers.conv1d(laser_p2, filters=32, kernel_size=6, strides=1, padding='same', activation=activation_fn)
            laser_p3 = tf.layers.max_pooling1d(laser_c3, pool_size=4, strides=4)
            laser_fl = tf.layers.flatten(laser_p3)
        else:
            laser_fl = tf.layers.flatten(laser)
            
        laser_fc = tf.layers.dense(laser_fl, units=n_hidden_neurons, activation=activation_fn)

        # goal, state, and action pre-processing
        goal_fc = tf.layers.dense(goal, units=n_hidden_neurons, activation=activation_fn)
        state_fc = tf.layers.dense(state, units=n_hidden_neurons, activation=activation_fn)

        return laser_fc, goal_fc, state_fc


    def _hidden_layers(self, in_tensor, n_layers, n_neurons, activation_fn):
        h = in_tensor
        for _ in range(n_layers):
            h = tf.layers.dense(h, units=n_neurons, activation=activation_fn)
        return h
    
    
    def _create_actor(self, laser, goal, state, n_commands, use_conv_layer, activation_fn, n_layers, n_neurons):
        with tf.variable_scope("actor"):
            l, g, s = self._encoder(laser, goal, state, use_conv_layer, activation_fn, n_neurons)
            i =  tf.concat(axis=-1, values=[l, g, s])
            h = self._hidden_layers(i, n_layers, n_neurons, activation_fn)

            self.command_phi = tf.layers.dense(h, units=int(n_commands/2), activation=tf.nn.tanh, name="command_phi")
            self.command_v = tf.layers.dense(h, units=int(n_commands/2), activation=self.v_activation_fn, name="command_v")
            self.actor_command = tf.concat([self.command_v, self.command_phi], axis=-1)
            #self.actor_command = tf.layers.dense(h, units=n_commands, activation=tf.nn.tanh, name="commands")


    def _critic_head(self, l, g, s, cmd, activation_fn, n_layers, n_neurons, reuse):
        with tf.variable_scope("head", reuse=reuse):
            c = tf.layers.dense(cmd, units=n_neurons, activation=activation_fn)

            critic_input = tf.concat(axis=-1, values=[l, g, s, c])

            h = self._hidden_layers(critic_input, n_layers, n_neurons, activation_fn)
            critic_q = tf.layers.dense(h, units=1, activation=None)
        
        return critic_q


    def _create_critic(self, laser, goal, state, use_conv_layer, activation_fn, n_layers, n_neurons):
        with tf.variable_scope("critic"):
            l, g, s = self._encoder(laser, goal, state, use_conv_layer, activation_fn, n_neurons)

            self.q_from_action_placeholder = self._critic_head(l, g, s, self.placeholder_action, activation_fn, n_layers, n_neurons, reuse=False)
            self.q_from_policy = self._critic_head(l, g, s, self.actor_command, activation_fn, n_layers, n_neurons, reuse=True)

            # Attribute names to the tensor to allow loading later
            self.q_from_policy = tf.identity(self.q_from_policy, name="q_from_policy")
            self.q_from_action_placeholder = tf.identity(self.q_from_action_placeholder, name="q_from_action")


class DDPG(object):
    def __init__(self, params, observation, goal):
        self.gamma = params['gamma']
        self.allow_negative_commands = params['allow_negative_velocity']
        agent_state_size = len(observation['pose'][3:])
        n_laser_readings = len(observation['laser'])
        goal_size = len(goal)
        n_commands = params['n_commands']  # TODO: get this value from the envs.

        self._create_session(params)
        self._create_models(n_laser_readings, agent_state_size, goal_size, n_commands, params)
        self._create_basic_objectives(params)
        
        self.critic_vars = self._vars("main/critic")
        self.policy_vars = self._vars("main/actor")

        self._add_regularizers(params)
        self._create_optimizers(params)
        self._create_target_update_operations(params)
        self._initialize_nets()
        
        self.params = params
        self.saver = tf.train.Saver()
        
        # print all trainable variables for debugging
        for v in self._vars(None):
            print(v)


    def _create_session(self, params):
        self.sess = tf.get_default_session()
        if self.sess is None:
            config = None
            if not params['use_gpu']:
                config = tf.ConfigProto(device_count={'GPU': 0},
                                        inter_op_parallelism_threads=1,
                                        intra_op_parallelism_threads=1)
            print("Session config:", config)
            self.sess = tf.Session(config=config)


    def _create_models(self, n_laser_readings, agent_state_size, goal_size, n_commands, params):
        neg = params['allow_negative_velocity']
        if params['spline_type'] is not None: 
            neg = True

        if params['spline_type'] == 'cubic':
            n_commands *= 3

        # Networks
        with tf.variable_scope('main'):
            self.main = ActorCritic(n_laser_readings,
                                    agent_state_size, goal_size,
                                    n_commands,
                                    params,
                                    neg)

        with tf.variable_scope('target'):
            self.target = ActorCritic(n_laser_readings,
                                      agent_state_size, goal_size,
                                      n_commands,
                                      params,
                                      neg)

        assert len(self._vars("main")) == len(self._vars("target"))


    def _create_basic_objectives(self, params):
        discounted_next_q = self.gamma * self.target.q_from_policy * (1. - self.main.placeholder_is_final)
        self.target_Q = self.main.placeholder_reward + discounted_next_q
        clipped_target_Q = tf.clip_by_value(self.target_Q, clip_value_min=-100., clip_value_max=100.)
        # The stop gradient prevents the target net from being trained.
        clipped_target_Q = tf.stop_gradient(clipped_target_Q)
        self.td = tf.square(clipped_target_Q - self.main.q_from_action_placeholder)
        self.critic_loss = tf.reduce_mean(self.td)
        self.policy_loss = -tf.reduce_mean(self.main.q_from_policy)


    def _add_regularizers(self, params):
        self.command_l2 = tf.reduce_mean(tf.square(self.main.actor_command))
        self.command_l2 *= params['l2_cmd']
        self.policy_loss += self.command_l2  

        self.critic_l2 = tf.reduce_mean([tf.nn.l2_loss(v) for v in self.critic_vars])
        self.critic_l2 *= params['l2_regularization_critic']
        self.critic_loss += self.critic_l2 
            
        self.actor_l2 = tf.reduce_mean([tf.nn.l2_loss(v) for v in self.policy_vars])
        self.actor_l2 *= params['l2_regularization_actor']
        self.policy_loss += self.actor_l2


    def _create_optimizers(self, params):
        #self.critic_train = tf.train.AdamOptimizer(learning_rate=params['critic_lr']).minimize(loss=self.critic_loss, var_list=v_critic)
        #self.policy_train = tf.train.AdamOptimizer(learning_rate=params['actor_lr']).minimize(loss=self.policy_loss, var_list=v_policy)
        
        policy_grads = tf.gradients(self.policy_loss, self.policy_vars)
        critic_grads = tf.gradients(self.critic_loss, self.critic_vars)
        
        if params['actor_gradient_clipping'] > 0:
            policy_grads, _ = tf.clip_by_global_norm(policy_grads, params['actor_gradient_clipping'])
            
        if params['critic_gradient_clipping'] > 0:
            critic_grads, _ = tf.clip_by_global_norm(critic_grads, params['critic_gradient_clipping'])
        
        self.policy_train = tf.train.AdamOptimizer(learning_rate=params['actor_lr']).apply_gradients(zip(policy_grads, self.policy_vars))
        self.critic_train = tf.train.AdamOptimizer(learning_rate=params['critic_lr']).apply_gradients(zip(critic_grads, self.critic_vars))
        
        """
        # ###########################
        # Codigo da OpenAI
        # ###########################
        Q_grads_tf = tf.gradients(self.Q_loss_tf, self._vars('main/Q'))
        pi_grads_tf = tf.gradients(self.pi_loss_tf, self._vars('main/pi'))
        assert len(self._vars('main/Q')) == len(Q_grads_tf)
        assert len(self._vars('main/pi')) == len(pi_grads_tf)
        self.Q_grads_vars_tf = zip(Q_grads_tf, self._vars('main/Q'))
        self.pi_grads_vars_tf = zip(pi_grads_tf, self._vars('main/pi'))
        self.Q_grad_tf = flatten_grads(grads=Q_grads_tf, var_list=self._vars('main/Q'))
        self.pi_grad_tf = flatten_grads(grads=pi_grads_tf, var_list=self._vars('main/pi'))
        """


    def _vars(self, scope):
        res = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope)
        assert len(res) > 0
        return res


    def _create_target_update_operations(self, params):
        # Additional operations
        self.main_vars = self._vars('main')
        self.target_vars = self._vars('target')

        self.copy_main_to_target = list(map(lambda v: v[0].assign(v[1]), zip(self.target_vars, self.main_vars)))
        self.target_update_rate = params['soft_update_rate']  # rate used for soft update of the target net.
        
        self.soft_update_target_net = list(
            map(lambda v: v[0].assign(self.target_update_rate * v[0] + (1. - self.target_update_rate) * v[1]),
                zip(self.target_vars, self.main_vars)))


    def _initialize_nets(self):
        self.sess.run(tf.global_variables_initializer())
        self.sess.run(self.copy_main_to_target)


    def update_target_net(self):
        self.sess.run(self.soft_update_target_net)


    def get_actions(self, obs, goal, use_target_net=False):
        policy = self.target if use_target_net else self.main

        # values to compute
        vals = [policy.actor_command, policy.q_from_policy]

        # forward
        feed = {
            self.main.placeholder_laser: [obs['laser']],
            self.main.placeholder_state: [[obs['pose'][3], obs['pose'][4]]],
            self.main.placeholder_goal: [goal],
        }

        ret = self.sess.run(vals, feed_dict=feed)
        
        u = ret[0][0]
        q = ret[1][0][0]
        
        return u, q


    def save(self, path):
        save_path = self.saver.save(self.sess, path)
        print("Model saved in path: %s" % save_path)

    
    def load(self, path):
        self.saver.restore(self.sess, path)


    def _print_batch(self, batch):
        for k, v in batch.items():
            print(k)
            for a in v:
                print('\t', a)
        import sys
        sys.exit(0)
    
    
    def train(self, batch, print_report):
        # debug
        # self._print_batch(batch)
        
        feed = {
            self.main.placeholder_goal: batch['goals'],
            self.main.placeholder_laser: batch['lasers'],
            self.main.placeholder_state: batch['agent_states'],
            self.main.placeholder_action: batch['cmds'],
            self.main.placeholder_reward: batch['rewards'],
            self.main.placeholder_is_final: batch['is_finals'],
            self.target.placeholder_goal: batch['next_goals'],
            self.target.placeholder_laser: batch['next_lasers'],
            self.target.placeholder_state: batch['next_agent_states'],
        }

        out = self.sess.run([self.critic_loss, 
                             self.policy_loss,
                             self.command_l2,
                             self.target_Q, 
                             self.main.q_from_action_placeholder,
                             self.main.q_from_policy,
                             self.target.q_from_policy,
                             self.actor_l2,
                             self.critic_l2,
                             self.critic_train, 
                             self.policy_train],
                            feed_dict=feed)
        
        if print_report:
            print('CriticLoss:', out[0], 'PolicyLoss:', out[1])
            print('Command L2:', out[2], 'Actor L2:', out[7], 'Critic L2:', out[8])
            print('Rw:\t\t', numlist2str(batch['rewards'][:10]))
            print('t.NextQ:\t', numlist2str(out[6][:10]))
            print('IsFinal:\t', numlist2str(batch['is_finals'][:10]))
            print('target_q:\t', numlist2str(out[3][:10]))
            print('m.q_act:\t', numlist2str(out[4][:10]))
            print('m.q_pol:\t', numlist2str(out[5][:10]))
            print()


    """
    ############################
    # Codigo da OpenAI:
    ############################
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


