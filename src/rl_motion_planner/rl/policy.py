
import numpy as np
import tensorflow as tf


fc = tf.contrib.layers.fully_connected


class PolicyPG:
    def __init__(self, input_shape, n_outputs, hidden_size, learning_rate, nonlin,
                 single_thread, continuous_std, n_hidden_layers=1, continuous_action_max=1.0):

        if nonlin == 'relu':
            nonlin_f = tf.nn.relu
        elif nonlin == 'elu':
            nonlin_f = tf.nn.elu
        elif nonlin == 'tanh':
            nonlin_f = tf.nn.tanh
        else:
            raise Exception('Invalid nonlin "' + nonlin + '"')

        self.n_outputs = n_outputs
        self.learning_rate = learning_rate
        self.continuous_std = continuous_std
        self.hidden_size = hidden_size
        self.n_hidden_layers = n_hidden_layers
        self.nonlin = nonlin_f

        input_shape = list(input_shape)
        self.state = tf.placeholder(tf.float32, shape=[None] + input_shape)

        h = fc(self.state, hidden_size, activation_fn=nonlin_f)

        for _ in range(n_hidden_layers - 1):
            h = fc(h, hidden_size, activation_fn=nonlin_f)

        self.policy_mean = fc(h, n_outputs, activation_fn=tf.nn.tanh) * continuous_action_max
        
        noise = tf.random_normal(shape=tf.shape(self.policy_mean))
        
        self.base_policy = self.policy_mean + noise * continuous_std
        self.policy = tf.clip_by_value(self.base_policy, -continuous_action_max, continuous_action_max)

        self.create_train_infrastructure()

        if single_thread:
            print('Single thread: True')
            sess_config = tf.ConfigProto(device_count={'GPU': 0},
                                    inter_op_parallelism_threads=1,
                                    intra_op_parallelism_threads=1)
        else:
            print('Single thread: False')
            sess_config = None

        self.saver = tf.train.Saver()
        self.sess = tf.Session(config=sess_config)
        self.sess.run(tf.global_variables_initializer())

    def save(self, path):
        save_path = self.saver.save(self.sess, path)
        print('Saved:', save_path)

    def load(self, checkpoint):
        self.saver.restore(self.sess, checkpoint)

    def forward(self, state):
        feed = {self.state: [state]}
        p = self.sess.run([self.policy], feed_dict=feed)
        return p[0][0]

    def create_train_infrastructure(self):
        self.actions = tf.placeholder(tf.float32, shape=[None, self.n_outputs])

        mult = 1.0 / (self.continuous_std * np.sqrt(2.0 * np.pi))
        diff = self.actions - self.policy_mean
        t = -0.5 * (tf.reduce_sum(tf.square(diff), axis=1) / (self.continuous_std ** 2))
        prob = mult * tf.exp(t) + 1e-5

        log_prob = tf.log(prob)

        self.returns = tf.placeholder(tf.float32, shape=[None])
        advantage = self.returns

        self.loss = -tf.reduce_sum(log_prob * advantage)
        optimizer = tf.train.AdamOptimizer(self.learning_rate)
        self.policy_trainer = optimizer.minimize(self.loss)

    def train(self, episode):
        returns = np.array([np.sum(episode['rewards'][i:]) for i in range(len(episode['rewards']))])

        feed = dict()
        feed[self.state] = episode['states']
        feed[self.returns] = returns
        feed[self.actions] = episode['actions']

        self.sess.run([self.policy_trainer], feed_dict=feed)
