
import numpy as np
import tensorflow as tf
from rl.util import Transform2d


fc = tf.contrib.layers.fully_connected


class Policy:
    def __init__(self, input_shape, n_outputs, hidden_size, learning_rate, nonlin,
                 single_thread, n_hidden_layers=1, continuous_action_max=1.0, continuous_std=0.1):

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

        # loss and optimizer to train with policy gradients
        mult = 1.0 / (self.continuous_std * np.sqrt(2.0 * np.pi))
        diff = self.actions - self.policy_mean
        t = -0.5 * (tf.reduce_sum(tf.square(diff), axis=1) / (self.continuous_std ** 2))
        prob = mult * tf.exp(t) + 1e-5

        log_prob = tf.log(prob)

        self.returns = tf.placeholder(tf.float32, shape=[None])
        advantage = self.returns

        self.pg_loss = -tf.reduce_sum(log_prob * advantage)
        self.pg_trainer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.pg_loss)

        # loss and optimizer to train with supervised learning
        self.immitation_loss = tf.reduce_mean(tf.reduce_sum((self.actions - self.policy_mean) ** 2, axis=1))
        self.immitation_trainer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.immitation_loss)
        

    def train(self, episode):
        returns = [] 
        
        for i in range(len(episode['rewards'])):
            r = 0.
            for j in range(i, len(episode['rewards'])):
                r += 0.95 ** (j - i) * episode['rewards'][j]
            returns.append(r)

        feed = dict()
        feed[self.state] = episode['states']
        feed[self.returns] = returns
        feed[self.actions] = episode['actions']

        self.sess.run([self.pg_trainer], feed_dict=feed)

    def train_immitation_learning(self, dataset_path, n_epochs, n_batches_by_epoch, batch_size):
        dataset = np.array([[float(f) for f in l.rstrip().rsplit(' ')] for l in open(dataset_path, 'r').readlines()])
        
        y = dataset[:, -3:-1]
        max_cmd = np.max(np.abs(y), axis=0)
        max_v, max_phi = max_cmd
        y /= max_cmd

        print('Max v:', max_v, 'Max phi:', max_phi)
        
        # preprocess dataset
        x = []
        
        for i in range(len(dataset)):
            l = dataset[i]

            v, phi = l[3], l[4]
            pose = Transform2d(x=0., y=0., th=l[2]) 
            g = Transform2d(x=l[5] - l[0], y=l[6] - l[1], th=l[7])
            
            g = pose.inverse().transform(g)
            g = [g.x, g.y, g.th]
            
            p = []
            for j in range(i - 10, i, 1): 
                if j < 0:
                    p.append(0.)
                    p.append(0.)
                else:
                    p.append(dataset[j][-3] / max_v)
                    p.append(dataset[j][-2] / max_phi)

            # state = g + p + [v / max_v, phi / max_phi]
            state = g + [v / max_v, phi / max_phi]
            x.append(state)
        
        # train
        for i in range(n_epochs):
            epoch_loss = 0.
            
            for j in range(n_batches_by_epoch):
                sample_ids = np.random.randint(low=len(x), size=batch_size)
                xs = [x[id] for id in sample_ids]
                ys = [y[id] for id in sample_ids]
                
                loss, _ = self.sess.run([self.immitation_loss, self.immitation_trainer], 
                                        feed_dict={self.state: xs, self.actions: ys})
                
                epoch_loss += loss
                
            print('Epoch:', i, 'Loss:', epoch_loss)
