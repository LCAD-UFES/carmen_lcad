
import sys
import numpy as np
import tensorflow as tf
from rl.util import Transform2d


fc = tf.contrib.layers.fully_connected


class Policy:
    def __init__(self, input_shape, n_outputs, hidden_size, learning_rate, nonlin,
                 single_thread, n_hidden_layers=1, continuous_action_max=1.0, continuous_std=0.1,
                 load_from_file=None):

        if nonlin == 'relu':
            nonlin_f = tf.nn.relu
        elif nonlin == 'elu':
            nonlin_f = tf.nn.elu
        elif nonlin == 'tanh':
            nonlin_f = tf.nn.tanh
        else:
            raise Exception('Invalid nonlin "' + nonlin + '"')

        self.config = dict([l.rstrip().rsplit(' ') for l in open('data/config.txt', 'r').readlines()])
        for key, value in self.config.items():
            self.config[key] = float(value)

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

        self.policy_mean = fc(h, n_outputs, activation_fn=None) * continuous_action_max
        
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

    def assemble_state(self, pose, goal, v, phi, goal_v, goal_phi, readings):
        pose.x -= self.config['x_offset']
        pose.y -= self.config['y_offset']
        goal.x -= self.config['x_offset']
        goal.y -= self.config['y_offset']
        
        goal = pose.inverse().transform(goal)
        
        g = [goal.x / self.config['max_gx'], 
             goal.y / self.config['max_gy'], 
             goal.th]
        
        v = [v / self.config['max_v'], 
             phi / self.config['max_phi'],
             goal_v / self.config['max_v'], 
             goal_phi / self.config['max_phi']]
        
        r = [reading / self.config['max_range'] for reading in readings]
        
        state = np.array(g + v + r)
        
        return state

    def save(self, path):
        save_path = self.saver.save(self.sess, path)
        print('Saved:', save_path)

    def load(self, checkpoint):
        self.saver.restore(self.sess, checkpoint)

    def forward(self, state):
        feed = {self.state: [state]}
        p = self.sess.run([self.policy], feed_dict=feed)
        v, phi = p[0][0][0], p[0][0][1]
        v *= self.config['max_v']
        phi *= self.config['max_phi']
        return v, phi

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

    def train_immitation_learning(self, dataset_path, n_epochs, n_batches_by_epoch, batch_size, checkpoint_path=None):
        """
        0: globalpos.x 
        1: globalpos.y 
        2: globalpos.theta 
        3: globalpos->v 
        4: globalpos->phi
        5: goal.x
        6: goal.y 
        7: goal.theta 
        8: goal.v 
        9: goal.phi
        10: command.x 
        11: command.y
        12: globalpos->timestamp
        13~: laser readings
        """
        dataset = np.array([[float(f) for f in l.rstrip().rsplit(' ')] for l in open(dataset_path, 'r').readlines()])
        
        x = []
        y = []
        
        for i in range(len(dataset)):
            l = dataset[i]

            pose = Transform2d(x=l[0], y=l[1], th=l[2]) 
            goal = Transform2d(x=l[5], y=l[6], th=l[7])
            v, phi = l[3], l[4]
            goal_v, goal_phi = l[8], l[9]
            readings = l[13:]
            cmd_v, cmd_phi = l[10], l[11]

            state = self.assemble_state(pose, goal, v, phi, goal_v, goal_phi, readings)
            x.append(state)
            y.append([cmd_v / self.config['max_v'], 
                      cmd_phi / self.config['max_phi']])
        
        # train
        for i in range(n_epochs):
            if i % 10 == 0:
                self.save(checkpoint_path)
                
            epoch_loss = 0.
            
            for j in range(n_batches_by_epoch):
                sample_ids = np.random.randint(low=len(x), size=batch_size)
                
                batch_xs = []
                batch_ys = []
                
                for id in sample_ids:
                    batch_xs.append(x[id])
                    batch_ys.append(y[id])
                
                loss, _ = self.sess.run([self.immitation_loss, self.immitation_trainer], 
                                        feed_dict={self.state: batch_xs, 
                                                   self.actions: batch_ys})
                
                epoch_loss += loss
                
            print('Epoch:', i, 'Loss:', epoch_loss)
            sys.stdout.flush()

        self.save(checkpoint_path)
