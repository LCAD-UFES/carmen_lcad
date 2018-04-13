
import os
import sys
import numpy as np
import tensorflow as tf
from rl.util import Transform2d


fc = tf.contrib.layers.fully_connected


class Policy:
    def __init__(self, input_shape, n_outputs, hidden_size, learning_rate, nonlin,
                 single_thread, n_hidden_layers=1, continuous_action_max=1.0, continuous_std=0.1,
                 load_from_file=None, use_critic=False):

        if nonlin == 'relu':
            nonlin_f = tf.nn.relu
        elif nonlin == 'elu':
            nonlin_f = tf.nn.elu
        elif nonlin == 'tanh':
            nonlin_f = tf.nn.tanh
        else:
            raise Exception('Invalid nonlin "' + nonlin + '"')

        carmen_path = os.environ['CARMEN_HOME']
        config_path = carmen_path + '/src/rl_motion_planner/data/config.txt'

        self.config = dict([l.rstrip().rsplit(' ') for l in open(config_path, 'r').readlines()])
        for key, value in self.config.items():
            self.config[key] = float(value)

        self.use_critic = use_critic
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

        self.policy_mean = fc(h, n_outputs, activation_fn=None)

        h = fc(self.state, hidden_size, activation_fn=nonlin_f)

        if self.use_critic:
            for _ in range(n_hidden_layers - 1):
                h = fc(h, hidden_size, activation_fn=nonlin_f)
    
            self.critic = fc(h, 1, activation_fn=None)
        
        # noise = tf.random_normal(shape=tf.shape(self.policy_mean))
        # self.base_policy = self.policy_mean + noise * continuous_std
        # self.policy = tf.clip_by_value(self.base_policy, -continuous_action_max, continuous_action_max)

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
        
        self.replay_memory = []

    def subsample_rddf(self, rddfs, px, py, pth):
        n = int(len(rddfs) / 3)
        x, y, th = px, py, pth
        subsampled_rddfs = []
        
        for i in range(n):
            p = 3 * i
            nx, ny, nth = rddfs[p], rddfs[p + 1], rddfs[p + 2]
            d = ((nx - x) ** 2 + (ny - y) ** 2) ** .5
            if d > 1.0:
                x, y, th = nx, ny, nth
                subsampled_rddfs += [x, y, th]
        
        nmax = 3 * int(self.config['n_rddf_poses'])
          
        if len(subsampled_rddfs) > nmax:
            subsampled_rddfs = subsampled_rddfs[:nmax]
        else:
            while len(subsampled_rddfs) < nmax:
                subsampled_rddfs += [px, py, pth]
                
        return subsampled_rddfs

    def subsample_laser(self, laser):
        i = 0
        ls = int(self.config['laser_subsample'])
        subsampled_laser = []
        while i < len(laser):
            subsampled_laser.append(np.min(laser[i:(i + ls)]))    
            i += ls
        return subsampled_laser

    def assemble_state(self, pose, goal, v, phi, goal_v, goal_phi, readings, rddfs):
        pose.x -= self.config['x_offset']
        pose.y -= self.config['y_offset']
        goal.x -= self.config['x_offset']
        goal.y -= self.config['y_offset']
        
        inverted_pose = pose.inverse() 
        goal = inverted_pose.transform(goal)
        
        g = [goal.x / self.config['max_gx'], 
             goal.y / self.config['max_gy'], 
             goal.th]
        
        v = [v / self.config['max_v'], 
             phi / self.config['max_phi'],
             goal_v / self.config['max_v'], 
             goal_phi / self.config['max_phi']]
        
        readings = self.subsample_laser(readings)

        r = [reading / self.config['max_range'] for reading in readings]
        
        rddf_poses = []
        """
        rddfs = self.subsample_rddf(rddfs, pose.x, pose.y, pose.th)
        n = int(len(rddfs) / 3)
        rddf_poses = []
        
        for i in range(n):
            p = i * 3
            rddf_pose = Transform2d(x = rddfs[p] - self.config['x_offset'], 
                                    y = rddfs[p + 1] - self.config['y_offset'], 
                                    th = rddfs[p + 2])
            rddf_pose = inverted_pose.transform(rddf_pose)
            rddf_pose = [rddf_pose.x / self.config['max_gx'], 
                         rddf_pose.y / self.config['max_gy'],
                         rddf_pose.th]
            rddf_poses += rddf_pose 
        """
        
        state = np.array(g + v + r + rddf_poses)
        return state

    def save(self, path):
        save_path = self.saver.save(self.sess, path)
        print('Saved:', save_path)

    def load(self, checkpoint):
        self.saver.restore(self.sess, checkpoint)

    def forward(self, state):
        feed = {self.state: [state]}
        p = self.sess.run([self.policy_mean], feed_dict=feed)
        
        v, phi = p[0][0][0], p[0][0][1]
        v, phi = np.clip(v, 0., 1.), np.clip(phi, -1., 1.)
        
        v *= self.config['max_v']
        phi *= self.config['max_phi']
        
        return v, phi

    def create_train_infrastructure(self):
        self.actions = tf.placeholder(tf.float32, shape=[None, self.n_outputs])

        # loss and optimizer to train with policy gradients
        mult = 1.0 / (self.continuous_std * np.sqrt(2.0 * np.pi))
        diff = self.actions - self.policy_mean
        t = -0.5 * (tf.reduce_sum(tf.square(diff), axis=1) / (self.continuous_std ** 2))
        prob = mult * tf.exp(t)
        log_prob = tf.log(prob + 1e-5)

        self.returns = tf.placeholder(tf.float32, shape=[None])
        advantage = self.returns
        
        if self.use_critic:
            advantage -= self.critic
            self.critic_loss = tf.reduce_mean((self.returns - self.critic) ** 2)
            self.critic_trainer = tf.train.AdamOptimizer(self.learning_rate * 10.).minimize(self.critic_loss)

        self.pg_loss = -tf.reduce_sum(log_prob * advantage)
        self.pg_trainer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.pg_loss)

        # loss and optimizer to train with supervised learning
        self.immitation_loss = tf.reduce_mean(tf.reduce_sum((self.actions - self.policy_mean) ** 2, axis=1))

        vars = tf.trainable_variables() 
        l2loss = tf.add_n([ tf.nn.l2_loss(v) for v in vars if 'bias' not in v.name]) * 0.001
        train_loss = self.immitation_loss + l2loss

        self.immitation_trainer = tf.train.AdamOptimizer(self.learning_rate).minimize(train_loss)

    def train(self, episode):
        returns = [] 
        
        for i in range(len(episode['rewards'])):
            r = 0.
            for j in range(i, len(episode['rewards'])):
                r += 0.9 ** (j - i) * episode['rewards'][j]
                # r += episode['rewards'][j]
            returns.append(r)

        feed = dict()
        feed[self.state] = episode['states']
        feed[self.returns] = returns
        feed[self.actions] = episode['actions']

        loss, _ = self.sess.run([self.pg_loss, self.pg_trainer], feed_dict=feed)
        
        if self.use_critic:
            self.sess.run([self.critic_trainer], feed_dict=feed)
        
        return loss

    
    def train_immitation_learning_episode(self, episode, n_batches, batch_size):
        if len(episode) > 10:
            self.replay_memory.append(episode)
        
        if len(self.replay_memory) <= 0:
            return
        
        if len(self.replay_memory) > 20:
            self.replay_memory.pop(0)

        total_loss = 0.

        for i in range(n_batches):
            batch_xs = []
            batch_ys = []
            
            for j in range(batch_size):
                ep_id = np.random.randint(len(self.replay_memory))
                tr_id = np.random.randint(len(self.replay_memory[ep_id]))
                
                state = self.replay_memory[ep_id][tr_id][0]
                v, phi = self.replay_memory[ep_id][tr_id][1]
                target_command = [v / self.config['max_v'], phi / self.config['max_phi']]
                
                batch_xs.append(state)
                batch_ys.append(target_command)
            
            loss, _ = self.sess.run([self.immitation_loss, self.immitation_trainer], 
                                    feed_dict={self.state: batch_xs, 
                                               self.actions: batch_ys})

            if i % 100 == 0:
                print("Trained batch", i, "of", n_batches, "Percentual: %.2f" % (100 * (i / n_batches)), 'loss:', loss)
            
            total_loss += loss

        return total_loss / n_batches


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
        13: n_laser_readings
        14~: laser readings
        ...
        n: n_rddf poses
        n+1~: rddf poses
        """
        dataset = [[float(f) for f in l.rstrip().rsplit(' ')] for l in open(dataset_path, 'r').readlines()]
        dataset = np.array(dataset)
        
        x = []
        y = []
        
        for i in range(len(dataset)):
            l = dataset[i]

            pose = Transform2d(x=l[0], y=l[1], th=l[2]) 
            goal = Transform2d(x=l[5], y=l[6], th=l[7])
            v, phi = l[3], l[4]
            goal_v, goal_phi = l[8], l[9]
            n_readings = int(l[13])
            readings = l[14:(14 + n_readings)]
            n_rddfs = int(l[(14 + n_readings)])
            rddfs = l[(14 + n_readings + 1):(14 + n_readings + n_rddfs)]
            cmd_v, cmd_phi = l[10], l[11]

            state = self.assemble_state(pose, goal, v, phi, goal_v, goal_phi, readings, rddfs)

            # print(len(state))
            # sys.exit(0)
            
            if cmd_v > 0.01:
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
