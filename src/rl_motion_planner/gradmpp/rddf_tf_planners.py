
import numpy as np
import tensorflow as tf

def rad2deg(x):
    return (180. * x) / np.pi

class MotionPlanner:
    def __init__(self, l2_weight=0.):
        max_v = 12.0
        max_phi = np.deg2rad(28.)
        L = 2.625
        lr = 0.001
        self.n_steps = 9

        self.rddf_p = tf.placeholder(tf.float32, shape=[self.n_steps, 3])
        self.rddf_dt = tf.placeholder(tf.float32, shape=[self.n_steps])
        self.rddf_cumdt = tf.placeholder(tf.float32, shape=[self.n_steps])

        self.poses = []
        self.cmds = []

        x = tf.constant(0, dtype=tf.float32)
        y = tf.constant(0, dtype=tf.float32)
        th = tf.constant(0, dtype=tf.float32)
        self.loss = tf.constant(0, dtype=tf.float32)

        self.vorder = 4
        self.porder = 4
        self.vcoeffs = tf.Variable(np.zeros(self.vorder), dtype=tf.float32)
        self.pcoeffs = tf.Variable(np.zeros(self.porder), dtype=tf.float32)

        for i in range(self.n_steps):
            v = max_v * self.compute_v(self.rddf_cumdt[i])
            phi = max_phi * self.compute_phi(self.rddf_cumdt[i])

            dt = self.rddf_dt[i]
            dx = dt * v * tf.cos(th)
            dy = dt * v * tf.sin(th)

            x = x + dx
            y = y + dy
            th = th + dt * (v / L) * tf.tan(phi)

            pose = tf.convert_to_tensor([x, y, th], dtype=tf.float32)
            cmd = tf.convert_to_tensor([v, phi], dtype=tf.float32)

            self.cmds.append(cmd)
            self.poses.append(pose)

            self.loss += (self.rddf_p[i][0] - x) ** 2 + (self.rddf_p[i][1] - y) ** 2 #+ (self.rddf_p[i][2] - th) ** 2
            # self.loss += rad2deg(phi) ** 2

        self.loss /= self.n_steps

        l2 = tf.reduce_sum(self.pcoeffs ** 2) + tf.reduce_sum(self.vcoeffs ** 2)
        self.loss += l2 * l2_weight

        self.optimize = tf.train.AdamOptimizer(learning_rate=lr).minimize(self.loss)
        self.sess = tf.Session()

    def forward(self, rddf_p, rddf_dt, rddf_cumdt):
        self.sess.run(tf.global_variables_initializer())
        poses, cmds, = [], []
        vcoeffs, pcoeffs = [], []
        feed = {self.rddf_p: rddf_p, self.rddf_dt: rddf_dt, self.rddf_cumdt: rddf_cumdt}
        fetches = [self.optimize, self.loss, self.cmds, self.poses, self.vcoeffs, self.pcoeffs]

        for i in range(300):
            out = self.sess.run(fetches, feed_dict=feed)
            cmds = out[2]
            poses = out[3]
            vcoeffs = out[4]
            pcoeffs = out[5]

            if i % 50 == 0:
                print('\t', i, 'loss:', np.sqrt(out[1]))

        return cmds, poses, vcoeffs, pcoeffs

    def _poly(self, x, coeffs):
        y = 0.

        for p in range(coeffs.shape[0]):
            y += coeffs[p] * (x ** p)

        return y

    def compute_v(self, t):
        return self._poly(t, self.vcoeffs)

    def compute_phi(self, t):
        return self._poly(t, self.pcoeffs)

