
import numpy as np
import tensorflow as tf

def rad2deg(x):
    return (180. * x) / np.pi


class Poly:
    def __init__(self, order=4):
        self.order = order
        self.coeffs = tf.Variable(np.zeros(self.order), dtype=tf.float32)

    def _poly(self, x, coeffs):
        y = 0.

        for p in range(coeffs.shape[0]):
            y += coeffs[p] * (x ** p)

        return y

    def evaluate(self, t):
        return self._poly(t, self.coeffs)

    def params(self):
        return self.coeffs


class MotionPlanner:
    def __init__(self, l2_weight=0., function_type='poly'):
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

        if function_type == 'poly':
            Function = Poly
        
        self.vfunc = Function()
        self.pfunc = Function()

        for i in range(self.n_steps):
            v = max_v * tf.tanh(self.vfunc.evaluate(self.rddf_cumdt[i]))
            phi = max_phi * tf.tanh(self.pfunc.evaluate(self.rddf_cumdt[i]))

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

        l2 = tf.reduce_sum(self.pfunc.params() ** 2) + tf.reduce_sum(self.vfunc.params() ** 2)
        self.loss += l2 * l2_weight

        self.optimize = tf.train.AdamOptimizer(learning_rate=lr).minimize(self.loss)
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())

    def forward(self, rddf_p, rddf_dt, rddf_cumdt, reinit=True):
        if reinit:
            self.sess.run(tf.global_variables_initializer())
            
        loss = 0.
        poses, cmds, = [], []
        vcoeffs, pcoeffs = [], []
        feed = {self.rddf_p: rddf_p, self.rddf_dt: rddf_dt, self.rddf_cumdt: rddf_cumdt}
        fetches = [self.optimize, self.loss, self.cmds, self.poses, self.vfunc.params(), self.pfunc.params()]

        for i in range(25):
            out = self.sess.run(fetches, feed_dict=feed)
            loss = out[1]
            cmds = out[2]
            poses = out[3]
            vparams = out[4]
            pparams = out[5]
            if i % 50 == 0:
                print('\t', i, 'loss:', np.sqrt(out[1]))

        return cmds, poses, vparams, pparams, loss


