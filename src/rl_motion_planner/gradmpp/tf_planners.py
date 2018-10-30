
import tensorflow as tf


class MotionPlanner:
    def inits(self):
        return

    def compute_v(self, i):
        raise NotImplementedError()

    def compute_phi(self, i):
        raise NotImplementedError()

    def additional_loss(self):
        return 0.

    def __init__(self):
        dt = 0.1
        max_v = 10.0
        max_phi = 0.488692  # 28 degrees
        L = 2.625
        lr = 0.1
        self.nsteps = 30

        self.goal = tf.placeholder(tf.float32, shape=[3])
        self.goalv = tf.placeholder(tf.float32, shape=[1])

        self.poses = []
        self.cmds = []

        x = tf.constant(0, dtype=tf.float32)
        y = tf.constant(0, dtype=tf.float32)
        th = tf.constant(0, dtype=tf.float32)
        self.loss = tf.constant(0, dtype=tf.float32)

        self.inits()
        v = phi = 0.

        for i in range(self.nsteps):
            v = max_v * self.compute_v(i)
            phi = max_phi * self.compute_phi(i)

            dx = dt * v * tf.cos(th)
            dy = dt * v * tf.sin(th)

            x = x + dx
            y = y + dy
            th = th + dt * (v / L) * tf.tan(phi)

            pose = tf.convert_to_tensor([x, y, th], dtype=tf.float32)
            cmd = tf.convert_to_tensor([v, phi], dtype=tf.float32)

            self.cmds.append(cmd)
            self.poses.append(pose)

            # rate = float(i) / self.nsteps
            # self.loss += (dx ** 2 + dy ** 2) * 0.1 # + (phi ** 2) * 5. * rate
            self.loss += (self.goal[0] - x) ** 2 + (self.goal[1] - y) ** 2
            self.loss += (self.goalv[0] - v) ** 2 * 0.1
            self.loss += (phi) ** 2 * 10.

            """
            if i > 0:
                phix, phiy = tf.cos(phi), tf.sin(phi)
                lphix, lphiy = tf.cos(self.cmds[-1][1]), tf.sin(self.cmds[-1][1])
                self.loss += ((phix - lphix) ** 2 + (phiy - lphiy) ** 2) * 0.1
            """

        # self.loss += (self.goal[0] - x) ** 2 + (self.goal[1] - y) ** 2
        # self.loss += (self.goalv[0] - v) ** 2 * 0.1
        # self.loss += (phi) ** 2 * 1.
        # self.loss += ((phi * 10.) ** 2) * 0.5
        self.loss += self.additional_loss()

        self.optimize = tf.train.AdamOptimizer(learning_rate=lr).minimize(self.loss)
        # self.optimize = tf.train.GradientDescentOptimizer(learning_rate=lr).minimize(self.loss)
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())

    def forward(self, goal, goalv):
        poses, cmds = [], []
        for i in range(500):
            out = self.sess.run([self.optimize, self.loss, self.cmds, self.poses],
                                feed_dict={self.goal: goal, self.goalv: [goalv]})
            cmds = out[2]
            poses = out[3]

            if i % 20 == 0:
                print(i, 'loss:', out[1])

            """
            if dist(goal, poses[-1]) < 0.5 and np.abs(goalv - cmds[-1][0]) < 0.5:
                print("Solution found after", i, "iterations.")
                break
            """

        return cmds, poses


class MotionPlannerVariables(MotionPlanner):
    def __init__(self):
        super(MotionPlannerVariables, self).__init__()

    def compute_v(self, i):
        return tf.tanh(tf.Variable(np.random.randn(1), dtype=tf.float32)[0])

    def compute_phi(self, i):
        return tf.tanh(tf.Variable(np.random.randn(1), dtype=tf.float32)[0])


class MotionPlannerSinV(MotionPlanner):
    def __init__(self):
        super(MotionPlannerSinV, self).__init__()

    def inits(self):
        self.shift = tf.Variable(np.random.randn(1), dtype=tf.float32)
        self.spread = tf.Variable(np.random.randn(1), dtype=tf.float32)

    def compute_v(self, i):
        return tf.sin(self.shift[0] + self.spread[0] * (float(i) / self.nsteps))

    def compute_phi(self, i):
        return tf.tanh(tf.Variable(np.random.randn(1), dtype=tf.float32)[0])


class MotionPlannerPolyVPolyPhi(MotionPlanner):
    def __init__(self):
        super(MotionPlannerPolyVPolyPhi, self).__init__()

    def inits(self):
        self.vorder = 5
        self.porder = 5
        self.vcoeffs = tf.Variable(np.random.randn(self.vorder) * 1., dtype=tf.float32)
        self.pcoeffs = tf.Variable(np.random.randn(self.porder) * 1., dtype=tf.float32)

    def _poly(self, i, coeffs):
        x = float(i) / self.nsteps
        y = 0.

        for p in range(coeffs.shape[0]):
            y += coeffs[p] * (x ** p)

        return tf.tanh(y)

    def compute_v(self, i):
        return self._poly(i, self.vcoeffs)

    def compute_phi(self, i):
        return self._poly(i, self.pcoeffs)

    def additional_loss(self):
        l2 = tf.reduce_sum(self.pcoeffs ** 2) + tf.reduce_sum(self.vcoeffs ** 2)
        return l2 * 0.1
