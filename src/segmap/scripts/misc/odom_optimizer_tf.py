
import sys
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt


def correct_phi(corrected_v, phi):
	with tf.variable_scope("phi_correction_function", reuse=tf.AUTO_REUSE):
		# from -2.8 to 2.8 degrees
		phi_bias = tf.tanh(tf.get_variable("phi_bias", shape=(), dtype=tf.float64)) * 0.0488692
		#print(phi)
		#print(phi_bias)
		zero_centred_phi = phi + phi_bias
		#print(corrected_v, zero_centred_phi)
		phi_and_v = tf.expand_dims(tf.stack([corrected_v, zero_centred_phi]), 0)
		#print(phi_and_v)
		h = tf.layers.dense(phi_and_v, 5, activation='elu', name='hidden1')
		#print(h)
		o = tf.layers.dense(h, 1, activation='tanh', name='output') * 0.488692
		#print(o)
		o = o[0][0]
		#print(o)
		return o


def ackerman(x, y, th, v, phi, dt):
	L = 2.625
	ds = dt * v
	x += ds * tf.cos(th)
	y += ds * tf.sin(th)
	th += (ds * tf.tan(phi)) / L
	
	return x, y, th


def generate_graph(data):
	v_multiplier = tf.sigmoid(tf.get_variable("v_multiplier", shape=(), dtype=tf.float64)) * 0.6 + 0.7
	init_angle = tf.get_variable("init_angle", shape=(), dtype=tf.float64)
	
	loss = 0.
	inv_n = 1.0 / (len(data) - 1.0)

	x, y, th = 0.0, 0.0, 0.0
	th += init_angle

	for i in range(1, len(data)):
		if i % 50 == 0:
			print('Processing message', i, 'of', len(data))

		v, phi = data[i][0], data[i][1]
		gps_x, gps_y = data[i][3], data[i][4]
		dt = np.abs(data[i][5] - data[i-1][5])
		
		corrected_v = v * v_multiplier
		corrected_phi = correct_phi(corrected_v, phi)
		
		x, y, th = ackerman(x, y, th, v, phi, dt)
		
		loss += inv_n * ((x - gps_x) ** 2 + (gps_y - y) ** 2)

	variables = tf.trainable_variables()
	return variables, loss


def read_data(path):
	with open(path, 'r') as f:
		data = f.readlines()
		data = [d.rstrip().rsplit() for d in data]
	
	data = np.array(data).astype(np.float64)
	data[:, 3] -= data[0, 3]
	data[:, 4] -= data[0, 4]

	return data
	

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("\nUse python %s <log path>\n" % sys.argv[0])
	else:
		data = read_data(sys.argv[1])
		#data = np.random.random((100, 6))

		variables, loss = generate_graph(data)
		print('graph generated.')
		
		optimizer = tf.train.AdamOptimizer(0.1).minimize(loss)

		sess = tf.Session()
		sess.run(tf.global_variables_initializer())

		print('----------')
		print('\nInitial values of variables:\n')
		for v in variables:
			print(v.name)
			print(sess.run(v))
			print()

		print('----------')

		for i in range(100):
			_, l, v = sess.run([optimizer, loss, variables])

			if i % 10 == 0:
				print('iteration:', i, 'loss:', l, '\n')
				for v in variables:
					print(v.name)
					print(sess.run(v))
					print()
				print('----------')				

			"""			
			p = np.asarray(p)
			if i % 50 == 0:
				plt.plot(p[:,0], p[:,1], 'b.', gps[:,0], gps[:,1], 'r.')
				plt.show()
			"""

