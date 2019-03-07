
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

offset_x = 7757677.517731
offset_y = -363602.117405
L = 2.625

def generate_graph(data):
    vm = tf.tanh(tf.Variable(1, dtype=tf.float64)) * 2.

    poses = []
    loss = 0.
    n = 0

    for i in range(700, 900): #len(data)):
        if i % 50 == 0:        
            print('step:', i)

        x = tf.Variable(data[i][13], dtype=tf.float64)
        y = tf.Variable(data[i][14], dtype=tf.float64)
        th = tf.Variable(float(data[i][19]), dtype=tf.float64)

        d = data[i]
        gx = d[13]
        gy = d[14]
        v = float(d[23]) * vm

        loss += 2 * (x - gx) ** 2 + 2 * (gy - y) ** 2

        if len(poses) > 0:
            dt = float(d[3]) - float(data[i-1][3])
            ds = dt * v
            dx = ds * tf.cos(poses[-1][2])
            dy = ds * tf.sin(poses[-1][2])
            #print(float(d[23]), dt, dx, dy)

            loss += 1000. * (x - (poses[-1][0] + dx)) ** 2 + 1000. * (y - (poses[-1][1] + dy)) ** 2
            loss += 10000. * (poses[-1][2] - th) ** 2.

        n += 1
        poses.append([x, y, th])

    loss /= n
    biases = [vm]
    return poses, biases, loss


if __name__ == "__main__":
    data = open('/dados/data/data_log_estacionamentos-20181130.txt/sync.txt', 'r').readlines()
    data = [d.rstrip().rsplit() for d in data]
    print('len(data):', len(data))    

    gps = []
    for i in range(700, 900): #len(data)):
        data[i][13] = float(data[i][13]) - offset_x
        data[i][14] = float(data[i][14]) - offset_y
        gps.append([data[i][13], data[i][14]])
    gps = np.asarray(gps)

    poses, biases, loss = generate_graph(data)
    print('graph generated.')
    optimizer = tf.train.AdamOptimizer(0.1).minimize(loss)
    sess = tf.Session()
    sess.run(tf.global_variables_initializer())
    print('variables initialized.')

    for i in range(1000):
        print('Running optimization.')
        _, p, b, l = sess.run([optimizer, poses, biases, loss])
        print('It:', i, 'Loss:', l, 'Biases:', b)
        p = np.asarray(p)

        if i % 50 == 0:
            plt.plot(p[:,0], p[:,1], 'b.', gps[:,0], gps[:,1], 'r.')
            plt.show()

        
    
    
