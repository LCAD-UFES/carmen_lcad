
import cv2
import numpy as np
from scipy.optimize import minimize
import tensorflow as tf

data = []


def create_graph(t, data):
    # Camera projection matrix
    Proj = tf.constant([[7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00], 
                     [0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00], 
                     [0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00]])

    # Transformation due to rectification
    Rect = tf.constant([[9.999239e-01, 9.837760e-03, -7.445048e-03, 0.], 
                     [-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.],
                     [7.402527e-03, 4.351614e-03, 9.999631e-01, 0.],
                     [0., 0., 0., 1.]])

    R = tf.constant([[7.533745e-03, -9.999714e-01, -6.166020e-04], 
                  [1.480249e-02, 7.280733e-04, -9.998902e-01], 
                  [9.998621e-01, 7.523790e-03, 1.480755e-02],
                  [0., 0., 0.]])

    t = tf.concat([t, [[1.]]], axis=0)
    V2C = tf.concat([R, t], axis=1)

    cost = 0
    n = 0
    
    for d in data:
        p3d = np.array(list(d[:3]) + [1.]).astype(np.float32).reshape([4, 1])
        p = tf.matmul(V2C, p3d)
        p = tf.matmul(Rect, p)
        pcam = tf.matmul(Proj, p)

        # Convert from homogeneous coordinates to pixel positions.
        px = pcam[0] / pcam[2]
        py = pcam[1] / pcam[2]
    
        cost += ((px - d[-2]) ** 2. + (py - d[-1]) ** 2.) ** 0.5
    
    return cost / len(data)


def objective_function(x):
    global data

    # Camera projection matrix
    Proj = np.array([[7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00], 
                     [0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00], 
                     [0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00]])

    # Transformation due to rectification
    Rect = np.array([[9.999239e-01, 9.837760e-03, -7.445048e-03, 0.], 
                     [-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.],
                     [7.402527e-03, 4.351614e-03, 9.999631e-01, 0.],
                     [0., 0., 0., 1.]])

    R = np.array([[7.533745e-03, -9.999714e-01, -6.166020e-04], 
                  [1.480249e-02, 7.280733e-04, -9.998902e-01], 
                  [9.998621e-01, 7.523790e-03, 1.480755e-02],
                  [0., 0., 0.]])

    t = list(x) + [1.] 
    t = np.array(t).reshape((4, 1))
    V2C = np.hstack((R, t))

    cost = 0
    n = 0
    
    for d in data:
        p3d = np.array(list(d[:3]) + [1.])
        p = np.matmul(V2C, p3d)
        p = np.matmul(Rect, p)
        pcam = np.matmul(Proj, p)

        # Convert from homogeneous coordinates to pixel positions.
        px = int(pcam[0] / pcam[2])
        py = int(pcam[1] / pcam[2])
    
        cost += ((px - d[-2]) ** 2. + (py - d[-1]) ** 2.) ** 0.5
    
    return cost / len(data)
    

def main():
    global data
    
    data = open('points.txt', 'r').readlines()
    for i in range(len(data)):
        data[i] = [float(x) for x in data[i].rstrip().rsplit()]

    t = tf.get_variable("t", [3, 1])
    loss = create_graph(t, data)       
    opt = tf.train.AdamOptimizer(learning_rate=0.01).minimize(loss)
    sess = tf.Session()
    sess.run(tf.global_variables_initializer())
    for i in range(1000):
        out = sess.run([t, loss, opt])
        print(i, out[0][:, 0], out[1][0])
 

def main_powell():
    global data
    
    data = open('points.txt', 'r').readlines()
    for i in range(len(data)):
        data[i] = [float(x) for x in data[i].rstrip().rsplit()]
    
    x0 = np.array([0., 0., 0.])
    res = minimize(objective_function, x0, method='powell', options={'maxiter': 10000, 'xtol': 0})
    print(res)
    
main()




