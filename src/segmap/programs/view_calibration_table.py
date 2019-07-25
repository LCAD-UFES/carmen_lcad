
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import cv2


def read_mat(f):
    rows, cols = f.readline().rstrip().rsplit()

    rows = int(rows)
    cols = int(cols)
    mat = np.zeros((rows, cols))

    for i in range(rows):
        line = f.readline().rstrip().rsplit()
        assert len(line) == cols
        for j in range(cols):
            mat[i, j] = float(line[j])

    return mat


def compute_table(sum, count, squared, moc):
    mean = np.zeros_like(sum)
    std = np.zeros_like(sum)

    for i in range(sum.shape[0]):
        for j in range(sum.shape[1]):
            if count[i, j] > moc:
                mean[i, j] = sum[i, j] / count[i, j]

                # rolling variance: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
                s = sum[i, j]
                sq = squared[i, j]
                n = count[i, j]

                var = (sq - (s * s) / n) / (n - 1)

                if np.abs(var) < 1e-5:
                    var = 1e-5

                std[i, j] = np.sqrt(var)

                """
                print("s*s:", s * s)
                print("s*s / n:", (s * s) / n)
                print("sq:", sq)
                print("var:", var)
                print("n:", n)
                print("std:", np.sqrt((sq - (s * s) / n) / (n - 1)))
                print()
                """

    return mean, std


def fill_unobserved_values(mean, std, moc):
    filled_mean = np.copy(mean)
    filled_std = np.copy(std)

    # TODO

    return filled_mean, filled_std


def model(deg, lr):
    x_in = tf.placeholder(tf.float32, shape=(None))
    y_in = tf.placeholder(tf.float32, shape=(None))
    l2_weight = tf.placeholder(tf.float32, shape=[])

    y_pred = tf.zeros_like(x_in)
    coeffs = []

    for i in range(deg):
        coeff_i = tf.get_variable("coeff" + str(i), initializer=tf.random_normal(shape=[], stddev=1e-2))
        # coeff_i = tf.get_variable("coeff" + str(i), initializer=tf.random_normal(shape=[], stddev=1e-2))
        # coeff_i = tf.get_variable("coeff" + str(i), initializer=tf.random_normal(shape=[], stddev=1.0))
        coeffs.append(coeff_i)
        y_pred += coeff_i * tf.pow(x_in, i)

    loss = tf.reduce_sum(tf.square(y_pred - y_in)) + \
           1e2 * tf.square(y_in[0] - y_pred[0]) + \
           l2_weight * tf.reduce_sum(tf.square(coeffs))

    train_op = tf.train.RMSPropOptimizer(lr).minimize(loss)

    poly_model = {
        'x': x_in,
        'y': y_in,
        'pred': y_pred,
        'coeffs': coeffs,
        'loss': loss,
        'train': train_op,
        'l2_weight': l2_weight
    }

    import pprint
    pprint.pprint(poly_model)

    return poly_model


def generate_calib_from_model(poly_model, sess):
    x_complete = np.arange(0, 256, 1) / 255.0
    pred = sess.run(poly_model['pred'], feed_dict={poly_model['x']: x_complete})
    return x_complete, pred


def show_model(x, y, poly_model, sess):
    x_complete, pred = generate_calib_from_model(poly_model, sess)
    plt.clf()
    plt.plot(x, y, '.')
    plt.plot(x_complete, pred)
    plt.waitforbuttonpress()


def train(poly_model, n_iter, x, y, l2, sess):
    sess.run(tf.global_variables_initializer())

    feed = {
        poly_model['x']: x,
        poly_model['y']: y,
        poly_model['l2_weight']: l2,
    }

    fetch = [
        poly_model['train'],
        poly_model['loss'],
    ]

    #print("Init:")
    _, l = sess.run(fetch, feed_dict=feed)
    #show_model(x, y, poly_model, sess)

    for i in range(int(n_iter)):
        _, l = sess.run(fetch, feed_dict=feed)

        # if i % 500 == 0:
            # print("Iter:", i, "loss:", l)
            # show_model(x, y, poly_model, sess)

    #show_model(x, y, poly_model, sess)
    _, calib = generate_calib_from_model(poly_model, sess)

    return calib


def generate_dataset(mean, count, moc):
    x = []
    y = []

    for i in range(len(mean)):
        if count[i] > moc:
            x.append(i / 255.0)
            y.append(mean[i])

    # force that all 255 readings are mapped to 255
    x.append(1.0)
    y.append(1.0)

    return x, y


def write_mat(f, mat):
    f.write("%d %d\n" % (mat.shape[0], mat.shape[1]))
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            f.write("%lf " % mat[i, j])
        f.write("\n")


if __name__ == "__main__":
    LR = 1e-2
    DEGREE = 6
    N_ITER = 2000
    L2_WEIGHT = 0.0
    MIN_OBSERVATIONS_TO_CONSIDER = 20

    with open("calib_table.txt", "r") as f:
        sum = read_mat(f)
        count = read_mat(f)
        sum_squared = read_mat(f)

        mean, std = compute_table(sum, count, sum_squared, MIN_OBSERVATIONS_TO_CONSIDER)
        mean, std = fill_unobserved_values(mean, std, MIN_OBSERVATIONS_TO_CONSIDER)

        poly_model = model(DEGREE, LR)
        poly_calib = np.zeros_like(mean)
        sess = tf.Session()

        for i in range(len(mean)):
            print("Calibrating %d-th ray." % i)

            """
            plt.clf()
            plt.plot(mean[i])
            plt.plot(count[i] / np.sum(count[i]))
            #plt.plot(count[i])
            plt.plot(std[i] / np.sum(std[i]))
            plt.legend(('mean', 'count', 'std'))
            plt.waitforbuttonpress()
            """

            # moc = MIN_OBSERVATIONS_TO_CONSIDER
            moc = np.max(count[i]) / 10.0

            x, y = generate_dataset(mean[i], count[i], moc)
            calibration = train(poly_model, N_ITER, x, y, L2_WEIGHT, sess)

            poly_calib[i, :] = calibration[:]

        """
        plt.plot(mean[0] * 255)
        #plt.plot(mean[4])
        #plt.plot(mean[8])
        plt.plot(mean[16] * 255)
        plt.plot(mean[31] * 255)
        plt.show()
        """

        view_sum = sum / np.max(sum)
        view_count = count / np.max(count)
        view_sum_squared = sum_squared / np.max(sum_squared)

        with open("poly_calib_table.txt", "w") as g:
            write_mat(g, poly_calib)
            print("Saved.")

        cv2.imshow("sum", view_sum)
        cv2.imshow("count", view_count)
        cv2.imshow("sum_squared", view_sum_squared)
        cv2.imshow("mean", mean)
        cv2.imshow("std", std)
        cv2.imshow("poly_calib", poly_calib)
        cv2.waitKey(-1)

