
import matplotlib.pyplot as plt
import numpy as np

data = [l.rstrip().rsplit() for l in open("data_log_volta_da_ufes-20180112.txt", "r").readlines()]
data = np.array([l[1:3] + l[4:6] + [l[8]] for l in data]).astype(np.double)
diff_vs = []
gps_vs = []
odom_vs = []

for i in range(1, data.shape[0]):
    gps_d = ((data[i, 0] - data[i-1, 0]) ** 2 + (data[i, 1] - data[i-1, 1]) ** 2) ** 0.5
    gps_dt = data[i, 4] - data[i-1, 4]

    if (gps_d < 0.5) and (gps_dt < 1.0) and (gps_dt > 0):
        gps_v = gps_d / gps_dt
        gps_vs.append(gps_v)
        diff_vs.append(gps_v - data[i, 2])
        odom_vs.append(data[i, 2])


plt.plot(gps_vs)
plt.plot(odom_vs)
plt.plot(diff_vs)
plt.legend(('gps_vs', 'odom_v', 'diff_vs'))
plt.figure()
plt.hist(diff_vs, bins=1000, range=(-10,10))
plt.show()


