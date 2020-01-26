import math
import numpy as np
from scipy import spatial

local_splines = {}
with open('/dados/rddf_predict/rddf_spline_points_2020-1-26_16:16:36') as f:
    for line in f:
        (data, key) = (line.split()[1:], line.split()[0])
        # print(len(line.split()))
        local_splines[key] = np.array(data, np.float64)

with open('/dados/rddf_predict/rddf_ground_truth_2020-1-26_15:45:35') as f:
    for line in f:
        timestamp = line.split()[0]
        iara_x = float(line.split()[1])
        iara_y = float(line.split()[2])
        iara_theta = float(line.split()[3])

        data = local_splines[timestamp]
        print(data.shape[0])
        local_waypoints = np.zeros((2, data.shape[0]), np.float64)
        local_waypoints[0, :] = np.arange(0, 30.0, 0.01)
        local_waypoints[1, :] = data

        rot_mat = np.array([[math.cos(iara_theta), -math.sin(iara_theta)], [math.sin(iara_theta), math.cos(iara_theta)]])
        global_predicted = np.add(np.matmul(rot_mat, local_waypoints), np.array([iara_x, iara_y]).reshape((2, 1)))

        n = 0
        for i in range(4, len(line.split()), 2):
            n += 1
            x = float(line.split()[i])
            y = float(line.split()[i + 1])
            ref = [x, y]
            # ref = np.array([x, y], np.float64)
            idx = spatial.KDTree(global_predicted.transpose()).query(ref)[1]
            print(x, y)
            print(global_predicted[0][idx], global_predicted[1][idx])
            print(n)

        #print(local_waypoints)
        # print(global_predicted)
        exit()