import math
import numpy as np
from scipy import spatial

local_splines = {}
with open('/dados/rddf_predict/rddf_spline_points_2020-1-26_16:16:36') as f:
    for line in f:
        (data, key) = (line.split()[1:], line.split()[0])
        # print(len(line.split()))
        local_splines[key] = np.array(data, np.float64)

with open('/dados/rddf_predict/dados para métricas/rddf_ground_truth_test_20191002.txt') as f:
    n_images = 0
    total_rms = 0
    for line in f:
        n_images += 1
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

        n_points = 0
        total_distance = 0
        for i in range(4, len(line.split()), 2):
            n_points += 1
            x = float(line.split()[i])
            y = float(line.split()[i + 1])
            ref = [x, y]
            # ref = np.array([x, y], np.float64)
            idx = spatial.KDTree(global_predicted.transpose()).query(ref)[1]
            distance = math.sqrt(math.pow(global_predicted[0][idx] - x, 2) + math.pow(global_predicted[1][idx] - y, 2))
            total_distance += distance**2
            print(x, y)
            print(global_predicted[0][idx], global_predicted[1][idx])
            print(n_points)
        rms = math.sqrt(total_distance)/n_points
        total_rms += rms
        #print(local_waypoints)
        # print(global_predicted)
    mean_rms = total_rms / n_images