import numpy as np
import os

preds = {}

with open('/dados/rddf_predict/rddf_spline_points_2020-1-26_16:16:36') as f:
    for line in f:
        (data, key) = (line.split()[:-1], line.split()[-1])
        preds[key] = data

rms = np.array([0.0, 0.0, 0.0, 0.0], np.float64)

with open('/dados/rddf_predict/rddf_ground_truth_2020-1-26_15:45:35') as f:                   
    n = 0
    for line in f:
        n += 1
        dy, k1, k2, k3, timestamp = line.split()
        preds_data = preds[timestamp]
        rms[0] += (float(dy) - float(preds_data[0])) * (float(dy) - float(preds_data[0]))
        rms[1] += (float(k1) - float(preds_data[1])) * (float(k1) - float(preds_data[1]))
        rms[2] += (float(k2) - float(preds_data[2])) * (float(k2) - float(preds_data[2]))
        rms[3] += (float(k3) - float(preds_data[3])) * (float(k3) - float(preds_data[3]))
        # print(float(dy) - float(preds_data[0]), float(k1) - float(preds_data[1]), float(k2) - float(preds_data[2]), float(k3) - float(preds_data[3]))
    rms = rms/n
    rms = np.sqrt(rms)
    print(rms)

#Utilizar root mean squared
