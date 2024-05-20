import os
import numpy as np
import matplotlib.pyplot as plt
import scipy.io


def read_log_file(file):
    t_odom = []
    v_odom = []
    t_command = []
    v_command = []
    theta_command = []
    first = True
    for line in file:
        if first:
            first = False
            t0 = float(line.split()[0][1:-1])
        parts = line.split()
        can_id, value = parts[2].split('#')
        if can_id == '425':
            t_odom.append(float(parts[0][1:-1]))
            least_significant = value[0:2]
            most_significant = value[2:4]
            v = int(most_significant + least_significant, 16)
            if v > 32767:
                v -= 65536
            v_odom.append(v)
        if can_id == '100':
            t_command.append(float(parts[0][1:-1]))
            least_significant_v = value[0:2]
            most_significant_v = value[2:4]
            least_significant_theta = value[4:6]
            most_significant_theta = value[6:8]
            v = int(most_significant_v + least_significant_v, 16)
            if v > 32767:
                v -= 65536
            v_command.append(v)
            theta_command.append(int(most_significant_theta + least_significant_theta, 16))
    t_odom = np.array(t_odom)
    t_odom = t_odom - t0
    t_command = np.array(t_command)
    t_command = t_command - t0

    v_odom = np.array(v_odom)
    v_command = np.array(v_command)
    theta_command = np.array(theta_command)
    return (t_odom, v_odom, t_command, v_command, theta_command)

log_files = []
filenames = []
for filename in os.listdir('.'):
    if filename.endswith(".log"):  # check for specific file extension if needed
        log_files.append(os.path.join('./', filename))
        filenames.append(str(filename[0:-4]).replace('-', '_'))

results = []
for file_path in log_files:
    with open(file_path) as file:
        results.append(read_log_file(file))

results_to_save = {}
for result, name in zip(results, filenames):
    results_to_save["t_odom_" + name] = result[0]
    results_to_save["v_odom_" + name] = result[1]
    results_to_save["t_command_" + name] = result[2]
    results_to_save["v_command_" + name] = result[3]
    results_to_save["theta_command_" + name] = result[4]

print(results_to_save)

scipy.io.savemat('hercules.mat', results_to_save)