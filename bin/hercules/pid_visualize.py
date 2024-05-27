import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file_path', help='Path to the log file')
args = parser.parse_args()
file_path = args.file_path

t_odom = []
v_odom = []
t_command = []
v_command = []
theta_command = []
with open(file_path) as file:
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

plt.plot(t_odom, v_odom)
plt.plot(t_command, v_command)

plt.xlabel('Time (s)')
plt.ylabel('Speed')
plt.title('Speed vs Time')
plt.legend(['Odometry', 'Command'])
plt.show()