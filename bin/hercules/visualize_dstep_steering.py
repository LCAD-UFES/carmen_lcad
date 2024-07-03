import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file_path', help='Path to the log file')
parser.add_argument('dt', help='Time between commands')
args = parser.parse_args()
file_path = args.file_path

dt = float(args.dt)
t_odom = []
theta_odom = []
t_command = []
theta_command = []
current_theta = 0
with open(file_path) as file:
    first = True
    for line in file:
        if first:
            first = False
            t0 = float(line.split()[0][1:-1])
        parts = line.split()
        can_id, value = parts[2].split('#')
        if can_id == '080':
            t_odom.append(float(parts[0][1:-1]))
            least_significant = value[0:2]
            most_significant = value[2:4]
            theta = int(most_significant + least_significant, 16)
            theta_odom.append(theta)
        if can_id == '100':
            t_command.append(float(parts[0][1:-1]))
            least_significant_theta = value[4:6]
            most_significant_theta = value[6:8]
            theta = int(most_significant_theta + least_significant_theta, 16)
            current_theta += dt * theta
            theta_command.append(current_theta)

# Create scatter plot
plt.scatter(theta_command, theta_odom, c='blue', label='Theta Odom vs. Theta Command')
plt.xlabel('Theta Command')
plt.ylabel('Theta Odom')
plt.title('Scatter Plot of Theta Command vs. Theta Odom')
plt.legend()
plt.grid(True)
plt.show()