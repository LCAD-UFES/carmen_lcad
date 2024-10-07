import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file_path', help='Path to the log file')
args = parser.parse_args()
file_path = args.file_path

theta_odom = []
theta_command = []
with open(file_path) as file:
    first = True
    for line in file:
        parts = line.split()
        can_id, value = parts[2].split('#')
        if can_id == '080':
            least_significant_theta = value[0:2]
            most_significant_theta = value[2:4]
            current_theta_odom = int(most_significant_theta + least_significant_theta, 16)

        if can_id == '100':
            least_significant_theta = value[4:6]
            most_significant_theta = value[6:8]
            current_theta_command = int(most_significant_theta + least_significant_theta, 16)
            if current_theta_odom > 32767:
                current_theta_odom -= 65536
            if current_theta_command > 32767:
                current_theta_command -= 65536
            theta_odom.append(current_theta_odom)
            theta_command.append(current_theta_command)

# Create scatter plot
plt.scatter(theta_command, theta_odom, c='blue', label='Theta Odom vs. Theta Command')
plt.xlabel('Theta Command')
plt.ylabel('Theta Odom')
plt.title('Scatter Plot of Theta Command vs. Theta Odom')
plt.legend()
plt.grid(True)
plt.show()