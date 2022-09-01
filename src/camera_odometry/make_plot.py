from cProfile import label
import os
import numpy as np
import matplotlib.pyplot as plt

list_lines_poses = []

with open("poses_file.txt", "r") as reader:
    list_lines_poses =  reader.readlines()

x_pose = []; y_pose = []

for line in list_lines_poses:
    line_splited = line.replace("\n", "").strip().split(" ")
    line_splited = list(filter(None, line_splited))
    x_pose.append(float(line_splited[0]))
    y_pose.append(float(line_splited[1]))

list_lines_poses.clear()

with open("poses_gps.txt", "r") as reader:
    list_lines_poses =  reader.readlines()

x_gps = []; y_gps = []

for line in list_lines_poses:
    line_splited = line.replace("\n", "").strip().split(" ")
    line_splited = list(filter(None, line_splited))
    x_gps.append(float(line_splited[0]))
    y_gps.append(float(line_splited[1]))

plt.title("Plots")
plt.plot(x_pose, y_pose , label="tartanvo")
plt.plot(x_gps, y_gps, label ="poses_gps")
plt.legend(loc="upper left")
plt.show()