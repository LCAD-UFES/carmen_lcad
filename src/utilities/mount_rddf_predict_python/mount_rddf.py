import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from scipy import spatial
import os
import math

xyt = "7757803.113726 -363522.333953 -0.111268".split(" ")

def find_position(x, y, data):
    points = np.zeros((len(data), 2))
    for idx, pose in enumerate(data):
        points[idx][0] = pose[0]
        points[idx][1] = pose[1]
    ref = [x, y]
    idx = spatial.KDTree(points).query(ref)[1]
    return idx, data[idx]
    

x_iara = float(xyt[0])
y_iara = float(xyt[1])
t_iara = float(xyt[2])

text_file = open(os.environ["CARMEN_HOME"]+"/data/rndf/rddf-log_volta_da_ufes-20191003.txt", "r")

all_text = text_file.read().split("\n")
data = []

# -1 porque ao realizar o split anterior, ele considera o \n da última linha e cria uma lista vazia

for i in range(len(all_text)-1):
    data.append(all_text[i].split(" "))

positions = find_position(x_iara, y_iara, data)
print("Pontos x y encontrados em: ")
print(positions)


points_x = []
points_y = []
# Obtenção dos pontos relativos ao primeiro ponto do rddf
for i in range(positions[0], positions[0]+150):
    points_x.append(float(data[i][0])-float(positions[1][0]))
    points_y.append(float(data[i][1])-float(positions[1][1]))
#    print(float(data[i][0])-float(positions[0][1]), float(data[i][1])-float(positions[0][2]))

#theta = math.atan2(points_y[1] - points_y[0], points_x[1] - points_x[0])
theta = float(positions[1][2])
dtheta = t_iara - theta

print(theta,dtheta)
#print(theta)
c, s = np.cos(theta), np.sin(theta)
R = np.array(((c,s), (-s, c)))

points_x = np.array(points_x)
points_y = np.array(points_y)

for i in range(len(points_x)):
    res = np.matmul(R,np.array([points_x[i], points_y[i]]))
    points_x[i] = res[0]
    points_y[i] = res[1]
dy = np.matmul(R,np.array([x_iara-float(positions[1][0]), y_iara-float(positions[1][1])]))[1]
print(dy)

arr = np.arange(np.amin(points_x), np.amax(points_x), 0.01)
s = interpolate.CubicSpline(points_x, points_y)
fig, ax = plt.subplots(1, 1)

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
#print(s.c)

ax.plot(points_x, points_y, 'bo', label='Data Point')
ax.plot(arr, s(arr), 'r-', label='Cubic Spline', lw=1)

ax.legend()
plt.axis("square")
plt.show()
