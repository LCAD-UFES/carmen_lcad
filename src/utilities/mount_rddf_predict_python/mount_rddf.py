import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
import os

xy = "7757737.982150 -363557.244595".split(" ")

def find_position(x, y, data):
    finded = []
    first = True
    for i in range(len(data)):
        if(float(data[i][0]) >= x and float(data[i][1]) >= y):
            if(first):
                finded.append([i, data[i][0], data[i][1], data[i][2]])
                first = False
        else:
            first = True
    return finded

x = float(xy[0])
y = float(xy[1])

text_file = open(os.environ["CARMEN_HOME"]+"/data/rndf/rddf-log_volta_da_ufes-20191003.txt", "r")

all_text = text_file.read().split("\n")
data = []

# -1 porque ao realizar o split anterior, ele considera o \n da última linha e cria uma lista vazia

for i in range(len(all_text)-1):
    data.append(all_text[i].split(" "))

positions = find_position(x, y, data)
print("Pontos x y encontrados em: ")
print(positions)

if(len(positions)>1):
    print("A função de busca encontrou múltiplos pontos parecidos.")

points_x = []
points_y = []
# Obtenção dos pontos relativos ao primeiro ponto do rddf
for i in range(positions[0][0], positions[0][0]+150):
    points_x.append(float(data[i][0])-float(positions[0][1]))
    points_y.append(float(data[i][1])-float(positions[0][2]))
    print(float(data[i][0])-float(positions[0][1]), float(data[i][1])-float(positions[0][2]))

points_x = np.array(points_x)
points_y = np.array(points_y)

arr = np.arange(np.amin(points_x), np.amax(points_x), 0.01)
s = interpolate.CubicSpline(points_x, points_y)
fig, ax = plt.subplots(1, 1)

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
print(s.c)

ax.plot(points_x, points_y, 'bo', label='Data Point')
ax.plot(arr, s(arr), 'r-', label='Cubic Spline', lw=1)

ax.legend()
plt.show()
