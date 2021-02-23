import math
import numpy as np
import numpy.linalg as LA
from collections import deque


columns = [('x', float), ('y', float), ('z', float), ('rx', float), ('ry', float), ('rz', float),
           ('timestamp', object), ('left_image', object), ('right_image', object)]


delta_datasetcolumns = [('delta_tx', float), ('delta_ty', float), ('delta_tz', float), ('delta_rx', float), ('delta_ry', float), ('delta_rz', float),
                        ('base_tx', float), ('base_ty', float), ('base_tz', float), ('base_rx', float), ('base_ry', float), ('base_rz', float),
                        ('live_tx', float), ('live_ty', float), ('live_tz', float), ('live_rx', float), ('live_ry', float), ('live_rz', float),
                        ('base_depth', object), ('base_image_left', object), ('base_image_right', object),
                        ('live_depth', object), ('live_image_left', object), ('live_image_right', object),
                        ('base_fx', float), ('base_cx', float), ('base_fy', float), ('base_cy', float), ('base_baseline', float),
                        ('live_fx', float), ('live_cx', float), ('live_fy', float), ('live_cy', float), ('live_baseline', float),
                        ('timestamp', object)]


def get_column_format():
    return columns


def get_column_strings():
    return [''.join(key) for (key,val) in columns]


def logfilename(dir, year):
    return dir + "camerapos-{0}.txt".format(year)


def rotate2d(point2d, radians):
    rotation = np.array([[math.cos(radians), -math.sin(radians)],
                       [math.sin(radians), math.cos(radians)]])
    return np.matmul(rotation, point2d)


def signed_direction(a, b):
    dif_xy = a[[0,1]] - b[[0,1]]
    delta_xy = rotate2d(dif_xy[:,np.newaxis], -b[[2]])
    return 1 if delta_xy[0] >= -0.01 else -1


def get_indices_of_sampled_data(data, min_distance, max_rotation=math.pi):
    last = None
    indices = []
    for i in range(len(data)):
        current_heading = np.array(data['rz'][i])
        current = np.array((data['x'][i], data['y'][i]))
        if last is None:
            distance = min_distance
            rotation = max_rotation
        else:
            distance = LA.norm(last - current)
            rotation = np.abs(last_heading - current_heading)
        if distance >= min_distance or rotation >= max_rotation:
            indices.append(i)
            last = current
            last_heading = current_heading
    return indices


def savedataset(data, dataset):
    sample_file = open(dataset, "w")
    sample_file.write("x y z rx ry rz timestamp left_image right_image\n")
    for i in range(len(data)):
        sample_file.write("{0} {1} {2} {3} {4} {5} {6} {7} {8}\n".format(
            data['x'][i], data['y'][i], data['z'][i],
            data['rx'][i], data['ry'][i], data['rz'][i],
            data['timestamp'][i], data['left_image'][i], data['right_image'][i])
        )
    sample_file.close()


def sample_dataset(datasetname_in, datasetname_out, min_distance, max_rotation):
    data = np.genfromtxt(datasetname_in, delimiter=' ', names=True, dtype=np.dtype(columns))
    indices = get_indices_of_sampled_data(data, min_distance, max_rotation)
    data = data[indices]
    savedataset(data, datasetname_out)


def find_start_point(curr_poses, base_poses):
    shortest_interval = 30.0
    curr_start = -1
    base_start = -1
    last_direction = signed_direction(curr_poses[0], base_poses[0])
    for j in range(min(1000, len(base_poses))):
        for i in range(min(1000, len(curr_poses))):
            # compute distance
            direction = signed_direction(curr_poses[i], base_poses[j])
            delta_t = LA.norm(curr_poses[i][[0,1]] - base_poses[j][[0,1]])
            # remove pontos na contra-mao
            orientation = np.abs(curr_poses[i][2] - base_poses[j][2])
            # encontra o ponto mais proximo usando minimo local
            if (delta_t <= shortest_interval) and (orientation <= math.pi/2):
                shortest_interval = delta_t
                curr_start, base_start = i, j
            if (shortest_interval < 1) and (last_direction != direction):
                return (curr_start, base_start)
            last_direction = direction
    return (curr_start,base_start)


def build_spacial_index(poses, start):
    build_index = range(len(poses))
    build_order = deque(build_index)
    i = 0
    while build_index[i] != start:
        build_order.rotate(-1)
        i = i + 1
    index = [0] * len(poses)
    count = 0
    for c in range(1,len(build_order)):
        i = build_order[c-1]
        j = build_order[c]
        count = count + LA.norm(poses[i][[0, 1]] - poses[j][[0, 1]])
        index[j] = count
    return index

