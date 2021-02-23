import os
import numpy as np
from dataset_util import *
import matplotlib.pyplot as plt


def base_datasetname(dir, year_base, year_curr, offset_base, offset_curr):
    return dir + "basepos-{0}-{1}-{2}m-{3}m.txt".format(year_base, year_curr, offset_base, offset_curr)


def curr_datasetname(dir, year_base, year_curr, offset_base, offset_curr):
    return dir + "livepos-{0}-{1}-{2}m-{3}m.txt".format(year_base, year_curr, offset_base, offset_curr)


def find_closest_in_space(curr_pose, base_poses, curr_time, base_times, base_offset):
    nearest_index = -1
    smallest_distance = base_offset
    shortest_interval = np.float('inf')
    for j in range(len(base_poses)):
        interval = np.abs(curr_time - base_times[j])
        distance = LA.norm(curr_pose[[0,1]]-base_poses[j][[0,1]])
        # remove pontos na contra-mao
        orientation = np.abs(curr_pose[2] - base_poses[j][2])
        if (orientation <= math.pi/2) \
                and (distance >= 0) and (distance <= smallest_distance) \
                and (interval >= 0) and (interval <= shortest_interval):
            smallest_distance = distance
            shortest_interval = interval
            nearest_index = j
    return nearest_index


def detect_closure_loop(data, min_distance):
    index = -1
    first = np.array((data['x'][0], data['y'][0]))
    previous_distance = 0
    for i in range(1, len(data)):
        current = np.array((data['x'][i], data['y'][i]))
        current_distance = LA.norm(first - current)
        if (previous_distance - current_distance) > 0.1 and current_distance <= min_distance:
            index = i
        previous_distance = current_distance
    return index


def plot_dataset(data1, data2, labels):
    plt.figure(figsize=(10, 10), dpi=100)
    data1_scatter = plt.scatter(data1[:,0], data1[:,1], facecolors='g', edgecolors='g', alpha=.5, s=5)
    data2_scatter = plt.scatter(data2[:,0], data2[:,1], facecolors='r', edgecolors='r', alpha=.5, s=5)
    for x2, x1 in enumerate(labels):
        plt.plot([data1[x1,0], data2[x2,0]], [data1[x1,1], data2[x2,1]])
    plt.xlabel('x (m)')
    plt.xlim(-2000,2000)
    plt.ylim(-2000,2000)
    plt.ylabel('y (m)')
    plt.legend((data1_scatter, data2_scatter), ('Base', 'Live'), loc='upper left')
    plt.show()


def save_dataset_base(data, labels, dataset):
    sample_file = open(dataset, "w")
    sample_file.write("image label x y z rx ry rz timestamp\n")
    for i in range(len(data)):
        sample_file.write("{0} {1} {2} {3} {4} {5} {6} {7} {8}\n".format(
            data['left_image'][i], labels[i],
            data['x'][i], data['y'][i], data['z'][i],
            data['rx'][i], data['ry'][i], data['rz'][i],
            data['timestamp'][i])
        )
    sample_file.close()


def create_dataset(datasetname_base, datasetname_curr, datasetname_base_out, datasetname_curr_out, offset_base, offset_curr):
    data_curr_label = []
    data_curr_index = []
    data_curr_label2 = []
    data_curr_index2 = []

    data_base = np.genfromtxt(datasetname_base, delimiter='\t', names=True, dtype=np.dtype(columns))
    data_curr = np.genfromtxt(datasetname_curr, delimiter='\t', names=True, dtype=np.dtype(columns))

    data_base_loop = detect_closure_loop(data_base, offset_base)
    data_curr_loop = detect_closure_loop(data_curr, offset_curr)


    # data_base = data_base[:data_base_loop]
    # data_curr = data_curr[:data_curr_loop]

    data_base_index = get_indices_of_sampled_data(data_base, offset_base)
    data_base = data_base[data_base_index]
    data_base_label = np.arange(data_base.shape[0])
    # data_base_label = np.random.permutation(data_base.shape[0])
    save_dataset_base(data_base, data_base_label, datasetname_base_out)

    data_base_pose_2d = np.dstack([data_base['x'], data_base['y'], data_base['rz']])[0]  # x, y, yaw
    data_curr_pose_2d = np.dstack([data_curr['x'], data_curr['y'], data_curr['rz']])[0]  # x, y, yaw
    curr_start, base_start = find_start_point(data_curr_pose_2d, data_base_pose_2d)
    data_curr_time = build_spacial_index(data_curr_pose_2d, curr_start)
    data_base_time = build_spacial_index(data_base_pose_2d, base_start)
    for index_curr in range(len(data_curr)):
        index_base = find_closest_in_space(data_curr_pose_2d[index_curr], data_base_pose_2d,
                                           data_curr_time[index_curr], data_base_time,
                                           offset_base if index_curr == 0 else 5.0)
        if index_base < 0:  # get only frames ahead in space/time
            print 'live frame with no match: ', index_curr
            continue

        data_curr_label.append(data_base_label[index_base])
        data_curr_index.append(index_curr)

    if offset_base == offset_curr:
        for index_base in range(len(data_base)):
            nearest_index = -1
            smallest_distance = 5 #offset_base/2 # estava travado em 5
            for index_curr in data_curr_index:
                distance = LA.norm(data_curr_pose_2d[index_curr][[0, 1]] - data_base_pose_2d[index_base][[0, 1]])
                if distance < smallest_distance:
                    smallest_distance = distance
                    nearest_index = index_curr
            if nearest_index >= 0:
                data_curr_label2.append(data_base_label[index_base])
                data_curr_index2.append(nearest_index)
        data_curr_index = data_curr_index2
        data_curr_label = np.array(data_curr_label2)
        data_curr = data_curr[data_curr_index]
    else:
        data_curr_label = np.array(data_curr_label)
        data_curr = data_curr[data_curr_index]
        data_curr_index = get_indices_of_sampled_data(data_curr, offset_curr)
        data_curr = data_curr[data_curr_index]
        data_curr_label = data_curr_label[data_curr_index]

    save_dataset_base(data_curr, data_curr_label, datasetname_curr_out)

    data_curr_pose_2d = np.dstack([data_curr['x'], data_curr['y'], data_curr['rz']])[0]
    plot_dataset(data_base_pose_2d, data_curr_pose_2d, data_curr_label)


if __name__ == '__main__':
    input_dir = '/home/tgcavalcante/DRIVE/baidu/camerapos/IDA/'
    output_dir = '/home/tgcavalcante/DRIVE/baidu/camerapos/IDA/'
    offset_base_list = [5]
    offset_curr = 5

    #os.system('rm -rf ' + output_dir + '*')
    datasets = ['front-20190918143332', 'front-20190924124848','front-20191014142530','front-20191021162130','front-20191025104732','front-20191130112819','front-20191216123346','front-20191225153609','BASE']

    for k in range(0, len(offset_base_list)):
        offset_base = offset_base_list[k]
        for i in range(len(datasets)-1,len(datasets)):  # base datasets
            for j in range(0, len(datasets)):  # curr datasets
                # if i != j: continue  # skips building base and curr datasets with different data
                #if i == j: continue  # skips building base and curr datasets with same data
                basefilename_in = logfilename(input_dir, datasets[i])
                currfilename_in = logfilename(input_dir, datasets[j])
                basefilename_out = base_datasetname(output_dir, datasets[i], datasets[j], offset_base, offset_curr)
                currfilename_out = curr_datasetname(output_dir, datasets[i], datasets[j], offset_base, offset_curr)
                if not os.path.isfile(currfilename_out):
                    print 'building ', basefilename_out, currfilename_out
                    create_dataset(basefilename_in, currfilename_in, basefilename_out, currfilename_out, offset_base, offset_curr)
                else:
                    print 'skipping ', basefilename_out, currfilename_out

