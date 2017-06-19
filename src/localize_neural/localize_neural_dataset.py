import os
import math
import numpy as np
from scipy import spatial

def carmen_radians_to_degrees(theta):
    return theta * 180.0 / math.pi

def carmen_degrees_to_radians(theta):
  return theta * math.pi / 180.0

def carmen_normalize_theta(theta):
    if (theta >= -math.pi and theta < math.pi):
        return theta
    multiplier = math.floor(theta / (2*math.pi))
    theta = theta - multiplier*2*math.pi
    if (theta >= math.pi):
        theta -= 2*math.pi
    if (theta < -math.pi):
        theta += 2*math.pi
    return theta

def concat_files(filename_list, filename_out):

    header = 'image x y z w p q r roll pitch yaw timestamp\n'
    with open(filename_out, 'w') as outfile:
        outfile.write(header)
        for fname in filename_list:
            with open(fname) as infile:
                for line in infile:
                    if header not in line:
                        outfile.write(line)


def concat_datasets(output_dir, days_fortrain, day_forvalid, day_fortest):

    for i in range(0,len(days_fortrain)):
        outfilename_fortrain = output_dir + 'dataset_train' + str(i) + '.txt'
        outfilename_fortest = output_dir + 'dataset_test' + str(i) + '.txt'
        outfilename_forvalid = output_dir + 'dataset_valid' + str(i) + '.txt'

        logfilename_fortrain_list = []
        for j in range(0, i + 1):
            print 'building train dataset :', days_fortrain[j]
            logfilename_fortrain_list.append(input_dir + 'imagepos-' + days_fortrain[j] + '.txt')

        concat_files(logfilename_fortrain_list, outfilename_fortrain)

        if day_fortest is not None:
            print 'building test dataset:', day_fortest
            logfilename_fortest = input_dir + 'imagepos-' + day_fortest + '.txt'
            concat_files([logfilename_fortest], outfilename_fortest)

        if day_forvalid is not None:
            print 'building validation dataset:', day_forvalid
            logfilename_forvalid = input_dir + 'imagepos-' + day_forvalid + '.txt'
            concat_files([logfilename_forvalid], outfilename_forvalid)

def rotate_point2d(point2d, radians):
    rotate = np.array([[np.cos(radians),  -np.sin(radians)],
                       [np.sin(radians), np.cos(radians)]])
    return np.matmul(rotate, point2d)
    

def distances_of_A_to_B(A, B, metric='euclidean'):

    return spatial.distance.cdist(A, B, metric=metric) # returns a matrix with size A rows and size B cols


def create_datasets(dataset_A, dataset_B, dataset_out, max_dist):

    columns = [('image', object),
               ('x', float), ('y', float), ('z', float),
               ('w', float), ('p', float), ('q', float), ('r', float),
               ('roll', float), ('pitch', float), ('yaw', float),
               ('timestamp', object)]

    data_of_A = np.genfromtxt(dataset_A, delimiter=' ', names=True, dtype=np.dtype(columns))
    data_xy_of_A = np.dstack([data_of_A['x'], data_of_A['y']])[0]

    data_of_B = np.genfromtxt(dataset_B, delimiter=' ', names=True, dtype=np.dtype(columns))
    data_xy_of_B = np.dstack([data_of_B['x'], data_of_B['y']])[0]

    distances = distances_of_A_to_B(data_xy_of_A, data_xy_of_B)

    outfile = open(dataset_out, 'w')
    outfile.write('base_image base_x base_y base_z base_yaw delta_trans delta_rot1 delta_rot2 curr_image curr_timestamp\n')
    #outfile.write('base_image base_x base_y base_z base_yaw delta_x delta_y delta_yaw curr_image curr_timestamp\n')
    for index_of_A in range(distances.shape[0]):
        indices_of_B_closest_to_A = np.where(distances[index_of_A] < max_dist)

        for index_of_B in indices_of_B_closest_to_A[0]:
            tA = np.array([data_of_A['x'][index_of_A], data_of_A['y'][index_of_A]])
            tB = np.array([data_of_B['x'][index_of_B], data_of_B['y'][index_of_B]])
            oA = data_of_A['yaw'][index_of_A]
            oB = data_of_B['yaw'][index_of_B]
            diff_xy = tB-tA
            diff_yaw = oB-oA
            #Label Option 1: (delta_x, delta_y, d_yaw)
            delta_xy = rotate_point2d(diff_xy, -oA)
            delta_yaw = carmen_normalize_theta(diff_yaw)
            #Label Option 2: (delta_trans, delta_rot1, delta_rot2) i.e. ODOMETRY COMMAND
            delta_trans = np.linalg.norm(diff_xy, ord=2) #euclidean distance
            delta_rot1 = math.atan2(diff_xy[1], diff_xy[0]) - oA
            delta_rot2 = diff_yaw - delta_rot1
            
            outfile.write(str(data_of_A['image'][index_of_A]) + ' ' + 
                          str(data_of_A['x'][index_of_A]) + ' ' + 
                          str(data_of_A['y'][index_of_A]) + ' ' + 
                          str(data_of_A['z'][index_of_A]) + ' ' + 
                          str(data_of_A['yaw'][index_of_A]) + ' ')
            outfile.write(str(delta_trans) + ' ' + str(delta_rot1) + ' ' + str(delta_rot2) + ' ')
            #outfile.write(str(delta_xy[0]) + ' ' + str(delta_xy[1]) + ' ' + str(delta_yaw) + ' ')
            outfile.write(str(data_of_B['image'][index_of_B]) + ' ' + 
                          str(data_of_A['timestamp'][index_of_A]) + '\n')
    outfile.close()

if __name__ == '__main__':
    #VOLTA DA UFES (FRONTAL)
    input_dir = '/dados/ufes/'
    output_dir = '/dados/ufes_delta/'
    days_fortrain = [
                     '20140418', 
                     #'20140912', #anoitecendo
                     #'20160825',
                     #'20160902',
                     #'20161021',
                     #'20160825-01',
                     #'20160906-02', #volta da ufes e reta da penha
                     #'20160830', #volta da ufes e reta da penha (muito blur)
                     #'20140606-05', #ambiental
                     #'20140606-06', #ambiental
                     #'20160721-01', #ambiental
                     #'20161112-00', #ambiental
                     #'20161112-01', #ambiental
                     #'20160721', #ambiental
                     ]
    day_forvalid = '20160825-02'
    day_fortest = '20160906'
    max_dist = 10.0
    img_size = 224

    os.system('rm -rf '+output_dir+'*')

    infilename_fortrain = input_dir + 'imagepos-'+ days_fortrain[0] + '.txt'
    outfilename_fortrain = output_dir + 'deltapos-'+ days_fortrain[0] + '.txt'
    create_datasets(infilename_fortrain, infilename_fortrain, outfilename_fortrain, max_dist)
    