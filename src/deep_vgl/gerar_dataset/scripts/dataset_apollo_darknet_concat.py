import os
import numpy as np

columns = [('image', object), ('label', int),
           ('x', float), ('y', float), ('z', float),
           ('rx', float), ('ry', float), ('rz', float),
           ('timestamp', object)]

def base_datasetname(dir, year_base, year_curr, offset_base, offset_curr):
    return dir + "basepos-{0}-{1}-{2}m-{3}m.txt".format(year_base, year_curr, offset_base, offset_curr)


def curr_datasetname(dir, year_base, year_curr, offset_base, offset_curr):
    return dir + "livepos-{0}-{1}-{2}m-{3}m.txt".format(year_base, year_curr, offset_base, offset_curr)


def save_dataset(data, dataset):
    sample_file = open(dataset, "w")
    sample_file.write("image label x y z rx ry rz timestamp\n")
    for i in range(len(data)):
        sample_file.write("{0} {1} {2} {3} {4} {5} {6} {7} {8}\n".format(
            data['image'][i], data['label'][i],
            data['x'][i], data['y'][i], data['z'][i],
            data['rx'][i], data['ry'][i], data['rz'][i],
            data['timestamp'][i])
        )
    sample_file.close()


def concat_dataset(output_dir, trainfile, basefile, datasets, offset_base, offset_curr):
    datasetname_out = curr_datasetname(output_dir, trainfile, basefile, offset_base, offset_curr)
    for i in range(0, len(datasets)):
        datasetname_in = curr_datasetname(output_dir,basefile, datasets[i], offset_base, offset_curr)
        data_base = np.genfromtxt(datasetname_in, delimiter=' ', names=True, dtype=np.dtype(columns))
        if i == 0:
            data_all = data_base
        else:
            data_all = np.concatenate((data_all, data_base))
    data_all = np.sort(data_all, axis=0, order=['label', 'timestamp'])
    save_dataset(data_all, datasetname_out)

if __name__ == '__main__':
    input_dir = '/dados/baidu/camerapos/IDA/'
    output_dir = '/dados/baidu/camerapos/IDA/'
    offset_base = 5
    offset_curr = 5

    #os.system('rm -rf ' + output_dir + '*')
    datasets = ['front-20190918143332', 'front-20190924124848','front-20191014142530','front-20191021162130','front-20191025104732','front-20191130112819']
    dataset_valid = 'front-20191216123346'
    dataset_test = 'front-20191225153609'
    dataset_name = 'BAIDU-TRAIN-LAP'
    valid_name = 'BAIDU-VALID-LAP'
    test_name = 'BAIDU-TEST-LAP'

    concat_dataset(output_dir, dataset_name, 'BASE', datasets, offset_base, offset_curr)
    os.system('cp ' + curr_datasetname(output_dir, 'BASE', dataset_test, offset_base, offset_curr) + ' ' + curr_datasetname(output_dir, test_name, dataset_test, offset_base, offset_curr))
    os.system('cp ' + curr_datasetname(output_dir, 'BASE', dataset_valid, offset_base, offset_curr) + ' ' + curr_datasetname(output_dir, valid_name, dataset_valid, offset_base, offset_curr))

