import os
import numpy as np
import sys
import argparse

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


def concat_dataset(output_dir, testfile, trainfile, datasets, offset_base, offset_curr):
    datasetname_out = curr_datasetname(output_dir, testfile,  trainfile, offset_base, offset_curr)
    for i in range(0, len(datasets)):
        datasetname_in = curr_datasetname(output_dir, testfile, datasets[i], offset_base, offset_curr)
        data_base = np.genfromtxt(datasetname_in, delimiter=' ', names=True, dtype=np.dtype(columns))
        if i == 0:
            data_all = data_base
        else:
            data_all = np.concatenate((data_all, data_base))
    data_all = np.sort(data_all, axis=0, order=['label', 'timestamp'])
    save_dataset(data_all, datasetname_out)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='dataset')
    parser.add_argument('-i', '--images_path')
    parser.add_argument('-o', '--output_dir')
    parser.add_argument('-b', '--base_offset')
    parser.add_argument('-l', '--live_offset')

    args = parser.parse_args()
    
    if args.images_path == None or args.output_dir == None or args.base_offset == None or args.live_offset == None :
        parser.print_help()
        exit()
    if not os.path.isdir(args.images_path):
        print 'images path doesnt exists'
        exit()
    if not os.path.isdir(args.output_dir):
        print 'creating output directory ...'
        try:
            os.mkdir(args.output_dir)
        except OSError:
            print ("Creation of the directory %s failed" % args.output_dir)
            exit()
        else:
            print ("Successfully created the directory %s " % args.output_dir)

    input_dir = args.images_path  #'/dados/ufes/'
    output_dir = args.output_dir #'/dados/ufes_wnn/'

    offset_base = int(args.base_offset)
    offset_curr = int(args.live_offset)

    print 'running script with those arguments:\n\timages path:',input_dir,'\n\toutput directory:',output_dir,'\n\toffset base:',offset_base,'\n\toffset live:',offset_curr

    datasets = ['20160825', '20160825-01', '20160825-02', '20171205', '20180112','20180112-02','20161021'] 
    datasets_valid = ['20160830', '20170119']
    datasets_test = ['20191003']
    dataset_train_name = 'UFES-TRAIN-LAPS'
    dataset_valid_name = 'UFES-VALID-LAP'
    dataset_test_name = 'UFES-TEST-LAP'
    concat_dataset(output_dir, datasets_test[0], dataset_train_name, datasets,       offset_base, offset_curr)
    concat_dataset(output_dir, datasets_test[0], dataset_valid_name, datasets_valid, offset_base, offset_curr)
    concat_dataset(output_dir, datasets_test[0], dataset_test_name,  datasets_test,  offset_base, offset_curr)

