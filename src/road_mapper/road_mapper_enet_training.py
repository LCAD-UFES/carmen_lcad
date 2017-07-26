'''
Input: This program generates random training and test lists for use in ENet neural network.
    The following command line parameters are expected:
    - dataset directory,
    - number of poses to be randomly selected to the training list,
    - number of poses to be randomly selected to the test list.
For each pose selected, all available offsets and rotations will be included in the lists. The remission and the road map images must be both available.
Output: 
    - training list,
    - test list.
'''
import sys
import os
import random

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def generate_filelist(ds, ds_dir, f_name, n_samples):
    filelist = open(f_name, 'w')
    n_valid_poses = 0
    n_valid_files_total = 0
    for i in range(len(ds)):
        if n_valid_poses >= n_samples:
            break
        pose_dir = ds_dir + '/' + ds[0]
        del ds[0]
        n_valid_files = 0
        for f in os.listdir(pose_dir):
            if f[0] != 'i' or f[-4:] != '.png':
                continue
            fi_name = os.path.abspath(pose_dir + '/' + f)
            fr_name = os.path.abspath(pose_dir + '/r' + f[1:])
            if not os.path.isfile(fi_name) or not os.path.isfile(fr_name):
                continue
            filelist.write(fi_name + ' ' + fr_name + '\n')
            n_valid_files += 1
        if n_valid_files == 0:
            print 'Pose directory', pose_dir, 'does not contain valid files in format i*.png and r*.png'
            continue
        n_valid_files_total += n_valid_files
        n_valid_poses += 1
    filelist.close()
    print 'File', f_name, 'contains', n_valid_poses, 'pose directories and', n_valid_files_total, 'total files.'
    return ds, n_valid_poses, n_valid_files_total

if __name__ == "__main__":
    USAGE = '<dataset directory> <number of poses for training> <number of poses for test>'
    if len(sys.argv) < 4:
        print 'Usage:\npython', sys.argv[0], USAGE
        if len(sys.argv) == 1 or (len(sys.argv) > 1 and sys.argv[1] != '-h' and sys.argv[1] != '--help'):
            print 'Insufficient number of arguments.'
        exit(0)
    if len(sys.argv) > 4:
        print 'Usage:\npython', sys.argv[0], USAGE
        print 'Excessive number of arguments.'
        exit(0)
    dataset_dir = sys.argv[1]
    dataset_name = dataset_dir.split('/')
    if not os.path.isdir(dataset_dir):
        print 'Usage:\npython', sys.argv[0], USAGE
        print "Dataset directory not found."
        exit(0)
    n_train = 0
    if is_number(sys.argv[2]):
        n_train = int(sys.argv[2])
    if n_train < 1:
        print 'Usage:\npython', sys.argv[0], USAGE
        print "Number of poses for training must be at least 1."
        exit(0)
    n_test = 0
    if is_number(sys.argv[3]):
        n_test = int(sys.argv[3])
    if n_test < 1:
        print 'Usage:\npython', sys.argv[0], USAGE
        print "Number of poses for test must be at least 1."
        exit(0)
    
    ds = os.listdir(dataset_dir)
    i = len(ds)
    while i > 0:
        i -= 1
        pose_dir = dataset_dir + '/' + ds[i]
        if not os.path.isdir(pose_dir):
            del ds[i]
        pose = ds[i].split('_')
        if len(pose) != 2 or not is_number(pose[0]) or not is_number(pose[1]):
            del ds[i]
    
    print 'Dataset', dataset_dir, 'contains', len(ds), 'pose directories.'
    if len(ds) < (n_train + n_test):
        print 'Insufficient number of poses for training and test. Please modify the arguments.'
        exit(0)
    random.shuffle(ds)
    
    f_train_name = dataset_name[-1] + '_train.txt'
    ds, n_poses, n_files = generate_filelist(ds, dataset_dir, f_train_name, n_train)
    if n_poses < n_train:
        print 'Insufficient poses for training.'
        exit(0)
    
    f_test_name = dataset_name[-1] + '_test.txt'
    ds, n_poses, n_files = generate_filelist(ds, dataset_dir, f_test_name, n_test)
    if n_poses < n_test:
        print 'Insufficient poses for test.'
        exit(0)
