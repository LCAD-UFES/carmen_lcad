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
import argparse

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def int_gt_zero(s):
    if not is_number(s) or float(s) != int(float(s)):
        msg = 'invalid int value: %r' % s
        raise argparse.ArgumentTypeError(msg)
    value = int(float(s))
    if value <= 0:
        msg = 'value must be greater than zero: %r' % s
        raise argparse.ArgumentTypeError(msg)
    return value

def path(s):
    if not os.path.isdir(s):
        msg = 'directory not found: %r' % s
        raise argparse.ArgumentTypeError(msg)
    return s

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
    parser = argparse.ArgumentParser(description='This program generates random training and test filelists for use with ENet neural network.',
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('dataset_dir', help='PNG sample files directory', type=path)
    parser.add_argument('n_train', help='number of subdirectories for training', type=int_gt_zero)
    parser.add_argument('n_test', help='number of subdirectories for testing', type=int_gt_zero)
    args = parser.parse_args()
    dataset_name = args.dataset_dir.split('/')[-1]
    ds = os.listdir(args.dataset_dir)
    i = len(ds)
    while i > 0:
        i -= 1
        pose_dir = args.dataset_dir + '/' + ds[i]
        if not os.path.isdir(pose_dir):
            del ds[i]
            continue
        pose = ds[i].split('_')
        if len(pose) != 2 or not is_number(pose[0]) or not is_number(pose[1]):
            del ds[i]
    
    print 'Dataset', args.dataset_dir, 'contains', len(ds), 'pose directories.'
    if len(ds) < (args.n_train + args.n_test):
        print 'Insufficient number of poses for training and test. Please modify the arguments.'
        exit(0)
    random.shuffle(ds)
    
    f_train_name = dataset_name + '_train.txt'
    ds, n_poses, n_files = generate_filelist(ds, args.dataset_dir, f_train_name, args.n_train)
    if n_poses < args.n_train:
        print 'Insufficient poses for training.'
        exit(0)
    
    f_test_name = dataset_name + '_test.txt'
    ds, n_poses, n_files = generate_filelist(ds, args.dataset_dir, f_test_name, args.n_test)
    if n_poses < args.n_test:
        print 'Insufficient poses for test.'
        exit(0)
