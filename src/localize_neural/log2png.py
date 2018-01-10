import os.path
import sys
import cv2
import argparse
import threading
import numpy as np


def check_setup(input_list, output_dir):
    if not os.path.isfile(input_list):
        raise Exception('The input_list is not a valid file!')
    if not os.path.isdir(output_dir):
        raise Exception('The output_dir is not a valid directory!')    


def save_any_img(img_left, img_right, img_size, dst_size, timestamp, camera_id, output_dir, max_height=None):
    img_left = img_left.reshape(img_size)[:, :, ::-1]  # transpose channels RGB -> BGR
    img_right = img_right.reshape(img_size)[:, :, ::-1]  # transpose channels RGB -> BGR

    ori_size = img_size[:2][::-1]  # transpose height x width -> width x height

    if dst_size is None:
        dst_size = ori_size

    if dst_size != ori_size:
        img_left = cv2.resize(img_left, dst_size)
        img_right = cv2.resize(img_right, dst_size)

    if max_height is not None:
        img_left = img_left[0:max_height]
        img_right = img_right[0:max_height]

    img_left_fname = '{0}.bb{1}.l.png'.format(timestamp, camera_id)
    img_right_fname = '{0}.bb{1}.r.png'.format(timestamp, camera_id)
    if not (os.path.isfile(os.path.join(output_dir, img_left_fname)) and 
            os.path.isfile(os.path.join(output_dir, img_right_fname))):
        print('Saving images {} and {} into {}'.format(img_left_fname, img_right_fname, output_dir))
        cv2.imwrite(os.path.join(output_dir, img_left_fname), img_left)
        cv2.imwrite(os.path.join(output_dir, img_right_fname), img_right)
    else:
        print('Skipping images {} and {} into {}'.format(img_left_fname, img_right_fname, output_dir))

def save_old_img(image, output_dir, camera_id, dst_size=None, max_height=None):
    img_left = np.fromstring(image['left'], count=image['bytes'], dtype=np.uint8)
    img_right = np.fromstring(image['right'], count=image['bytes'], dtype=np.uint8)

    save_any_img(img_left, img_right, image['size'], dst_size, image['timestamp'], camera_id, output_dir, max_height)


def save_new_img(image, output_dir, camera_id, dst_size=None, max_height=None):
    img_handler = open(image['path'], 'rb')

    img_left = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)
    img_right = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)

    save_any_img(img_left, img_right, image['size'], dst_size, image['timestamp'], camera_id, output_dir, max_height)


def read_old_log(input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, max_height=None):
    total = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
    while line and total < max_lines:
        item = line.strip().split()
        if len(item) < 8:
        	continue
        image = {
                'right': item[5].decode('hex'),
                'left': item[6].decode('hex'),
                'size': (int(item[2]), int(item[1]), 3),
                'bytes': int(item[3]),
                'timestamp': item[7].decode('utf-8')
        }
        line = f.readline()
        t = threading.Thread(target=save_old_img, args=(image, output_dir, camera_id, dst_size, max_height))
        mythreads.append(t)
        t.start()
        total += 1
        if (len(mythreads) >= max_threads) or not (line and total < max_lines):
            for t in mythreads:
                t.join()
            mythreads[:] = []  # clear the thread's list
            print('Saved {} images already...'.format(total))
    f.close()


def read_new_log(input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, max_height=None):
    total = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
    while line and total < max_lines:
        item = line.strip().split()
        image = {
            'path': item[1].decode('utf-8'),
            'size': (int(item[3]), int(item[2]), 3),
            'bytes': int(item[4]),
            'timestamp': item[6].decode('utf-8'),
        }
        line = f.readline()
        t = threading.Thread(target=save_new_img, args=(image, output_dir, camera_id, dst_size, max_height))
        mythreads.append(t)
        t.start()
        total += 1
        if (len(mythreads) >= max_threads) or not (line and total < max_lines):
            for t in mythreads:
                t.join()
            mythreads[:] = []  # clear the thread's list
            print('Saved {} images already...'.format(total))
    f.close()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert images from LOG to PNG')
    parser.add_argument('-i', '--input_list', type=str, required=True)
    parser.add_argument('-o', '--output_dir', type=str, required=True)
    parser.add_argument('-c', '--camera_id', type=int, required=True)
    parser.add_argument('-f', '--log_format', type=int, required=True)
    parser.add_argument('-l', '--max_lines', type=int, required=False, default=sys.maxint)
    parser.add_argument('-t', '--max_threads', type=int, required=False, default=10)
    parser.add_argument('-m', '--max_height', type=int, required=False, default=0)
    parser.add_argument('-s', '--image_size', type=str, required=False, default='none')
    argv = vars(parser.parse_args())

    check_setup(argv['input_list'], argv['output_dir'])

    max_height = argv['max_height'] if argv['max_height'] > 0 else None
    dst_size = tuple(map(int, argv['image_size'].split('x'))) if argv['image_size'] != 'none' else None

    print('Destination size: {}'.format(dst_size))

    if argv['log_format'] == 1:     # 1-New Log Format
        read_new_log(argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, max_height)
    else:   # 0-Old Log Format
        read_old_log(argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, max_height)

