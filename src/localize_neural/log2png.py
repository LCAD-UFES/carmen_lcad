import os
import cv2
import argparse
import threading
import numpy as np
from pprint import pprint


def check_setup(input_list, output_dir):
    if not os.path.isfile(input_list):
        raise Exception('The input_list is not a valid file!')
    if not os.path.isdir(output_dir):
        raise Exception('The output_dir is not a valid directory!')    


def main(input_list, output_dir, max_threads, camera_id, dst_size):
    check_setup(input_list, output_dir)

    with open(input_list, 'rb') as f:
        image_list = [line.strip().split() for line in f.readlines()]
        image_list = [
            {
                'path': item[1].decode('utf-8'), 
                'size': (int(item[3]), int(item[2]), 3), 
                'bytes': int(item[4]),
                'timestamp': item[6].decode('utf-8'), 
            } 
            for item in image_list
        ]

    if len(image_list) == 0:
        raise Exception('The input_list has 0 lines!')

    mythreads = []
    total = 0
    for i, image in enumerate(image_list):
        t = threading.Thread(target=thread_save_images, args=(image, output_dir, max_threads, camera_id, dst_size))
        mythreads.append(t)
        t.start()
        if (len(mythreads) >= max_threads) or (i == len(image_list) - 1):
            for t in mythreads:
                t.join()
            total += len(mythreads)
            mythreads[:] = []  # clear the thread's list
            print('Saved {}/{} images already...'.format(total, len(image_list)))


def thread_save_images(image, output_dir, max_threads, camera_id, dst_size=None):
    with open(image['path'], 'rb') as img_handler:
            img_left = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)
            img_right = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)
            img_left = img_left.reshape(image['size'])[:,:,::-1]
            img_right = img_right.reshape(image['size'])[:,:,::-1]
            
            if dst_size is None:
                dst_size = img_right.shape[:2] #height x width
            else:
                dst_size = dst_size[::-1] #transpose width x height -> height x width
                
            if dst_size != img_right.shape[:2]:
                img_left = cv2.resize(img_left, dst_size)
                img_right = cv2.resize(img_right, dst_size)
            
            img_left_fname = '{0}.bb{1}.l.png'.format(image['timestamp'], camera_id)
            img_right_fname = '{0}.bb{1}.r.png'.format(image['timestamp'], camera_id)
            if max_threads <= 10:
                print('Saving images {} and {} into {}'.format(img_left_fname, img_right_fname, output_dir))
            cv2.imwrite(os.path.join(output_dir, img_left_fname), img_left)
            cv2.imwrite(os.path.join(output_dir, img_right_fname), img_right)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert images from LOG to PNG')
    parser.add_argument('-i', '--input_list', type=str, required=True)
    parser.add_argument('-o', '--output_dir', type=str, required=True)
    parser.add_argument('-c', '--camera_id', type=int, required=True)
    parser.add_argument('-t', '--max_threads', type=int, required=False, default=10)
    parser.add_argument('-s', '--size', type=str, required=False, default='none')
    argv = vars(parser.parse_args())

    dst_size = tuple(map(int, argv['size'].split('x'))) if argv['size'] != 'none' else None
    print('Destination size: {}'.format(dst_size)) 
    main(argv['input_list'], argv['output_dir'], argv['max_threads'], argv['camera_id'], dst_size)

