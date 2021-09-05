from os import system
import os.path
import sys
import cv2
import argparse
import threading
import numpy as np
import struct
import binascii

from numpy.core.defchararray import index
from numpy.core.shape_base import hstack

def GETINDEX(a):
    ret = int(a,base=16)
    return ret

def HEX_TO_SHORT(fourth,third,second,first):
    hex = ( fourth << 12 | (third << 8 | (second << 4 | first)))
    return hex

def check_setup(input_list, output_dir):
    if not os.path.isfile(input_list):
        raise Exception('The input_list is not a valid file!')
    if not os.path.isdir(output_dir):
        raise Exception('The output_dir is not a valid directory!')    


def save_one_img(img, img_size, dst_size, timestamp, camera_id, output_dir, max_height=None, ignore_top=0):
    img = img.reshape(img_size)[:, :, ::-1]  # transpose channels RGB -> BGR

    ori_size = img_size[:2][::-1]  # transpose height x width -> width x height

    if dst_size is None:
        dst_size = ori_size

    if dst_size != ori_size:
        img = cv2.resize(img, dst_size)

    if max_height is not None:
        img = img[ignore_top:max_height]
    else:
        img = img[ignore_top:]      

    img_fname = '{0}.intelbras{1}.png'.format(timestamp, camera_id)

    if not (os.path.isfile(os.path.join(output_dir, img_fname))):
        print('Saving image {} into {}'.format(img_fname, output_dir))
        cv2.imwrite(os.path.join(output_dir, img_fname), img)
    else:
        print('Skipping image {} into {}'.format(img_fname, output_dir))


def save_any_img(img_left, img_right, img_size, dst_size, timestamp, camera_id, output_dir, max_height=None, ignore_top=0):
    img_left = img_left.reshape(img_size)[:, :, ::-1]  # transpose channels RGB -> BGR
    img_right = img_right.reshape(img_size)[:, :, ::-1]  # transpose channels RGB -> BGR

    ori_size = img_size[:2][::-1]  # transpose height x width -> width x height

    if dst_size is None:
        dst_size = ori_size

    if dst_size != ori_size:
        img_left = cv2.resize(img_left, dst_size)
        img_right = cv2.resize(img_right, dst_size)

    if max_height is not None:
        img_left = img_left[ignore_top:max_height]
        img_right = img_right[ignore_top:max_height]
    else:
        img_left = img_left[ignore_top:]
        img_right = img_right[ignore_top:]

    img_left_fname = '{0}.bb{1}.l.png'.format(timestamp, camera_id)
    img_right_fname = '{0}.bb{1}.r.png'.format(timestamp, camera_id)
    
    if not (os.path.isfile(os.path.join(output_dir, img_left_fname)) and 
            os.path.isfile(os.path.join(output_dir, img_right_fname))):
        print('Saving images {} and {} into {}'.format(img_left_fname, img_right_fname, output_dir))
        cv2.imwrite(os.path.join(output_dir, img_left_fname), img_left)
        cv2.imwrite(os.path.join(output_dir, img_right_fname), img_right)
    else:
        print('Skipping images {} and {} into {}'.format(img_left_fname, img_right_fname, output_dir))

def save_old_img(image, output_dir, camera_id, dst_size=None, max_height=None, ignore_top=0):
    img_left = np.fromstring(image['left'], count=image['bytes'], dtype=np.uint8)
    img_right = np.fromstring(image['right'], count=image['bytes'], dtype=np.uint8)

    save_any_img(img_left, img_right, image['size'], dst_size, image['timestamp'], camera_id, output_dir, max_height, ignore_top)


def save_new_img(image, output_dir, camera_id, dst_size=None, max_height=None, ignore_top=0):
    img_handler = open(image['path'], 'rb')

    img_left = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)
    img_right = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)

    save_any_img(img_left, img_right, image['size'], dst_size, image['timestamp'], camera_id, output_dir, max_height, ignore_top)


def save_new_img2(image, output_dir, camera_id, dst_size=None, max_height=None, ignore_top=0):
    img_handler = open(image['path'], 'rb')

    img = np.fromfile(img_handler, count=image['bytes'], dtype=np.uint8)

    save_one_img(img, image['size'], dst_size, image['timestamp'], camera_id, output_dir, max_height, ignore_top)


#para logs cuja nuvem de pontos esta salva em arquivo separado
# usando os angulos lidos
def save_point_cloud_as_img2(image, output_dir, camera_id, dst_size=None, angle_left=180, angle_right=180):
    pointcloud_file = open(image['path'],'rb')
    shot_angle = ''
    shot_distance =''
    shot_intensity = ''
    line = pointcloud_file.read(8)
    col = 0
    blank_image = np.zeros((32,int(image['shots']),3), np.uint8)
    shot_angle_order = []
    while line: 
        shot_angle = struct.unpack('d',line)
        shot_angle_order.append(float(shot_angle[0])/100)
        shot_distance = struct.unpack('H'*32,pointcloud_file.read(64))
        shot_intensity = struct.unpack('B'*32,pointcloud_file.read(32))
        shot_distance = np.asarray(shot_distance)/500
        linha = 0
        for i in [31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0]:
            distancia = int(float(shot_distance[i]*765/25))
            B = 0 if distancia < 511 else distancia - 510
            G = 0 if distancia < 256 else distancia - B - 255
            R = 255 if distancia > 255 else distancia
            if distancia == 0:
                B = G = R = 255
            blank_image[linha,col] = (B,G,R)
            linha+=1
        col+=1
        line = pointcloud_file.read(8)
    indexes = np.asarray(shot_angle_order)
    half_point =np.abs(indexes - 180.0).argmin()
    left_part = blank_image[:,:half_point]
    right_part = blank_image[:,half_point:]
    centered_image = hstack((left_part,right_part))
    ini = int(float(image['shots']/360)*(180-abs(angle_left)))
    end = int(float(image['shots']/360)*(180+abs(angle_right)))    
    partial_image = centered_image[:,ini:end]
    resized = cv2.resize(partial_image, dst_size , interpolation = cv2.INTER_AREA)
    cv2.imwrite(output_dir+'/'+str(image['timestamp'])+'.png',resized)

#para logs cuja nuvem de pontos esta salva no proprio log
# usando os angulos lidos
def save_old_point_cloud_as_img2(image, output_dir, camera_id, dst_size=None, angle_left=180, angle_right=180):
    pointcloud_data = image['data'] 
    shot_angle = ''
    shot_distance =''
    shot_intensity = ''
    col = 0
    shot_angle_order=[]
    blank_image = np.zeros((32,int(image['shots']),3), np.uint8)
    for j in range(0,len(pointcloud_data),2):
        linha = 0
        shot_angle = float(pointcloud_data[j])
        shot_angle_order.append(float(shot_angle)/100)
        distance=[]
        for k in range(32):
            distance.append(HEX_TO_SHORT(GETINDEX(pointcloud_data[j+1][(k*6)+3]),GETINDEX(pointcloud_data[j+1][(k*6)+2]),GETINDEX(pointcloud_data[j+1][(k*6)+1]),GETINDEX(pointcloud_data[j+1][(k*6)])))
        
        distance = np.asarray(distance)/500
        
        for i in [31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0]:            
            distancia = int(float(distance[i])*765/25)
            B = 0 if distancia < 511 else distancia - 510
            G = 0 if distancia < 256 else distancia - B - 255
            R = 255 if distancia > 255 else distancia
            if distancia == 0:
                B = G = R = 255
            blank_image[linha,col] = (B,G,R)
            linha+=1
        col+=1
    indexes = np.asarray(shot_angle_order)
    half_point =np.abs(indexes - 180.0).argmin()
    left_part = blank_image[:,:half_point]
    right_part = blank_image[:,half_point:]
    centered_image = hstack((left_part,right_part))
    ini = int(float(image['shots']/360)*(180-abs(angle_left)))
    end = int(float(image['shots']/360)*(180+abs(angle_right)))    
    partial_image = centered_image[:,ini:end]
    resized = cv2.resize(partial_image, dst_size , interpolation = cv2.INTER_AREA)
    cv2.imwrite(output_dir+'/'+str(image['timestamp'])+'.png',resized)



#para logs cuja nuvem de pontos esta salva em arquivo separado
# usando as colunas e linhas ao inves dos angulos lidos
def save_point_cloud_as_img(image, output_dir, camera_id, dst_size=None, angle_left=180, angle_right=180):
    pointcloud_file = open(image['path'],'rb')
    shot_angle = ''
    shot_distance =''
    shot_intensity = ''
    line = pointcloud_file.read(8)
    col = 0
    if image['shots'] > 1000:
        blank_image = np.zeros((32,int(image['shots']),3), np.uint8)
        while line: 
            shot_angle = struct.unpack('d',line)
            shot_distance = struct.unpack('H'*32,pointcloud_file.read(64))
            shot_intensity = struct.unpack('B'*32,pointcloud_file.read(32))
            shot_distance = np.asarray(shot_distance)/500
            linha = 0
            for i in [31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0]:
                distancia = int(float(shot_distance[i]*765/25))
                B = 0 if distancia < 511 else distancia - 510
                G = 0 if distancia < 256 else distancia - B - 255
                R = 255 if distancia > 255 else distancia
                if distancia == 0:
                    B = G = R = 255
                blank_image[linha,col] = (B,G,R)
                linha+=1
            col+=1
            line = pointcloud_file.read(8)
        ini = int(float(image['shots']/360)*(180-abs(angle_left)))
        end = int(float(image['shots']/360)*(180+abs(angle_right)))
        partial_image = blank_image[:,ini:end]
        resized = cv2.resize(partial_image, dst_size , interpolation = cv2.INTER_AREA)
        cv2.imwrite(output_dir+'/'+str(image['timestamp'])+'.png',resized)

#para logs cuja nuvem de pontos esta salva no proprio log
# usando as colunas e linhas ao inves dos angulos lidos
def save_old_point_cloud_as_img(image, output_dir, camera_id, dst_size=None, angle_left=180, angle_right=180):
    pointcloud_data = image['data'] 
    shot_angle = ''
    shot_distance =''
    shot_intensity = ''
    col = 0
    blank_image = np.zeros((32,int(image['shots']),3), np.uint8)
    for j in range(0,len(pointcloud_data),2):
        linha = 0
        distance=[]
        for k in range(32):
            distance.append(HEX_TO_SHORT(GETINDEX(pointcloud_data[j+1][(k*6)+3]),GETINDEX(pointcloud_data[j+1][(k*6)+2]),GETINDEX(pointcloud_data[j+1][(k*6)+1]),GETINDEX(pointcloud_data[j+1][(k*6)])))
        
        distance = np.asarray(distance)/500
        
        for i in [31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0]:            
            distancia = int(float(distance[i])*765/25)
            B = 0 if distancia < 511 else distancia - 510
            G = 0 if distancia < 256 else distancia - B - 255
            R = 255 if distancia > 255 else distancia
            if distancia == 0:
                B = G = R = 255
            blank_image[linha,col] = (B,G,R)
            linha+=1
        
        
        col+=1
    ini = int(float(image['shots']/360)*(180-abs(angle_left)))
    end = int(float(image['shots']/360)*(180+abs(angle_right)))
    partial_image = blank_image[:,ini:end]
    resized = cv2.resize(partial_image, dst_size , interpolation = cv2.INTER_AREA)
    cv2.imwrite(output_dir+'/'+str(image['timestamp'])+'.png',resized)

#para logs cuja nuvem de pontos esta salva em arquivo separado
def read_point_cloud_log(log, input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, angle_left=180, angle_right=180):

    total = 0
    total_ig = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
        
    while line and total < max_lines:
        item = line.strip().split()
        
        timestamp = item[3]
        path = log + item[1]
        if item[1].find("/dados/") >=0 :
            path = item[1]
        
        image = {
            'path': path,
            'shots': (int(item[2])),
            'timestamp': timestamp
        }
        if image['shots'] > 1030:
            line = f.readline()
            t = threading.Thread(target=save_point_cloud_as_img2, args=(image, output_dir, camera_id, dst_size, angle_left, angle_right))
            mythreads.append(t)
            t.start()
            #break
            total += 1
            if (len(mythreads) >= max_threads) or not (line and total < max_lines):
                for t in mythreads:
                    t.join()
                mythreads[:] = []  # clear the thread's list
                print('Saved {} point cloud already...'.format(total))
        else:
            total_ig += 1
            print('Ignored {} point cloud: low points '.format(total_ig))
    f.close()

#para logs cuja nuvem de pontos esta salva no proprio log
def read_old_point_cloud_log(log, input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, angle_left=180, angle_right=180):

    total = 0
    total_ig = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
        
    while line and total < max_lines:
        item = line.strip().split()
        
        timestamp = item[-3]
        points = item[2:-3]
        
        image = {
            'data': points,
            'shots': (int(item[1])),
            'timestamp': timestamp
        }
        if image['shots'] > 1030:
            line = f.readline()
        
            t = threading.Thread(target=save_old_point_cloud_as_img2, args=(image, output_dir, camera_id, dst_size, angle_left, angle_right))
            mythreads.append(t)
            t.start()
            #break
            total += 1
            if (len(mythreads) >= max_threads) or not (line and total < max_lines):
                for t in mythreads:
                    t.join()
                mythreads[:] = []  # clear the thread's list
                print('Saved {} point cloud already...'.format(total))
        else:
            total_ig += 1
            print('Ignored {} point cloud: low points '.format(total_ig))
    f.close()


def read_old_log(input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, max_height=None, ignore_top=0):
    total = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
    
    print('max lines:{}'.format(max_lines))
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
        t = threading.Thread(target=save_old_img, args=(image, output_dir, camera_id, dst_size, max_height, ignore_top))
        mythreads.append(t)
        t.start()
        total += 1
        if (len(mythreads) >= max_threads) or not (line and total < max_lines):
            for t in mythreads:
                t.join()
            mythreads[:] = []  # clear the thread's list
            print('Saved {} images already...'.format(total))
        print("loop ainda")
    f.close()
    print("sai do readlog")


def read_new_log(input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, max_height=None, ignore_top=0):
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
        t = threading.Thread(target=save_new_img, args=(image, output_dir, camera_id, dst_size, max_height, ignore_top))
        mythreads.append(t)
        t.start()
        total += 1
        if (len(mythreads) >= max_threads) or not (line and total < max_lines):
            for t in mythreads:
                t.join()
            mythreads[:] = []  # clear the thread's list
            print('Saved {} images already...'.format(total))
    f.close()


def read_new2_log(log, input_list, output_dir, max_threads, max_lines, camera_id, dst_size=None, max_height=None, ignore_top=0):
    total = 0
    mythreads = []
    f = open(input_list, 'rb')
    line = f.readline()
        
    while line and total < max_lines:
        item = line.strip().split()
        
        timestamp = float(item[4])
        high_level_subdir = int(int(timestamp / 10000.0) * 10000.0)
        low_level_subdir = int(int(timestamp / 100.0) * 100.0)
        path = log + "_images/" + str(high_level_subdir) + "/" + str(low_level_subdir) + "/" + item[4] + "_camera" + str(camera_id) + "_0.image"   
        
        image = {
            'path': path,
            'size': (int(item[8]), int(item[7]), 3),
            'bytes': int(item[6]),
            'timestamp': item[4].decode('utf-8'),
        }
        line = f.readline()
        t = threading.Thread(target=save_new_img2, args=(image, output_dir, camera_id, dst_size, max_height, ignore_top))
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
    parser.add_argument('-g', '--log', type=str, required=True)
    parser.add_argument('-o', '--output_dir', type=str, required=True)
    parser.add_argument('-c', '--camera_id', type=int, required=False, default=1)
    parser.add_argument('-f', '--log_format', type=int, required=True)
    parser.add_argument('-l', '--max_lines', type=int, required=False, default=10000000)
    parser.add_argument('-t', '--max_threads', type=int, required=False, default=10)
    parser.add_argument('-m', '--max_height', type=int, required=False, default=0)
    parser.add_argument('-s', '--image_size', type=str, required=False, default='none')
    parser.add_argument('-p', '--ignore_top', type=int, required=False, default=0)
    parser.add_argument('-al', '--angle_left', type=int, required=False, default=180)
    parser.add_argument('-ar', '--angle_right', type=int, required=False, default=180)
    argv = vars(parser.parse_args())

    check_setup(argv['input_list'], argv['output_dir'])

    max_height = argv['max_height'] if argv['max_height'] > 0 else None
    dst_size = tuple(map(int, argv['image_size'].split('x'))) if argv['image_size'] != 'none' else None
    ignore_top = argv['ignore_top']
    
    print('Destination size: {}'.format(dst_size))

    if argv['log_format'] == 0: # 0-Old Log Format
        read_old_log(argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, max_height, ignore_top)
    elif argv['log_format'] == 1:  # 1-New Log Format
        read_new_log(argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, max_height, ignore_top)
    elif argv['log_format'] == 2:
        read_new2_log(argv['log'], argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, max_height, ignore_top)
    elif argv['log_format'] == 3:
        read_point_cloud_log(argv['log'], argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, argv['angle_left'], argv['angle_right'])
    else:
        read_old_point_cloud_log(argv['log'], argv['input_list'], argv['output_dir'], argv['max_threads'], argv['max_lines'], argv['camera_id'], dst_size, argv['angle_left'], argv['angle_right'])
    print('out')
