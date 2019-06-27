
import time
import cv2
import os

carmen_home = os.getenv("CARMEN_HOME")

import numpy as np
from PIL import Image, ImageEnhance

# Imports just from my module
from model import DeepLabModel
from visualize import vis_segmentation, label_to_color_image

saved_path = carmen_home + '/sharedlib/Deeplab/deeplab_model_cityscapes.tar.gz'
model = DeepLabModel(saved_path)

def process_image(png_filename):
    raw_img_teste = Image.open(png_filename)
    #enhancer = ImageEnhance.Brightness(raw_img_teste)
    #raw_img_teste = enhancer.enhance(1.0)
    width, height = raw_img_teste.size
    img_teste = raw_img_teste.crop((0, int((50/480)*height), width, height-int((110/480) * height)))
    #img_teste = raw_img_teste
    init = time.time()
    resized_im, seg_map = model.run(img_teste)
    print('Time to run model:', time.time() - init, end=' ')
    seg_map = np.asarray(seg_map.astype(np.uint8))
    seg_image = label_to_color_image(seg_map).astype(np.uint8)
    resized_im = cv2.resize(np.asarray(resized_im), (640, 480))
    seg_image = cv2.resize(np.asarray(seg_image), (640, 480))
    result = np.concatenate([resized_im[...,::-1], seg_image[...,::-1]], axis=1)
    cv2.imshow("result", result)
    cv2.waitKey(1)
    return seg_map, result

if __name__ == "__main__":
    from sys import argv, version_info
    if len(argv) < 3:
        print('\nUsage: python %s <file with a list of images> <output path>' % argv[0])
    else:
        pngs = open(argv[1], 'r').readlines()
        count = 0
        for filename in pngs:
            filename = filename.rstrip()
            s = filename.rsplit('/')
            if len(filename) < 2 or len(s) < 2:
                continue
            name = argv[2] + '/' + s[-1]
            result_name = argv[2] + '/result_' + s[-1]
            
            init = time.time()
            
            image_exists = False
            if os.path.exists(name) and os.path.exists(result_name):
                img_check = cv2.imread(name)
                if img_check is not None:
                    if img_check.shape[0] != 0 and img_check.shape[1] != 0:
                        image_exists = True
            
            if not image_exists:            
                seg_map, result = process_image(filename)
                cv2.imwrite(name, seg_map)
                cv2.imwrite(result_name, result)   

            consumed_time = time.time() - init 
            print(count, 'of', len(pngs), filename, 'consumed_time:', consumed_time)
            count += 1

