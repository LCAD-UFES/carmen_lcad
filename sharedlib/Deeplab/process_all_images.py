
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


def process_image(raw_img_teste):
    #raw_img_teste = Image.open(png_filename)
    #enhancer = ImageEnhance.Brightness(raw_img_teste)
    #raw_img_teste = enhancer.enhance(1.0)

    raw_img_teste = Image. fromarray(raw_img_teste)
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


def read_carmen_img(path, nr, nc):
    f = open(path, "rb")
    
    size = 3 * nr * nc
    f.seek(size)
    img = np.fromfile(f, dtype=np.uint8).reshape((nr, nc, 3))
    #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    f.close()

    return img
    

if __name__ == "__main__":
    from sys import argv, version_info

    if len(argv) < 2:
        print('\nUsage: python %s <file with a list of images (obtained with "grep BUMB log")>' % argv[0])
    else:
        all_lines = open(argv[1], 'r').readlines()
        count = 0
        
        for line in all_lines:
            line = line.rstrip().rsplit()

            img_path = line[1]
            nc = int(line[2])
            nr = int(line[3])
            
            spath = [x for x in img_path.rsplit('/') if len(x) > 2]

            log_name = spath[1].replace("_bumblebee", "")
            data_dir = "/dados/data/data_" + log_name
            semantic_dir = data_dir + "/semantic/"

            img_name = spath[-1]
            output_img_name = img_name.replace(".bb3.image", "-r.png")
            result_img_name = "result_" + output_img_name
            output_path = semantic_dir + "/" + output_img_name
            result_path = semantic_dir + "/" + result_img_name
            
            if not os.path.exists(data_dir):
                os.mkdir(data_dir)
                
            if not os.path.exists(semantic_dir):
                os.mkdir(semantic_dir)
                
            init = time.time()

            img = read_carmen_img(img_path, nr, nc)

            if not os.path.exists(output_path):
                seg_map, result = process_image(img)
                cv2.imwrite(output_path, seg_map)
                cv2.imwrite(result_path, result)

            consumed_time = time.time() - init 
            print(count, 'of', len(all_lines), img_path, "result:", result_path, 'consumed_time:', consumed_time)
            count += 1




