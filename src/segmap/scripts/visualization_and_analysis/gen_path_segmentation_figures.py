import cv2
import numpy as np
import os

n = 4

dirs = [d for d in os.listdir('/dados/data/') if d[:4] == "data" and d[-4:] == ".txt"]
for d in dirs:
    semantic_path = '/dados/data/' + d + '/semantic'
    imgs_path = sorted([f for f in os.listdir(semantic_path) if 'result' in f])
    step = int(len(imgs_path) / n + 1)
    imgs = [cv2.imread(semantic_path + '/' + imgs_path[i]) for i in range(0, len(imgs_path), step)]
    out_img = np.concatenate(imgs, axis=1)
    out_img_path = 'semantics_' + d.replace('.txt', '') + '.png'
    cv2.imwrite(out_img_path, out_img)
    print('Saved', out_img_path)

print('Done.')
        
        
