
import numpy as np
import os
import cv2


dataset = 'data_log_estacionamentos-20181130.txt'
d = '/dados/maps/map_' + dataset 
origins = np.array([f.replace('.png', '').rsplit('_')[1:3] for f in os.listdir(d) if f[-3:] == 'png']).astype(np.float)
min_h, max_h = np.min(origins[:, 0]), np.max(origins[:, 0])
min_w, max_w = np.min(origins[:, 1]), np.max(origins[:, 1])

print(origins)
print(min_h, max_h, min_w, max_w)

concated = []
for h in np.arange(min_h, max_h, 50):
    imgs = []
    for w in np.arange(min_w, max_w, 50):
        name = d + '/semantic_%f_%f.png' % (h, w)
        if os.path.exists(name):
            imgs.append(cv2.imread(name))
        else:
            imgs.append(np.ones((250, 250, 3)) * 128)

    concated.append(np.concatenate(imgs, axis=1))
    
cv2.imwrite('img_map_%s.png' % dataset, np.concatenate(concated, axis=0))






