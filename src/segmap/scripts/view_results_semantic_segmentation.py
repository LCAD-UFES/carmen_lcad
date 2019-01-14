
import numpy as np
import cv2
import os

#Labels para modelo *_cityscapes_*
LABEL_NAMES = np.asarray([
    'road', 
    'sidewalk', 
    'building', 
    'wall',
    'fence',
    'pole',
    'trafficlight',
    'trafficsign',
    'vegetation',
    'terrain',
    'sky',
    'person',
    'rider',
    'car',
    'truck',
    'bus',
    'train',
    'motorcycle',
    'bicycle'
])

def create_cityscapes_label_colormap():
  """Creates a label colormap used in CITYSCAPES segmentation benchmark.

  Returns:
    A Colormap for visualizing segmentation results.
  """
  colormap = np.asarray([
      [128, 64, 128],
      [244, 35, 232],
      [70, 70, 70],
      [102, 102, 156],
      [190, 153, 153],
      [153, 153, 153],
      [250, 170, 30],
      [220, 220, 0],
      [107, 142, 35],
      [152, 251, 152],
      [70, 130, 180],
      [220, 20, 60],
      [255, 0, 0],
      [0, 0, 142],
      [0, 0, 70],
      [0, 60, 100],
      [0, 80, 100],
      [0, 0, 230],
      [119, 11, 32],
  ])
  return colormap

  
colors = create_cityscapes_label_colormap()
 
  
def apply_colors(img):
	for i in range(img.shape[0]):
		for j in range(img.shape[1]):
			p = img[i, j, 0]
			if p >= 19:
			    color = [0, 0, 0]
			else:
			    color = [colors[p, 2], colors[p, 1], colors[p, 0]]
			img[i, j] = color
	return img
	


'''
	
imgs_dir = '/dados/logs/log_volta_da_ufes-20180112-2.txt_bumblebee_png'
xception_dir = '/dados/results_semantic_segmentation/xception71_cityscapes_trainfine/log_volta_da_ufes-20180112-2.txt_bumblebee_png/'
mobilenet_dir = '/dados/results_semantic_segmentation/mobilenetv2_cococityscapes/log_volta_da_ufes-20180112-2.txt_bumblebee_png/'

imgs = sorted([d for d in os.listdir(imgs_dir) if '-r' in d])
step = True

for ind in range(0, len(imgs), 4):
    i = imgs[ind]
    if '-r' in i:
        raw_img = cv2.imread(imgs_dir + '/' + i)
        xcep = cv2.imread(xception_dir + '/' + i.replace('-r', '-r_trainfine'))
        mob = cv2.imread(mobilenet_dir + '/' + i.replace('-r', '-r_mobile'))
        
        res_img = cv2.resize(raw_img, (640, 480))
        cropped_img = res_img[50:50+320, :]
        img = cv2.resize(cropped_img, (xcep.shape[1], xcep.shape[0]))
        
        xcep = apply_colors(xcep)
        mob = apply_colors(mob)
        
        view = np.concatenate([img, xcep, mob], axis=1)
        cv2.putText(view, 'Xception', (550, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        cv2.putText(view, 'MobileNet', (1060, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        cv2.imshow('segmentation', view)

        t = -1 if step else 1
        c = cv2.waitKey(t)

        #print(c, step, ord('s'))
        if c == 1048691: # ord('s'):
            step = not step
            
'''
cv2.imwrite("seg.png", apply_colors(cv2.imread('/home/filipe/workspace/papers/2018_segmap/fig/img2_seg.jpg')))
#cv2.waitKey(-1)


