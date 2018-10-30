import os
import cv2
import numpy as np

#exemplo de testes, colocar nos arquivos de treino e teste.
path = "/dados/neural_mapper/60mts/labels/"

list_images = os.listdir(path)
list_images = filter(lambda x : x.split('_')[1] == 'label.png',list_images)
for name_image in list_images:
    img = cv2.imread(path + name_image)
    height, width = img.shape[:2]
    img_map = np.zeros((width,height,3), np.uint8)
    #print(np.where((img == [1]).all(axis = 2)))
    img_map[np.where((img == [1]).all(axis = 2))] = np.array([255,120,0])
    img_map[np.where((img == [2]).all(axis = 2))] = np.array([255,255,255])
    img_map[np.where((img == [3]).all(axis = 2))] = np.array([0,0,0])
#    print(path + "map_" + name_image)   
    cv2.imwrite(path + "map_" + name_image, img_map)
#   cv2.imshow('image',img_map)
#    key = cv2.waitKey(0) & 0xFF
#    print(key)
#    if key == 27:
#        exit()

