#!/bin/python

import cv2
import glob
import numpy as np


def rot90(m, k=1, axes=(0,1)):
    axes = tuple(axes)
    if len(axes) != 2:
        raise ValueError("len(axes) must be 2.")

    m = np.asanyarray(m)

    if axes[0] == axes[1] or np.absolute(axes[0] - axes[1]) == m.ndim:
        raise ValueError("Axes must be different.")

    if (axes[0] >= m.ndim or axes[0] < -m.ndim
        or axes[1] >= m.ndim or axes[1] < -m.ndim):
        raise ValueError("Axes={} out of range for array of ndim={}."
            .format(axes, m.ndim))

    k %= 4

    if k == 0:
        return m[:]
    if k == 2:
        return flip(flip(m, axes[0]), axes[1])

    axes_list = np.arange(0, m.ndim)
    (axes_list[axes[0]], axes_list[axes[1]]) = (axes_list[axes[1]],
                                                axes_list[axes[0]])

    if k == 1:
        return np.transpose(flip(m,axes[1]), axes_list)
    else:
        # k == 3
        return flip(transpose(m, axes_list), axes[1])

def flip(m, axis):
    if not hasattr(m, 'ndim'):
        m = np.asarray(m)
    indexer = [slice(None)] * m.ndim
    try:
        indexer[axis] = slice(None, None, -1)
    except IndexError:
        raise ValueError("axis=%i is invalid for the %i-dimensional input array"
                         % (axis, m.ndim))
    return m[tuple(indexer)]

def rotate_image(mat, angle):
  # angle in degrees

  height, width = mat.shape[:2]
  image_center = (width/2, height/2)

  rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

  abs_cos = abs(rotation_mat[0,0])
  abs_sin = abs(rotation_mat[0,1])

  bound_w = int(height * abs_sin + width * abs_cos)
  bound_h = int(height * abs_cos + width * abs_sin)

  rotation_mat[0, 2] += bound_w/2 - image_center[0]
  rotation_mat[1, 2] += bound_h/2 - image_center[1]

  rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h), cv2.INTER_NEAREST)
  return rotated_mat


#img = cv2.imread('test.jpg')
#modificada = rotate_image(img,90)
#cv2.imwrite('modif.jpg', modificada)

path = glob.glob("classificadas/*.png")
codigos = {
        "101010000":1,
        "000010101":2,
        "100010100":3,
        "001010001":4,
        "010010010":5,
        "000111000":6,
        "100010001":7,
        "001010100":8
    }
f=open("classificadas/database_classified.txt", "a+")


#Le as imagens
for imgp in path:
    n = cv2.imread(imgp)
    img = imgp[14:]
    print img
    mat= np.matrix(list(img[7:16])).reshape((3, 3))
   
    if int(img[-5]) <= 4:
        #Vai ser girado e gravado 3 vezes
        modificada = n
        rot = mat
        for i in range(3):
            rot = rot90(rot, k=1, axes=(1,0))
            rotname = rot.tostring()
            modificada = rotate_image(modificada,90)
            f.write('database_classes/'+img[:6]+'_'+rotname+'__'+str(codigos.get(rotname))+'.png'+' '+rotname+'\n')
            cv2.imwrite('modif/'+img[:6]+'_'+rotname+'__'+str(codigos.get(rotname))+'.png', modificada)
    else:
        #Vai ser girado uma unica vez
        
        rot = rot90(mat, k=1, axes=(1,0))
        rotname = rot.tostring()
        modificada = rotate_image(n,90)
        f.write('database_classes/'+img[:6]+'_'+rotname+'__'+str(codigos.get(rotname))+'.png'+' '+rotname+'\n')
        cv2.imwrite('modif/'+img[:6]+'_'+rotname+'__'+str(codigos.get(rotname))+'.png', modificada)


f.close()
