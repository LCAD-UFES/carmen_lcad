
import math
import numpy as np

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

R = np.array([[-0.0127058,  -0.999762,  0.0177392],
 [0.0489445,  0.0170975,   0.998655],
 [-0.998721,   0.013557,  0.0487156]])

#R = np.array([[0.998721,  -0.013557, -0.0487156],
# [0.0127058,   0.999762, -0.0177392],
# [0.0489445,  0.0170975,   0.998655]])

print(rotationMatrixToEulerAngles(R))


