
import cv2
import numpy as np

"""
This code is an example of how to project a 3d point into an image.
The matrices cames from kitti's calibration files.
For more information regarding the meaning of the matrices refer to the file "useful_information_from_kitti.txt", section "example transformations".
"""

def main():
    # 3d position of the truck's right mirror 
    p3d = np.array([10.675, 1.829, -0.409, 1.])

    # Camera projection matrix
    Proj = np.array([[7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00], 
                     [0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00], 
                     [0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00]])

    # Transformation due to rectification
    Rect = np.array([[9.999239e-01, 9.837760e-03, -7.445048e-03, 0.], 
                     [-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.],
                     [7.402527e-03, 4.351614e-03, 9.999631e-01, 0.],
                     [0., 0., 0., 1.]])

    # Transform from velodyne to camera
    R = np.array([[7.533745e-03, -9.999714e-01, -6.166020e-04], 
                  [1.480249e-02, 7.280733e-04, -9.998902e-01], 
                  [9.998621e-01, 7.523790e-03, 1.480755e-02],
                  [0., 0., 0.]])
    t = np.array([-4.069766e-03, -7.631618e-02, -2.717806e-01, 1.]).reshape((4, 1))
    V2C = np.hstack((R, t))

    # Project 3d point to camera (Y = P_rect_xx * R_rect_00 * (R|T)_velo_to_cam * X)
    p = np.matmul(V2C, p3d)
    p = np.matmul(Rect, p)
    pcam = np.matmul(Proj, p)

    # Convert from homogeneous coordinates to pixel positions.
    px = int(pcam[0] / pcam[2])
    py = int(pcam[1] / pcam[2])

    # View result
    print(p3d, "->", (px, py))
    img = cv2.imread("cam0_0000000000.png")    
    cv2.circle(img, (px, py), 5, (0, 255, 0), -1)
    cv2.imshow("img", img)
    cv2.waitKey(-1)

main()



