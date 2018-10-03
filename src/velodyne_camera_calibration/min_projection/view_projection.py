
import cv2
import numpy as np

def main():
    point3d = np.array([10.675, 1.829, -0.409])

    R = np.eye(3, 3)
    t = np.zeros((3, 1))
 
    point_cam = np.matmul(R, point3d) + t   

    fx_meters = 1.0
    fy_meters = 1.0
    pixel_size = 1.0

    px = fx_meters * (point_cam[1] / point_cam[0]) / pixel_size + cu
    py = fy_meters * (-point_cam[2] / point_cam[0]) / pixel_size + cv

    print(px, py)

    # img = cv2.imread("cam0_0000000000.png")

main()



