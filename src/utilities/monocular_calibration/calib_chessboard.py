# import matplotlib.pyplot as plt
# import matplotlib as mpl
# from scipy import misc
import numpy as np
import argparse
import imageio
import glob
import PIL
import os
import cv2
from cv2 import aruco



def rectify_images(images, mtx, dist, outdir):

	#######################
	# RECTIFY IMAGES LIST #
	#######################
	#
	# rectifies images from a given image list and saves it in a given outdir.
	#
	# PARAMETERS
	# images: image path list
	# mtx: camera matrix
	# dist: distortion coefficients vector
	# outdir: output directory

	print('rectifying images... ', end='')

	# get image size
	img = cv2.imread(images[0])
	size = img.shape

	# calculate undistortion maps
	map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, mtx, (size[1],size[0]), cv2.CV_16SC2)

	# check if folder exists
	if not os.path.isdir(outdir):
		os.mkdir(outdir)

	# undistort each image and save them to folder
	# undistortion is done by using map1 and map2, and bilinear interpolation
	for fname in images:
		img = cv2.imread(fname)
		#dst = cv2.undistort(img, mtx, dist, None, mtx)
		dst = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
		out_path = outdir + fname
		cv2.imwrite(out_path, dst)

	print('done.')


def get_chessboard_corners(images):

	##########################
	# GET CHESSBOARD CORNERS #
	##########################
	#
	# identify chessboard corners to be used for calibration.
	#
	# PARAMETERS
	# images: images list
	#
	# RETURNS
	# objpoints: 3d points in real world space
	# imgpoints: 2d points in image plane


	print('processing chessboard corners... ', end='')

	# criteria for subpixel precision
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

	# create object points
	objp = np.zeros((6 * 9, 3), np.float32)
	objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

	# Arrays to store object points and image points from all the images.
	objpoints = []
	imgpoints = []

	img = cv2.imread(images[0])

	# iterate through images
	for fname in images:

		# read image
		img = cv2.imread(fname)

		# convert to grayscale
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# find the chessboard corners
		ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

		# if found, add object points, image points (after refining them)
		if ret == True:

			# append object point to objpoints
			objpoints.append(objp)

			# subpixel precision to corners given by criteria
			corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

			# append corners to imgpoints
			imgpoints.append(corners2)

			print(objp)
			print(corners2)
			print('----')

			# draw and display the corners
			img = cv2.drawChessboardCorners(img, (9, 6), corners2, ret)
			#cv2.imshow('img', img)
			#cv2.waitKey(100)

	cv2.destroyAllWindows()

	print('done.')

	return objpoints, imgpoints



#def get_rectification_error():


def main():
	outdir = '/home/ecotech4/Documents/camera/rectified/'
	indir = '/home/ecotech4/Documents/camera/chessboard/'
	images = glob.glob(indir + '*.bmp')

	# get chessboard points in 2d and 3d
	objpoints, imgpoints = get_chessboard_corners(images)

	#########################
	# CALIBRATES THE CAMERA #
	#########################
	# ret: the overall RMS re-projection error
	# mtx: camera matrix (cointains fx, fy, cx, cy)
	# dist: distortion coefficients vector
	# tvecs: translation coefficients vector
	print('calibrating camera... ', end='')
	img = cv2.imread(images[0])
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	reproj_error, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
	print('done.')

	# undistort images and save them to output directory
	rectify_images(images, mtx, dist, outdir)

	# todo calculate rectification error
	# get_rectification_error()

	# todo save calibration parameters
	f = open('/home/ecotech4/Documents/camera/intelbras-vip3320b.ini', "w+")
	f.write("fx " + str(mtx[0][0]) + '\n')
	f.write("fy " + str(mtx[1][1]) + '\n')
	f.write("cx " + str(mtx[0][2]) + '\n')
	f.write("cy " + str(mtx[1][2]) + '\n')
	f.write("k1 " + str(dist[0][0]) + '\n')
	f.write("k2 " + str(dist[0][1]) + '\n')
	f.write("p1 " + str(dist[0][2]) + '\n')
	f.write("p2 " + str(dist[0][3]) + '\n')
	f.write("k3 " + str(dist[0][4]) + '\n')
	f.close()

	# print
	print("overall reprojection error:", reproj_error)


if __name__ == "__main__":
	main()

