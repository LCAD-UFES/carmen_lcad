import sys
import os
import cv2
import numpy as np


def plot_poly_curve(img, points):
	X = [ xy[0] for xy in points ]
	Y = [ xy[1] for xy in points ]
	poly_coeffs_left  = np.polyfit(Y[:4], X[:4], 3)
	poly_coeffs_right = np.polyfit(Y[4:], X[4:], 3)

	for y in reversed(range(len(img))):
		x_left  = int(np.polyval(poly_coeffs_left,  y))
		x_right = int(np.polyval(poly_coeffs_right, y))
		if abs(x_left - x_right) < 5:
			break
		cv2.circle(img, (x_left,  y), 2, (0,0,255), -1)
		cv2.circle(img, (x_right, y), 2, (0,0,255), -1)


def file_name_compare(a, b):
	na = int(a[5:-4])
	nb = int(b[5:-4])
	
	return (na - nb)


def main(gt_path, images_path):
	gt_file_list = [ f for f in os.listdir(gt_path) ]
	gt_file_list = sorted(gt_file_list, cmp = file_name_compare)
	
	for gt_file in gt_file_list:
		print(gt_file)
		img_file = images_path + gt_file.replace('.txt', '.png')
		if not os.path.isfile(img_file):
			img = np.full((480, 640, 3), (255,255,255), np.uint8)
		else:
			img = cv2.imread(img_file)

		gt = open(gt_path + gt_file, "r")
		points = []
		for line in gt:
			if len(line) < 3:
				continue
			x, y = [ int(data) for data in line.split() ]
			points.append([x, y])
		gt.close()

		plot_poly_curve(img, points)

		for i, (x, y) in enumerate(points):
			cv2.circle(img, (x, y), 5, (255,0,0), -1)
			if (i % 4) != 0:
				(x1, y1) = points[i - 1]
				cv2.rectangle(img, (x, y), (x1, y1), (255,0,0), 2)
				cv2.putText(img, str(i), ((x + x1) / 2, (y + y1) / 2), cv2.FONT_HERSHEY_PLAIN, 1, [0,255,0], 1);

		while (1):
			cv2.imshow('GT View', img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:      # Enter key
				break
			elif key == 27:    # ESC key
				return


if __name__ == "__main__":
	if len(sys.argv) != 3:
		print("\nUsage: python " + sys.argv[0] + " gt_path images_path\n")
		exit(1)

	if not sys.argv[1].endswith('/'):
		sys.argv[1] += '/'
	if not sys.argv[2].endswith('/'):
		sys.argv[2] += '/'
		
	main(sys.argv[1], sys.argv[2])
