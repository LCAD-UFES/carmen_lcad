import sys
import os
import cv2
import numpy as np
from glob import glob

out_of_range = 999999


def check_file_name(file_name):
	# file_name format: lane_<n>.txt
	try:
		n = int(file_name[5:-4])
		
		if file_name[:5] != 'lane_' or file_name[-4:] != '.txt':
			raise ValueError
		
	except ValueError:
		n = -1
	
	return n


def file_name_compare(file_a, file_b):
	if os.path.dirname(file_a) != os.path.dirname(file_b):
		return -1 if (file_a < file_b) else 1

	fa = os.path.basename(file_a)
	fb = os.path.basename(file_b)
	# file_name format: lane_<n>.txt
	try:
		na = int(fa[5:-4])
		nb = int(fb[5:-4])
		
		if check_file_name(fa) < 0 or check_file_name(fb) < 0:
			raise ValueError
		
	except ValueError:
		return -1 if (fa < fb) else int(fa > fb)
	
	return (na - nb)


def get_image_file_name(label_file, labels_path, images_path, image_type):
	file_name = os.path.basename(label_file)[:-3] + image_type
	file_dir = os.path.dirname(label_file)
	labels_parts = labels_path.split('*')
	subdir = file_dir[len(labels_parts[0]):]
	if len(labels_parts) > 1 and len(labels_parts[1]) > 1:
		subdir = subdir[:-len(labels_parts[1]) + 1]
	images_parts = images_path.split('*')
	img_file_name = subdir.join(images_parts) + file_name
	
	return img_file_name


def get_poly_line(points, height):
	if not points:
		return []
	
	X  = [ xy[0] for xy in points ]
	Y  = [ xy[1] for xy in points ]
	poly_coeffs  = np.polyfit(Y, X, len(points) - 1)

	poly_line = [out_of_range] * height

	for y in range(height):
		x  = np.polyval(poly_coeffs, y)
		poly_line[y] = x
	
	return poly_line


def plot_poly_lines(img, poly_left, poly_right):
	for y in reversed(range(1, img.shape[0])):
		cv2.circle(img, (int(round(poly_left[y])),  y), 2, (255,0,0), -1)
		cv2.circle(img, (int(round(poly_right[y])), y), 2, (255,0,0), -1)

		if (poly_right[y - 1] < poly_left[y - 1]) or ((poly_right[y - 1] - poly_left[y - 1]) > (poly_right[y] - poly_left[y])):
			break


def point_order_by_y(pa, pb):
	# Order by y coordinate (reverse order)
	return (pb[1] - pa[1])


def plot_bboxes(img, points):
	for i, (x, y) in enumerate(sorted(points, cmp = point_order_by_y)):
		cv2.circle(img, (x, y), 5, (255,0,0), -1)
		if i > 0:
			(x1, y1) = points[i - 1]
			cv2.rectangle(img, (x, y), (x1, y1), (255,0,0), 2)


def	read_gt_file(gt_file, width, height):
	gt_points = []
	gt = open(gt_file)
	
	for line in gt:
		gt_point = line.split()
		if len(gt_point) == 2:
			try:
				x, y = [ int(data) for data in gt_point ]

				if  x >= 0 and x < width and y >= 0 and y < height:
					gt_points.append([x, y])
				else:
					print('ERROR: Invalid data (out of image limits): ' + line)
				
			except ValueError:
				print('ERROR: Invalid data format: ' + line)
				continue

		elif gt_point:
			print('ERROR: Invalid gt record format: ' + line)
	
	gt.close()

	return gt_points


def remove_outliers(points, width):
	outliers = []
	
	i = 1
	while i < len(points):
		(last_x, last_y) = points[i - 1]
		(x, y) = points[i]
		dx = x - last_x
		dy = y - last_y
		theta = np.arctan2(dy, dx)
		if i == 1:
			last_theta = theta
		
		if abs(dx) > 30 and abs(np.sin(theta)) < 0.1:
			if abs(x - width / 2) < abs(last_x - width / 2):
				outliers.append(points[i])
				del points[i]
			else:
				outliers.append(points[i - 1])
				del points[i - 1]
			continue
		
		if abs(theta - last_theta) > (np.pi / 4):
			outliers.append(points[i])
			del points[i]
			continue
		
		last_theta = theta
		i += 1
	
	return outliers


def split_points(points, width):
	points_left = []; points_right = []; outliers = []
	
	for (x, y) in points:
		if x < (width / 2):
			points_left.append([x, y])
		else:
			points_right.append([x, y])
	
	points_left = sorted(points_left, cmp = point_order_by_y)
	outliers_left = remove_outliers(points_left, width)

	points_right = sorted(points_right, cmp = point_order_by_y)
	outliers_right = remove_outliers(points_right, width)
	
	if outliers_right:
		points_left += outliers_right
		points_left  = sorted(points_left, cmp = point_order_by_y)
		outliers += remove_outliers(points_left, width)

	if outliers_left:
		points_right += outliers_left
		points_right  = sorted(points_right, cmp = point_order_by_y)
		outliers += remove_outliers(points_right, width)

	for (x, y) in outliers:
		print("ERROR: Point does not fit in any lane marking: (" + str(x) + ", " + str(y) + ")")
	
	return points_left, points_right


def main(gt_path, images_path, image_width, image_height):
	gt_file_list = glob(gt_path + '*')
	if not gt_file_list:
		print("\nERROR: No ground truth files found: " + gt_path + "\n")
		return 1

	for gt_file in sorted(gt_file_list, cmp = file_name_compare):
		if check_file_name(os.path.basename(gt_file)) < 0:
			print('ERROR: FILE NAME FORMAT: ' + gt_file)
			continue

		print(gt_file)
		img_file = get_image_file_name(gt_file, gt_path, images_path, 'png')
		if not os.path.isfile(img_file):
			print('ERROR: FILE NOT FOUND: ' + img_file)
			img = np.full((image_height, image_width, 3), (255,255,255), np.uint8)
		else:
			print(img_file)
			img = cv2.imread(img_file)
		
		# Each line of the gt file contains the coordinates of one groundtruth point
		# Line format: x y
		#   Field 0: x coordinate of the point (in pixels)
		#   Field 1: y coordinate of the point (in pixels)
		
		gt_points = read_gt_file(gt_file, image_width, image_height)
		gt_left, gt_right = split_points(gt_points, image_width)
		poly_left  = get_poly_line(gt_left,  image_height)
		poly_right = get_poly_line(gt_right, image_height)
		plot_poly_lines(img, poly_left, poly_right)
		plot_bboxes(img, gt_left)
		plot_bboxes(img, gt_right)

		while True:
			cv2.imshow('GT View', img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:      # Enter key
				break
			if key == 27:      # ESC key
				return 0
	return 0


if __name__ == "__main__":
	if len(sys.argv) == 4:
		if not sys.argv[1].endswith('/'):
			sys.argv[1] += '/'
		if not sys.argv[2].endswith('/'):
			sys.argv[2] += '/'
		
		if len(sys.argv[1].split('*')) > 2 or len(sys.argv[2].split('*')) > 2: 
			print("\nOnly one '*' allowed in pathnames\n")
		else:	
			s = sys.argv[3].split('x')
			if len(s) != 2:
				print("\nInvalid image size: wxh\n")
			elif main(sys.argv[1], sys.argv[2], int(s[0]), int(s[1])) == 0:
				sys.exit()
			
	print("\nUsage: python " + sys.argv[0] + " gt_path images_path image_size(wxh)\n")
	print("Example: python " + sys.argv[0] + ' "/lane_dataset/gt/*/labels/" "/lane_dataset/gt/*/images/" 640x480\n')
	print("Note: label lines must be in format: x y (in pixels)\n")
