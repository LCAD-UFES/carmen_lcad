import sys
import os
import cv2
import numpy as np

out_of_range = 999999

def file_name_compare(fa, fb):
	# file_name format: lane_<n>.txt
	try:
		na = int(fa[5:-4])
		nb = int(fb[5:-4])
		
		if fa[:5] != 'lane_' or fb[:5] != 'lane_':
			raise ValueError
		
	except ValueError:
		return -1 if (fa < fb) else int(fa > fb)
	
	return (na - nb)


def read_groud_truth_points(gt_dir, gt_file_name):
	print(gt_dir + gt_file_name)
	gt_file = open(gt_dir + gt_file_name, "r")
	
	gt_points_list = []
	
	# Each line of the gt_file contains the coordinates of one groundtruth point
	# Line format: x y
	#   Field 0: x coordinate of the point (in pixels)
	#   Field 1: y coordinate of the point (in pixels)
	
	for line in gt_file:
		gt = line.split()
		if len(gt) == 2:
			x, y = [ int(data) for data in gt ]
			gt_points_list.append([x, y])
		elif gt:
			print('Invalid gt file record: ' + line)
	
	if len(gt_points_list) == 8:	
		gt_points_list = [ gt_points_list[:4], gt_points_list[4:] ]  # Left and right lane gt points
	else:
		print(str(len(gt_points_list)) + ' gt points found, but 8 were expected')
		gt_points_list = [ [], [] ]
	
	return gt_points_list


def get_ground_truth_poly_lines(gt_points, height):
	X_left  = [ xy[0] for xy in gt_points[0] ]
	Y_left  = [ xy[1] for xy in gt_points[0] ]
	poly_coeffs_left  = np.polyfit(Y_left, X_left, 3)

	X_right = [ xy[0] for xy in gt_points[1] ]
	Y_right = [ xy[1] for xy in gt_points[1] ]
	poly_coeffs_right = np.polyfit(Y_right, X_right, 3)

	gt_poly_lines = [ [out_of_range] * height, [out_of_range] * height ]

	for y in reversed(range(height)):
		x_left  = int(np.polyval(poly_coeffs_left,  y))
		x_right = int(np.polyval(poly_coeffs_right, y))
		if abs(x_left - x_right) < 5:
			break
		gt_poly_lines[0][y] = x_left
		gt_poly_lines[1][y] = x_right
	
	return gt_poly_lines


def read_and_convert_4_points_coordinates(predictions_dir, file_name, image_width, image_height):
	print(predictions_dir + file_name)
	predictions_list_file = open(predictions_dir + file_name, "r")
	
	predictions_list = []

	# Each line of the file contains the coordinates of one predicted bounding box
	# Line format: class x y w h
	#   Field 0: object class
	#   Field 1: x coordinate of the bounding box's center (in a fraction of image_width)
	#   Field 2: y coordinate of the bounding box's center (in a fraction of image_height)
	#   Field 3: the bounding box's width  (in a fraction of image_width)
	#   Field 4: the bounding box's height (in a fraction of image_height)
	
	for line in predictions_list_file:
		bbox = line.split()
		if len(bbox) == 5:
			x, y, w, h = [ float(data) for data in bbox[1:] ]
			x_left   = int((x - (w / 2)) * image_width)
			x_right  = int((x + (w / 2)) * image_width)
			y_top    = int((y - (h / 2)) * image_height)
			y_bottom = int((y + (h / 2)) * image_height)
			predictions_list.append([ [ [x_left, y_bottom], [x_right, y_top] ], [ [x_left, y_top], [x_right, y_bottom] ] ])
		elif bbox:
			print('Invalid prediction file record: ' + line)
		
	return predictions_list


def find_image_path():
	for i, param in enumerate(sys.argv):
		if param == '-show' and len(sys.argv) > (i + 1):
			return sys.argv[i + 1]
	return ""


def plot_poly_lines(img, poly_lines):
	for i in range(len(poly_lines)):
		for y, x in enumerate(poly_lines[i]):
			if x != out_of_range:
				cv2.circle(img, (x, y), 2, (255,0,0), -1)


def plot_gt_points(img, gt_points):
	for i in range(len(gt_points)):
		for (x, y) in gt_points[i]:
			cv2.circle(img, (x, y), 5, (255,0,0), -1)


def plot_predictions(img, predictions_points, gt_poly_lines):
	for bbox in predictions_points:
		( diagonal_1, diagonal_2 ) = bbox
		err_1 = compute_diagonal(gt_poly_lines, diagonal_1)
		err_2 = compute_diagonal(gt_poly_lines, diagonal_2)

		if err_1 == out_of_range and err_2 == out_of_range:
			cv2.rectangle(img, bbox[0], (255,255,0), 2)
		elif err_1 < err_2:
			((xa, ya), (xb, yb)) = diagonal_1
		else:
			((xa, ya), (xb, yb)) = diagonal_2
			
		cv2.rectangle(img, (xa, ya), (xb, yb), (0,0,255), 2)
		cv2.circle(img, (xa, ya), 5, (0,0,255), -1)
		cv2.circle(img, (xb, yb), 5, (0,0,255), -1)


def show_image(gt_points, gt_poly_lines, predictions_points, gt_file_name, images_path, image_width, image_height):
	img_file = images_path + gt_file_name.replace('.txt', '.png')
	if not os.path.isfile(img_file):
		img = np.full((image_height, image_width, 3), (255,255,255), np.uint8)
	else:
		img = cv2.imread(img_file)

	plot_poly_lines(img, gt_poly_lines)
	
	plot_gt_points(img, gt_points)
	
	plot_predictions(img, predictions_points, gt_poly_lines)
	
	while (1):
		cv2.imshow('Lane Detector Compute Error', img)
		key = cv2.waitKey(0) & 0xff
		
		if key == 10:      # Enter key
			break
		elif key == 27:    # ESC key
			sys.exit()


def compute_diagonal(gt_poly_lines, diagonal):
	((x1, y1), (x2, y2)) = diagonal

	if  gt_poly_lines[0][y1] == out_of_range or gt_poly_lines[0][y2] == out_of_range or \
		gt_poly_lines[1][y1] == out_of_range or gt_poly_lines[1][y2] == out_of_range:
		return out_of_range

	err_x1_left  = abs(x1 - gt_poly_lines[0][y1])
	err_x2_left  = abs(x2 - gt_poly_lines[0][y2])
	err_x1_right = abs(x1 - gt_poly_lines[1][y1])
	err_x2_right = abs(x2 - gt_poly_lines[1][y2])
	err = float(min((err_x1_left + err_x2_left), (err_x1_right + err_x2_right))) / 2
	
	return err


def compute_error(gt_points, gt_poly_lines, predictions_points):
	error = 0; true_pos = 0; false_pos = 0

	for bbox in predictions_points:
		( diagonal_1, diagonal_2 ) = bbox
		err_1 = compute_diagonal(gt_poly_lines, diagonal_1)
		err_2 = compute_diagonal(gt_poly_lines, diagonal_2)
		err = min(err_1, err_2)
		if err != out_of_range:
			error += err
			true_pos += 1
		else:
			false_pos += 1
	
	false_neg = len(gt_points[0]) + len(gt_points[1]) - 2 - true_pos - false_pos

	return error, true_pos, false_pos, false_neg


if __name__ == "__main__":
	if len(sys.argv) < 5 or len(sys.argv) > 7:
		print("\nUsage: python " + sys.argv[0] + " <ground_truth_dir> <predictions_dir> <image_width> <image_height> -show <images_dir> (optional)\n")
	else:
		if not sys.argv[1].endswith('/'):
			sys.argv[1] += '/'
		if not sys.argv[2].endswith('/'):
			sys.argv[2] += '/'
			
		image_width  = int(sys.argv[3])
		image_height = int(sys.argv[4])
		
		images_path = find_image_path()
		if images_path and not images_path.endswith('/'):
			images_path += '/'
		
		gt_files_list = [file_name for file_name in os.listdir(sys.argv[1])]
		gt_files_list = sorted(gt_files_list, cmp = file_name_compare)
		
		accum_error = 0; accum_true_pos = 0; accum_false_pos = 0; accum_false_neg = 0

		for gt_file_name in gt_files_list:
			if not gt_file_name.endswith('.txt'):
				continue
				
			gt_points = read_groud_truth_points(sys.argv[1], gt_file_name)
			gt_poly_lines = get_ground_truth_poly_lines(gt_points, image_height)
			
			predictions_points = read_and_convert_4_points_coordinates(sys.argv[2], gt_file_name, image_width, image_height)
			
			(error, true_pos, false_pos, false_neg) = compute_error(gt_points, gt_poly_lines, predictions_points)
			
			accum_error += error
			accum_true_pos += true_pos
			accum_false_pos += false_pos
			accum_false_neg += false_neg
			
			if len(sys.argv) >= 6 and sys.argv[5] == '-show':
				show_image(gt_points, gt_poly_lines, predictions_points, gt_file_name, images_path, image_width, image_height)
			
		print('\nTrue positives:  %6d' % accum_true_pos + ' bounding boxes in range')
		print('False positives: %6d' % accum_false_pos + ' bounding boxes out of range')
		print('False negatives: %6d' % accum_false_neg + ' bounding boxes missed')
		print('Average Error: %.2f' % (float(accum_error)/(accum_true_pos + accum_false_pos)) + ' pixels per predicted point\n')
		print('Precision: %.6f' % (float(accum_true_pos)/(accum_true_pos + accum_false_pos)))
		print('Recall:    %.6f' % (float(accum_true_pos)/(accum_true_pos + accum_false_neg)))
