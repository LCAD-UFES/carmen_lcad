import sys
import os
import cv2
import numpy as np
from glob import glob

out_of_range = 999999


def file_number(file_name):
	# file_name format: lane_<n>.txt
	try:
		n = int(file_name[5:-4])
		
		if file_name[:5] != 'lane_' or file_name[-4:] != '.txt':
			raise ValueError
		
	except ValueError:
		return -1
	
	return n


def file_name_by_number(file_a, file_b):
	if os.path.dirname(file_a) != os.path.dirname(file_b):
		return -1 if (file_a < file_b) else 1
	
	na = file_number(os.path.basename(file_a))
	nb = file_number(os.path.basename(file_b))
	
	if na < 0 or nb < 0:
		return -1 if (file_a < file_b) else int(file_a > file_b)
	
	return (na - nb)


def gt_point_order_by_y(pa, pb):
	# Points ordered by y coordinate (in reverse order)
	return int(pb[2] - pa[2])


def gt_line_order_by_x(la, lb):
	# Lines ordered by x coordinate of the first point of the line
	return int(la[0][1] - lb[0][1])


def bbox_order_by_y(bbox_a, bbox_b):
	# Order by y coordinate (in reverse order) of the bbox's center
	return int(bbox_b[1][1] - bbox_a[1][1])


def read_ground_truth_file(gt_file_name, width, height):
	print(gt_file_name)
	gt_lines = []
	
	# Each record of the gt_file contains the coordinates of one ground truth point
	# Record format: class line x y
	#   Field 0: object class
	#   Field 1: line sequence number (starting from zero)
	#   Field 2: x coordinate of the point (in pixels)
	#   Field 3: y coordinate of the point (in pixels)

	gt_file = open(gt_file_name)
	for record in gt_file:
		try:
			gt = record.split()
			if len(gt) != 4:
				raise ValueError
				
			gt_data = [ int(data) for data in gt ]
			(obj_class, line_seq, x, y) = gt_data
			if  (0 > x >= width) or (0 > y >= height):
				raise ValueError
			
			for i in range(len(gt_lines), line_seq + 1):
				gt_lines.append([])
			
			gt_lines[line_seq].append([obj_class, x, y])
		
		except ValueError:
			if gt:
				print('ERROR: Invalid grount truth record (class line x y): ' + record)
			continue
	gt_file.close()

	for i in range(len(gt_lines)):
		gt_line_sorted = sorted(gt_lines[i], cmp = gt_point_order_by_y)
		gt_lines[i] = gt_line_sorted
	
	gt_lines_sorted = sorted(gt_lines, cmp = gt_line_order_by_x)
	
	return gt_lines_sorted


def get_ground_truth_top(gt_lines, height):
	coeffs_left  = [0, 0]
	coeffs_right = [0, 0]
	y_left  = 0
	y_right = 0
	
	for gt_line in gt_lines:
		if len(gt_line) > 0:
			p1 = gt_line[-1]
			(obj_class, x1, y1) = p1
			if len(gt_line) == 1:
				coeffs = [0, x1]
				y1 = 0
			else:
				p0 = gt_line[-2]
				(obj_class, x0, y0) = p0
				coeffs = np.polyfit([y0, y1], [x0, x1], 1)
			if coeffs[0] < coeffs_left[0]:
				coeffs_left = coeffs[:]
				y_left = y1
			if coeffs[0] > coeffs_right[0]:
				coeffs_right = coeffs[:]
				y_right = y1
	
	if (coeffs_left[0] - coeffs_right[0]) != 0:
		y = - (coeffs_left[1] - coeffs_right[1]) / (coeffs_left[0] - coeffs_right[0])
		if y < y_left and y < y_right:
			return y

	return 0


def gt_interpolate(y_predicted, gt_line, gt_top):
	(obj_class, x0, y0) = gt_line[0]
	if y_predicted < gt_top:
		return (obj_class, out_of_range)
	if len(gt_line) == 1:
		return (obj_class, x0)

	for j in range(1, len(gt_line)):
		(obj_class, x1, y1) = gt_line[j]
		if (y1 <= y_predicted) or (j == (len(gt_line) - 1)):
			break
		(obj_class, x0, y0) = gt_line[j]
		
	coeffs = np.polyfit([y0, y1], [x0, x1], 1)
	gt_x = np.polyval(coeffs, y_predicted)

	return (obj_class, gt_x)


# def get_ground_truth_continuum_lines(gt_lines, height):
# 	gt_continuum_lines = [[]] * len(gt_lines)
# 	
# 	for i in range(len(gt_lines)):
# 		if len(gt_lines[i]) > 0:
# 			p0 = gt_lines[i][0]
# 			(obj_class, x0, y0) = p0
# 			if len(gt_lines[i]) == 1:
# 				gt_continuum_lines[i] = [[obj_class, x0]] * height
# 			else:
# 				gt_continuum_lines[i] = [[obj_class, out_of_range]] * height
# 				for j in range(1, len(gt_lines[i])):
# 					p1 = gt_lines[i][j]
# 					(obj_class, x1, y1) = p1
# 					coeffs = np.polyfit([y0, y1], [x0, x1], 1)
# 					p0 = p1
# 					if j == 1:
# 						y0 = height
# 					if j == len(gt_lines[i]) - 1:
# 						y1 = 0
# 					
# 					for y in range(int(y1), int(y0)):
# 						x = int(np.polyval(coeffs, y))
# 						gt_continuum_lines[i][y] = [obj_class, x]
# 	
# 	if len(gt_continuum_lines) > 1:					
# 		for y in reversed(range(1, height)):
# 			for i in range(1, len(gt_continuum_lines)):
# 				(p0_left, p0_right) = (gt_continuum_lines[i - 1][y],     gt_continuum_lines[i][y])
# 				(p1_left, p1_right) = (gt_continuum_lines[i - 1][y - 1], gt_continuum_lines[i][y - 1])
# 				(obj_class_left,  x0_left)  = p0_left
# 				(obj_class_right, x0_right) = p0_right
# 				(obj_class_left,  x1_left)  = p1_left
# 				(obj_class_right, x1_right) = p1_right
# 				if (out_of_range in (x0_left, x0_right, x1_left, x1_right)) or \
# 				   (x1_left > x1_right) or ((x1_left - x1_right) > (x0_left - x0_right)):
# 					gt_continuum_lines[i - 1][y - 1] = [obj_class_left,  out_of_range]
# 					gt_continuum_lines[i][y - 1]     = [obj_class_right, out_of_range]
# 
# 	return gt_continuum_lines


# def get_ground_truth_poly_lines(gt_points, height):
# 	X_left  = [ xy[0] for xy in gt_points[0] ]
# 	Y_left  = [ xy[1] for xy in gt_points[0] ]
# 	poly_coeffs_left  = np.polyfit(Y_left, X_left, 3)
# 
# 	X_right = [ xy[0] for xy in gt_points[1] ]
# 	Y_right = [ xy[1] for xy in gt_points[1] ]
# 	poly_coeffs_right = np.polyfit(Y_right, X_right, 3)
# 
# 	gt_poly_lines = [ [out_of_range] * height, [out_of_range] * height ]
# 
# 	for y in reversed(range(height)):
# 		x_left  = int(np.polyval(poly_coeffs_left,  y))
# 		x_right = int(np.polyval(poly_coeffs_right, y))
# 		if abs(x_left - x_right) < 5:
# 			break
# 		gt_poly_lines[0][y] = x_left
# 		gt_poly_lines[1][y] = x_right
# 	
# 	return gt_poly_lines


def remove_bbox_outliers(bboxes, width):
	outliers = []
	
	i = 1
	while i < len(bboxes):
		(last_obj_class, (last_x, last_y), last_diag_1, last_diag_2) = bboxes[i - 1]
		(obj_class, (x, y), diag_1, diag_2) = bboxes[i]
		dx = x - last_x
		dy = y - last_y
		theta = np.arctan2(dy, dx)
		if i == 1:
			last_theta = theta
		
		if abs(dx) > 30 and abs(np.sin(theta)) < 0.1:
			if abs(x - width / 2) < abs(last_x - width / 2):
				outliers.append(bboxes[i])
				del bboxes[i]
			else:
				outliers.append(bboxes[i - 1])
				del bboxes[i - 1]
			continue
		
		if abs(theta - last_theta) > (np.pi / 4):
			outliers.append(bboxes[i])
			del bboxes[i]
			continue
		
		last_theta = theta
		i += 1
	
	return outliers


def bboxes_clustering(bboxes, width):
	bboxes_clusters = []; bboxes_left = []; bboxes_right = []; outliers = []
	
	for bbox in bboxes:
		(obj_class, (x, y), diagonal_1, diagonal_2) = bbox
		if x < (width / 2):
			bboxes_left.append(bbox)
		else:
			bboxes_right.append(bbox)
	
	bboxes_left = sorted(bboxes_left, cmp = bbox_order_by_y)
	outliers_left = remove_bbox_outliers(bboxes_left, width)

	bboxes_right = sorted(bboxes_right, cmp = bbox_order_by_y)
	outliers_right = remove_bbox_outliers(bboxes_right, width)
	
	if outliers_right:
		bboxes_left += outliers_right
		bboxes_left = sorted(bboxes_left, cmp = bbox_order_by_y)
		outliers += remove_bbox_outliers(bboxes_left, width)

	if outliers_left:
		bboxes_right += outliers_left
		bboxes_right = sorted(bboxes_right, cmp = bbox_order_by_y)
		outliers += remove_bbox_outliers(bboxes_right, width)

	for bbox in outliers:
		(obj_class, (x, y), diagonal_1, diagonal_2) = bbox
		print("ERROR: Bounding box does not fit in any line: (x=" + str(x) + ", y=" + str(y) + ")")

	for bboxes_cluster in (bboxes_left, bboxes_right):
		(obj_class_first, (x_first, y_first), diag_1_first, diag_2_first) = bboxes_cluster[ 0]
		(obj_class_last,  (x_last,  y_last ), diag_1_last,  diag_2_last ) = bboxes_cluster[-1]
		diag = 1 if (x_first < x_last) else 2
		cluster = []
		for bbox in bboxes_cluster:
			(obj_class, center, diagonal_1, diagonal_2) = bbox
			diagonal = diagonal_1 if (diag == 1) else diagonal_2
			cluster.append([ obj_class, diagonal ])
		bboxes_clusters.append(cluster)
	
	return bboxes_clusters


def check_bbox_limits(x_left, x_right, y_top, y_bottom, width, height):
	x_tolerance = 0.1 * width
	y_tolerance = 0.1 * height
	min_x = - x_tolerance
	max_x = width + x_tolerance
	min_y = - y_tolerance
	max_y = height + y_tolerance
			
	if  not ((min_x <= x_left < max_x) and (min_x <= x_right < max_x) and (min_y <= y_top < max_y) and (min_y <= y_bottom < max_y)):
		raise ValueError
	

def read_predictions_file(predictions_dir, file_name, width, height):
	predictions_file_name = predictions_dir + os.path.basename(file_name)
	print(predictions_file_name)
	predictions_list = []

	# Each record of the file contains the coordinates of one predicted bounding box
	# Record format: class x y w h
	#   Field 0: object class
	#   Field 1: x coordinate of the bounding box's center (in a fraction of image_width)
	#   Field 2: y coordinate of the bounding box's center (in a fraction of image_height)
	#   Field 3: the bounding box's width  (in a fraction of image_width)
	#   Field 4: the bounding box's height (in a fraction of image_height)
	
	predictions_file = open(predictions_file_name)
	for record in predictions_file:
		try:
			bbox = record.split()
			if len(bbox) != 5:
				raise ValueError

			obj_class = int(bbox[0])				
			(x, y, w, h) = [ float(data) for data in bbox[1:] ]
			x_left   = (x - (w / 2)) * width
			x_right  = (x + (w / 2)) * width
			y_top    = (y - (h / 2)) * height
			y_bottom = (y + (h / 2)) * height
			check_bbox_limits(x_left, x_right, y_top, y_bottom, width, height)

			center = [x * width, y * height]
			diagonal_1 = [ [x_left, y_bottom], [x_right, y_top] ]
			diagonal_2 = [ [x_left, y_top],    [x_right, y_bottom] ]
			predictions_list.append([ obj_class, center, diagonal_1, diagonal_2 ])
		
		except ValueError:
			if bbox:
				print("ERROR: Invalid predictions file record: " + record)
			continue
	predictions_file.close()
	
	return predictions_list


def plot_gt_lines(img, gt_lines, gt_top):
	for gt_line in gt_lines:
		if len(gt_line) > 0:
			(obj_class, x0, y0) = gt_line[0]
			if len(gt_line) == 1:
				cv2.line(img, (x0, len(img) - 1), (x0, gt_top), (255,0,0), 2)

			for j in range(1, len(gt_line)):
				(obj_class, x1, y1) = gt_line[j]
				coeffs = np.polyfit([y0, y1], [x0, x1], 1)
				if j == 1:
					y0 = len(img) - 1
					x0 = np.polyval(coeffs, y0)
				if j == (len(gt_line) - 1):
					y1 = gt_top
					x1 = np.polyval(coeffs, y1)
				cv2.line(img, (int(x0), int(y0)), (int(x1), int(y1)), (255,0,0), 2)
				(obj_class, x0, y0) = gt_line[j]

			for (obj_class, x, y) in gt_line:
				(x, y) = (int(x), int(y))
				cv2.circle(img, (x, y), 5, (255,0,0), -1)
				cv2.putText(img, str(obj_class), ((x + 6), (y + 9)), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 1)


def plot_bboxes(img, predictions, gt_lines, gt_top, max_error):
	for bbox_list in predictions:
		for bbox in bbox_list:
			(obj_class, diagonal) = bbox
			((xa, ya), (xb, yb)) = diagonal
			((xa, ya), (xb, yb)) = ((int(xa), int(ya)), (int(xb), int(yb)))
			cv2.rectangle(img, (xa, ya), (xb, yb), (0,0,255), 2)
			cv2.putText(img, str(obj_class), ((min(xa, xb) + 3), (min(ya, yb) - 6)), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1)
			(compute_a, compute_b) = compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error)
			(class_a, bbox_a, err_a) = compute_a
			(class_b, bbox_b, err_b) = compute_b
			if bbox_a:
				cv2.circle(img, (xa, ya), 5, (0,0,255), -1)
			else:
				cv2.circle(img, (xa, ya), 5, (255,255,0), -1)
			if bbox_b:
				cv2.circle(img, (xb, yb), 5, (0,0,255), -1)
			else:
				cv2.circle(img, (xb, yb), 5, (255,255,0), -1)
	

def plot_image(predictions, gt_lines, gt_top, max_error, gt_file_name, image_width, image_height, show_images, images_dir, save_results, save_dir):
	img_file = images_dir + os.path.basename(gt_file_name)[:-3] + 'png'
	if os.path.isfile(img_file):
		img = cv2.imread(img_file)
	else:
		print('ERROR: FILE NOT FOUND: ' + img_file)
		img = np.full((image_height, image_width, 3), (255,255,255), np.uint8)

	plot_gt_lines(img, gt_lines, gt_top)
	plot_bboxes(img, predictions, gt_lines, gt_top, max_error)

	if save_results:
		cv2.imwrite(save_dir + os.path.basename(img_file), img)
	
	if show_images:
		window_name = 'Lane Detector - ' + os.path.basename(gt_file_name)[:-4]
		cv2.namedWindow(window_name)
		cv2.moveWindow(window_name, 10, 10)
		while True:
			cv2.imshow(window_name, img)
			key = cv2.waitKey(0) & 0xff
			if key == 10:      # Enter key
				break
			if key == 27:      # ESC key
				sys.exit(0)
		cv2.destroyWindow(window_name)


def compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error):
	(class_a, bbox_a, err_a) = (False, False, out_of_range)
	(class_b, bbox_b, err_b) = (False, False, out_of_range)
	((xa, ya), (xb, yb)) = diagonal
	for gt_line in gt_lines:
		if len(gt_line) > 0:
			(gt_class_a, gt_xa) = gt_interpolate(ya, gt_line, gt_top)
			(gt_class_b, gt_xb) = gt_interpolate(yb, gt_line, gt_top)
			gt_err_a = abs(gt_xa - xa) if (gt_xa != out_of_range) else out_of_range
			gt_err_b = abs(gt_xb - xb) if (gt_xb != out_of_range) else out_of_range
	
			if (gt_err_a + gt_err_b) <= (err_a + err_b):
				err_a = gt_err_a
				err_b = gt_err_b
				bbox_a |= (err_a <= max_error)
				bbox_b |= (err_b <= max_error)
				class_a |= (bbox_a and (obj_class == gt_class_a))
				class_b |= (bbox_b and (obj_class == gt_class_b))

	compute_a = (class_a, bbox_a, err_a)
	compute_b = (class_b, bbox_b, err_b)
	return (compute_a, compute_b)


def compute_error(predictions, gt_lines, gt_top, max_error):
	(class_error, class_true_pos, class_false_pos) = (0, 0, 0)
	(bbox_error,  bbox_true_pos,  bbox_false_pos)  = (0, 0, 0)

	for prediction_bboxes in predictions:
		for bbox in prediction_bboxes:
			(obj_class, diagonal) = bbox
			(compute_a, compute_b) = compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error)

			for (class_p, bbox_p, err_p) in (compute_a, compute_b):
				if class_p:
					class_error += err_p
					class_true_pos += 1
				else:
					class_false_pos += 1

				if bbox_p:
					bbox_error += err_p
					bbox_true_pos += 1
				else:
					bbox_false_pos += 1
	
	expected_bbox_points = sum([ (len(gt_line) - 1) for gt_line in gt_lines ]) * 2
	class_false_neg = expected_bbox_points - class_true_pos - class_false_pos
	bbox_false_neg  = expected_bbox_points - bbox_true_pos  - bbox_false_pos
	result_class = (class_error, class_true_pos, class_false_pos, class_false_neg)
	result_bbox  = (bbox_error,  bbox_true_pos,  bbox_false_pos,  bbox_false_neg)

	return (result_class, result_bbox)


def recursive_glob(dir_name, file_pattern):
	file_list = []
	if os.path.isdir(dir_name):
		file_list = [file_name for file_name in glob(dir_name + file_pattern) if os.path.isfile(file_name)]
		for item in os.listdir(dir_name):
			file_list += recursive_glob(dir_name + item + '/', file_pattern)
	
	return file_list


def main(ground_truth_dir, predictions_dir, image_width, image_height, max_error, show_images, images_dir, save_results, save_dir):
	file_pattern = 'lane_*.txt'
	gt_file_list = recursive_glob(ground_truth_dir, file_pattern)
	if not gt_file_list:
		raise Exception(3, "ERROR: No ground truth " + file_pattern + " files found in: " + ground_truth_dir)
	
	(class_accum_error, class_accum_true_pos, class_accum_false_pos, class_accum_false_neg) = (0, 0, 0, 0)
	(bbox_accum_error,  bbox_accum_true_pos,  bbox_accum_false_pos,  bbox_accum_false_neg)  = (0, 0, 0, 0)

	for gt_file_name in sorted(gt_file_list, cmp = file_name_by_number):
		gt_lines = read_ground_truth_file(gt_file_name, image_width, image_height)
		gt_top = get_ground_truth_top(gt_lines, image_height)
		predictions_list = read_predictions_file(predictions_dir, gt_file_name, image_width, image_height)
		predictions = bboxes_clustering(predictions_list, image_width)
		
		(result_class, result_bbox) = compute_error(predictions, gt_lines, gt_top, max_error)
		(class_error, class_true_pos, class_false_pos, class_false_neg) = result_class
		(bbox_error,  bbox_true_pos,  bbox_false_pos,  bbox_false_neg)  = result_bbox
		class_accum_error += class_error
		class_accum_true_pos += class_true_pos
		class_accum_false_pos += class_false_pos
		class_accum_false_neg += class_false_neg
		bbox_accum_error += bbox_error
		bbox_accum_true_pos += bbox_true_pos
		bbox_accum_false_pos += bbox_false_pos
		bbox_accum_false_neg += bbox_false_neg

		if show_images or save_results:
			plot_image(predictions, gt_lines, gt_top, max_error, gt_file_name, image_width, image_height, show_images, images_dir, save_results, save_dir)
	
	result_class_accum = (class_accum_error, class_accum_true_pos, class_accum_false_pos, class_accum_false_neg)
	result_bbox_accum  = (bbox_accum_error,  bbox_accum_true_pos,  bbox_accum_false_pos,  bbox_accum_false_neg)
	return (result_class_accum, result_bbox_accum)


def print_stats(results):
	(result_class, result_bbox) = results
	
	for (result_type, error, true_pos, false_pos, false_neg) in (['Class'] + list(result_class), ['Bounding Box'] + list(result_bbox)):
		print('\n' + result_type + ' statistics:')
		print('   True positives:  (%6d)' % true_pos  + ' predicted points')
		print('   False positives: (%6d)' % false_pos + ' predicted points')
		print('   False negatives: (%6d)' % false_neg + ' unpredicted points')
		print('   Average Error:   %6.2f' % (1.0 * error  / (true_pos + false_pos)) + ' pixels')
		print('   Precision: %6.2f%%' % (100.0 * true_pos / (true_pos + false_pos)))
		print('   Recall:    %6.2f%%' % (100.0 * true_pos / (true_pos + false_neg)))
		print('   Accuracy:  %6.2f%%' % (100.0 * true_pos / (true_pos + false_pos + false_neg)) + '\n')
	

def read_parameters(argv):
	(show_images, images_dir, save_results, save_dir) = (False, './', False, './')

	i = 5
	while i < len(argv):
		if argv[i] == '-show':
			show_images = True

		elif argv[i] == '-img':
			if (i + 1) < len(argv) and argv[i + 1][0] != '-':
				i += 1
				images_dir = argv[i] if argv[i].endswith('/') else argv[i] + '/'

		elif argv[i] == '-save':
			save_results = True
			if (i + 1) < len(argv) and argv[i + 1][0] != '-':
				i += 1
				save_dir = argv[i] if argv[i].endswith('/') else argv[i] + '/'
				
		else:
			raise Exception(2, "ERROR: Invalid command line argument [%d]: " % i + argv[i])
		
		i += 1

	return (show_images, images_dir, save_results, save_dir)


def get_image_size(size):
	try:
		dims = size.split('x')
		if len(dims) != 2:
			raise ValueError

		image_width  = int(dims[0]) 
		image_height = int(dims[1])
	
	except ValueError:
		raise Exception(1, "ERROR: Invalid image size format wxh: " + size)
	
	return (image_width, image_height)


if __name__ == "__main__":
	try:
		if not 5 <= len(sys.argv) <= 10:
			raise Exception(0, 'usage')

		ground_truth_dir = sys.argv[1] if sys.argv[1].endswith('/') else sys.argv[1] + '/'
		predictions_dir  = sys.argv[2] if sys.argv[2].endswith('/') else sys.argv[2] + '/'
		(image_width, image_height) = get_image_size(sys.argv[3])
		max_error = float(sys.argv[4])
		(show_images, images_dir, save_results, save_dir) = read_parameters(sys.argv)
		
		results = main(ground_truth_dir, predictions_dir, image_width, image_height, max_error, show_images, images_dir, save_results, save_dir)
		print_stats(results)
		sys.exit(0)
	
	except Exception as error:
		if error.args[-1] != 'usage':
			print('\n' + error.args[-1] + '\n')
		
		print("\nUsage: python " + sys.argv[0] + "  <ground_truth_dir>  <predictions_dir>  <image_size>[wxh]  <max_error>[pixels]"
			"  -show  -img  <images_dir>  -save  <save_dir>\n")
		print("Example: python " + sys.argv[0] + "  /lane_dataset/groundtruth/labels  /lane_dataset/yolo/labels  640x480  30"
			"  -show  -img  /lane_dataset/groundtruth/images  -save /results/\n")
		print("Note: Groundtruth label records must be in the format: class line x y (in pixels)")
		print("      Predictions label records must be in the format: class x y w h (in fractions)\n")

		if len(error.args) == 1:
			raise
		sys.exit(1)
