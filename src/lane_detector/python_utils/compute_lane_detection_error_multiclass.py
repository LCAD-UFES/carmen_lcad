import sys
import os
import cv2
import numpy as np
from glob import glob

out_of_range = 999999


class statistics:
	def __init__(self, error = 0.0, true_pos = 0, true_neg = 0, false_pos = 0, false_neg = 0):
		self.error = error
		self.true_pos = true_pos
		self.true_neg = true_neg
		self.false_pos = false_pos
		self.false_neg = false_neg

	def total(self):
		return (self.true_pos + self.true_neg + self.false_pos + self.false_neg)
	
	def avg_error(self):
		return (float(self.error) / self.true_pos) if (self.true_pos > 0) else None
	
	def precision(self):
		positives = (self.true_pos + self.false_pos)
		return (float(self.true_pos) / positives) if (positives > 0) else None
	
	def recall(self):
		gt_positives = (self.true_pos + self.false_neg)		
		return (float(self.true_pos) / gt_positives) if (gt_positives > 0) else None
	
	def accuracy(self):
		return (float(self.true_pos + self.true_neg) / self.total()) if (self.total() > 0) else None
	
	def __add__(self, other):
		return statistics((self.error + other.error), (self.true_pos + other.true_pos), (self.true_neg + other.true_neg), 
			(self.false_pos + other.false_pos), (self.false_neg + other.false_neg))


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


def point_order_by_y(pa, pb):
	# Points ordered by y coordinate (in reverse order)
	return int(pb[2] - pa[2])


def line_order_by_x(la, lb):
	# Lines ordered by x coordinate of the first point of the line
	return int(la[0][1] - lb[0][1])


def bbox_order_by_y(bbox_a, bbox_b):
	# Order by y coordinate (in reverse order) of the bbox's center
	return int(bbox_b[1][1] - bbox_a[1][1])


def check_point_limits(x, y, width, height):
	x_tolerance = 0.1 * width
	y_tolerance = 0.1 * height
	min_x = - x_tolerance
	max_x = width + x_tolerance
	min_y = - y_tolerance
	max_y = height + y_tolerance
			
	if  not ((min_x <= x < max_x) and (min_y <= y < max_y)):
		raise ValueError


def read_points_file(file_name, width, height):
	print(file_name)
	lines = []
	
	# Each record of the file contains the coordinates of one point
	# Record format: class line x y
	#   Field 0: object class
	#   Field 1: line sequence number (starting from zero)
	#   Field 2: x coordinate of the point (in pixels)
	#   Field 3: y coordinate of the point (in pixels)

	p_file = open(file_name)
	for record in p_file:
		try:
			p = record.split()
			if len(p) != 4:
				raise ValueError
				
			p_data = [ int(data) for data in p ]
			(obj_class, line_seq, x, y) = p_data
			check_point_limits(x, y, width, height)
			
			for i in range(len(lines), line_seq + 1):
				lines.append([])
			
			lines[line_seq].append([obj_class, x, y])
		
		except ValueError:
			if p:
				print('ERROR: Invalid point record (class line x y): ' + record)
			continue
	p_file.close()

	for i in range(len(lines)):
		line_sorted = sorted(lines[i], cmp = point_order_by_y)
		lines[i] = line_sorted
	
	lines_sorted = sorted(lines, cmp = line_order_by_x)
	
	return lines_sorted


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
		if len(bboxes_cluster) > 0:
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
	

def read_bboxes_file(file_name, width, height):
	print(file_name)
	bbox_list = []

	# Each record of the file contains the coordinates of one bounding box
	# Record format: class x y w h
	#   Field 0: object class
	#   Field 1: x coordinate of the bounding box's center (in a fraction of image_width)
	#   Field 2: y coordinate of the bounding box's center (in a fraction of image_height)
	#   Field 3: the bounding box's width  (in a fraction of image_width)
	#   Field 4: the bounding box's height (in a fraction of image_height)
	
	bbox_file = open(file_name)
	for record in bbox_file:
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
			bbox_list.append([ obj_class, center, diagonal_1, diagonal_2 ])
		
		except ValueError:
			if bbox:
				print("ERROR: Invalid bounding box file record (class x y w h): " + record)
			continue
	bbox_file.close()
	
	return bbox_list


def plot_points(img, gt_lines, gt_top, show_lines, color = (255,0,0)):
	line_color  = [color[0], (255 - color[1]) // 2, color[2]]

	for gt_line in gt_lines:
		if len(gt_line) > 0:
			if show_lines:
				(obj_class, x0, y0) = gt_line[0]
				if len(gt_line) == 1:
					cv2.line(img, (x0, len(img) - 1), (x0, gt_top), line_color, 2)
	
				for j in range(1, len(gt_line)):
					(obj_class, x1, y1) = gt_line[j]
					coeffs = np.polyfit([y0, y1], [x0, x1], 1)
					if j == 1:
						y0 = len(img) - 1
						x0 = np.polyval(coeffs, y0)
					if j == (len(gt_line) - 1):
						y1 = gt_top
						x1 = np.polyval(coeffs, y1)
					cv2.line(img, (int(x0), int(y0)), (int(x1), int(y1)), line_color, 2)
					(obj_class, x0, y0) = gt_line[j]

			for (obj_class, x, y) in gt_line:
					(x, y) = (int(x), int(y))
					cv2.circle(img, (x, y), 5, color, -1)
					cv2.putText(img, str(obj_class), ((x + 6), (y + 9)), cv2.FONT_HERSHEY_PLAIN, 1, color, 1)


def plot_bboxes(img, predictions, gt_lines, gt_top, max_error, color = (0,0,255)):
	line_color  = [color[0], (255 - color[1]) // 2, color[2]]
	error_color = [(255 - color[i]) for i in range(3)]
	
	for bbox_list in predictions:
		for bbox in bbox_list:
			(obj_class, diagonal) = bbox
			((xa, ya), (xb, yb)) = diagonal
			((xa, ya), (xb, yb)) = ((int(xa), int(ya)), (int(xb), int(yb)))
			cv2.rectangle(img, (xa, ya), (xb, yb), line_color, 2)
			cv2.putText(img, str(obj_class), ((min(xa, xb) + 3), (min(ya, yb) - 6)), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1)
			(compute_a, compute_b) = compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error)
			(class_a, bbox_a, err_a) = compute_a
			(class_b, bbox_b, err_b) = compute_b
			if bbox_a:
				cv2.circle(img, (xa, ya), 5, color, -1)
			else:
				cv2.circle(img, (xa, ya), 5, error_color, -1)
			if bbox_b:
				cv2.circle(img, (xb, yb), 5, color, -1)
			else:
				cv2.circle(img, (xb, yb), 5, error_color, -1)
	

def plot_image(file_name, elas_predictions, yolo_predictions, gt_lines, gt_top, params):
	(width, height, max_error, yolo_dir, elas_dir, images_dir, save_dir, show_images, show_lines) = params
	img_file = images_dir + file_name[:-3] + 'png'
	if os.path.isfile(img_file):
		img = cv2.imread(img_file)
		(img_h, img_w) = img.shape[0:2]
		if (img_h, img_w) != (height, width):
			raise Exception(4, "ERROR: Image shape %dx%d does not match size parameter: %dx%d" % (img_w, img_h, width, height))
	else:
		print('ERROR: FILE NOT FOUND: ' + img_file)
		img = np.full((height, width, 3), (255,255,255), np.uint8)

	plot_points(img, gt_lines, gt_top, show_lines, color = (255,0,0))

	if elas_dir:
		plot_points(img, elas_predictions, gt_top, show_lines = False, color = (0,255,0))

	if yolo_dir:
		plot_bboxes(img, yolo_predictions, gt_lines, gt_top, max_error, color = (0,0,255))

	if save_dir:
		save_file = save_dir + file_name[:-3] + 'png'
		save_subdir = os.path.dirname(save_file)
		if not os.path.exists(save_subdir):
			os.makedirs(save_subdir)	
		cv2.imwrite(save_file, img)
	
	if show_images:
		window_name = 'Lane Detector'
# 		cv2.namedWindow(window_name)
# 		cv2.moveWindow(window_name, 300, 600)
		while True:
			cv2.imshow(window_name, img)
			key = cv2.waitKey(0) & 0xff
			if key == 10:      # Enter key
				break
			if key == 27:      # ESC key
				sys.exit(0)
# 		cv2.destroyWindow(window_name)


def compute_point(gt_lines, gt_top, point, obj_class, max_error):
	(classif, detect, error) = (False, False, out_of_range)
	(x, y) = point

	for gt_line in gt_lines:
		if len(gt_line) > 0:
			(gt_classif, gt_x) = gt_interpolate(y, gt_line, gt_top)
			gt_error = abs(gt_x - x) if (gt_x != out_of_range) else out_of_range
	
			if gt_error <= error:
				error = gt_error
				detect  |= (error <= max_error)
				classif |= (detect and (obj_class == gt_classif))

	compute_point = (classif, detect, error)
	return compute_point


def compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error):
	(classif_a, detect_a, err_a) = (False, False, out_of_range)
	(classif_b, detect_b, err_b) = (False, False, out_of_range)
	((xa, ya), (xb, yb)) = diagonal
	for gt_line in gt_lines:
		if len(gt_line) > 0:
			(gt_classif_a, gt_xa) = gt_interpolate(ya, gt_line, gt_top)
			(gt_classif_b, gt_xb) = gt_interpolate(yb, gt_line, gt_top)
			gt_err_a = abs(gt_xa - xa) if (gt_xa != out_of_range) else out_of_range
			gt_err_b = abs(gt_xb - xb) if (gt_xb != out_of_range) else out_of_range
	
			if (gt_err_a + gt_err_b) <= (err_a + err_b):
				err_a = gt_err_a
				err_b = gt_err_b
				detect_a |= (err_a <= max_error)
				detect_b |= (err_b <= max_error)
				classif_a |= (detect_a and (obj_class == gt_classif_a))
				classif_b |= (detect_b and (obj_class == gt_classif_b))

	compute_a = (classif_a, detect_a, err_a)
	compute_b = (classif_b, detect_b, err_b)
	return (compute_a, compute_b)


def compute_error(method, predictions, gt_lines, gt_top, max_error):
	(classif_error, classif_true_pos, classif_false_pos) = (0, 0, 0)
	(detect_error,  detect_true_pos,  detect_false_pos)  = (0, 0, 0)

	for prediction in predictions:
		for item in prediction:

			if method == 'Yolo':
				(obj_class, diagonal) = item
				(compute_a, compute_b) = compute_diagonal(gt_lines, gt_top, diagonal, obj_class, max_error)
				compute_points = [compute_a, compute_b]
	
			if method == 'Elas':
				(obj_class, x, y) = item
				compute_p = compute_point(gt_lines, gt_top, (x, y), obj_class, max_error)
				compute_points = [compute_p]
				
			for (classif_p, detect_p, err_p) in compute_points:
				if classif_p:
					classif_error += err_p
					classif_true_pos += 1
				else:
					classif_false_pos += 1

				if detect_p:
					detect_error += err_p
					detect_true_pos += 1
				else:
					detect_false_pos += 1

	if method == 'Yolo':
		expected_points = sum([ (len(gt_line) - 1) for gt_line in gt_lines ]) * 2
	if method == 'Elas':
		expected_points = sum([ len(gt_line) for gt_line in gt_lines ])
		
	classif_false_neg = expected_points - classif_true_pos - classif_false_pos
	detect_false_neg  = expected_points - detect_true_pos  - detect_false_pos
	result_classif = (classif_error, classif_true_pos, classif_false_pos, classif_false_neg)
	result_detect  = (detect_error,  detect_true_pos,  detect_false_pos,  detect_false_neg)

	return (result_classif, result_detect)


def recursive_glob(dir_name, file_pattern, relative = False, origin = ''):
	file_list = []
	if origin == '':
		origin = dir_name

	if os.path.isdir(dir_name):
		if relative:
			file_list = [file_name[len(origin):] for file_name in glob(dir_name + file_pattern) if os.path.isfile(file_name)]
		else:
			file_list = [file_name for file_name in glob(dir_name + file_pattern) if os.path.isfile(file_name)]

		for item in os.listdir(dir_name):
			file_list += recursive_glob(dir_name + item + '/', file_pattern, relative, origin)
	
	return file_list


def update_accum_results(method, accum_results, results):
	(result_classif, result_detect) = results
	(classif_error, classif_true_pos, classif_false_pos, classif_false_neg) = result_classif
	(detect_error,  detect_true_pos,  detect_false_pos,  detect_false_neg)  = result_detect

	accum_results[method][0][0] += classif_error
	accum_results[method][0][1] += classif_true_pos
	accum_results[method][0][2] += classif_false_pos
	accum_results[method][0][3] += classif_false_neg
	accum_results[method][1][0] += detect_error
	accum_results[method][1][1] += detect_true_pos
	accum_results[method][1][2] += detect_false_pos
	accum_results[method][1][3] += detect_false_neg


def main(ground_truth_dir, params):
	(width, height, max_error, yolo_dir, elas_dir, images_dir, save_dir, show_images, show_lines) = params
	dir_name = yolo_dir if yolo_dir else ground_truth_dir
	file_pattern = 'lane_*.txt'
	file_list = recursive_glob(dir_name, file_pattern, relative = True)
	if not file_list:
		raise Exception(3, "ERROR: No " + file_pattern + " files found in: " + dir_name)
	
	elas_predictions = []
	yolo_predictions = []
	accum_results = {}
	if elas_dir:
		accum_results['Elas'] = [[0, 0, 0, 0], [0, 0, 0, 0]]
	if yolo_dir:
		accum_results['Yolo'] = [[0, 0, 0, 0], [0, 0, 0, 0]]

	for file_name in sorted(file_list, cmp = file_name_by_number):
		gt_lines = read_points_file(ground_truth_dir + file_name, width, height)
		gt_top = get_ground_truth_top(gt_lines, height)

		if elas_dir:
			elas_predictions = read_points_file(elas_dir + file_name, width, height)
			results = compute_error('Elas', elas_predictions, gt_lines, gt_top, max_error)
			update_accum_results('Elas', accum_results, results)

		if yolo_dir:
			yolo_bboxes = read_bboxes_file(yolo_dir + file_name, width, height)
			yolo_predictions = bboxes_clustering(yolo_bboxes, width)
			results = compute_error('Yolo', yolo_predictions, gt_lines, gt_top, max_error)
			update_accum_results('Yolo', accum_results, results)
		
		if show_images or save_dir:
			plot_image(file_name, elas_predictions, yolo_predictions, gt_lines, gt_top, params)
	
	return accum_results


def print_stats(results):
	for method in results.keys():
		(stats_classif, stats_detect) = results[method]
		print('\n' + method.upper() + '  statistics:\n')
		for (result_type, stats) in (('classification', stats_classif), ('detection', stats_detect)):
			try:
				print('   ' + result_type.capitalize() + ':')
				print('      True positives:  (%6d)'  % stats.true_pos  + ' predicted points')
				print('      False positives: (%6d)'  % stats.false_pos + ' predicted points')
				print('      False negatives: (%6d)'  % stats.false_neg + ' unpredicted points')
				print('      Total:           (%6d)'  % stats.total()   + ' points')
				print('      Average error:    %6.2f' % stats.avg_error() + '  pixels')
				print('      Precision:  %6.2f%%' % (100 * stats.precision()))
				print('      Recall:     %6.2f%%' % (100 * stats.recall()))
				print('      Accuracy:   %6.2f%%' % (100 * stats.accuracy()) + '\n')

			except TypeError:
				pass


def get_image_size(size):
	try:
		dims = size.split('x')
		if len(dims) != 2:
			raise ValueError

		image_width  = int(dims[0]) 
		image_height = int(dims[1])
	
	except ValueError:
		raise Exception(0, "ERROR: Invalid image size format wxh: " + size)
	
	return (image_width, image_height)
	

def check_argv(argv, i):
	if i >= len(argv) or argv[i][0] == '-':
		raise Exception(0, "ERROR: argument expected after [%d]: " % (i - 1) + argv[i - 1])
	return i


def read_parameters(argv):
	width = 640
	height = 480
	max_error = 20
	yolo_dir = ''
	elas_dir = ''
	images_dir = ''
	save_dir = ''
	show_images = False
	show_lines = False

	i = 2
	while i < len(argv):
		if argv[i] == '-size':
			i = check_argv(argv, i + 1)
			(width, height) = get_image_size(argv[i])

		elif argv[i] == '-error':
			i = check_argv(argv, i + 1)
			max_error = float(argv[i])

		elif argv[i] == '-yolo':
			i = check_argv(argv, i + 1)
			yolo_dir   = argv[i] if argv[i].endswith('/') else argv[i] + '/'

		elif argv[i] == '-elas':
			i = check_argv(argv, i + 1)
			elas_dir   = argv[i] if argv[i].endswith('/') else argv[i] + '/'

		elif argv[i] == '-img':
			i = check_argv(argv, i + 1)
			images_dir = argv[i] if argv[i].endswith('/') else argv[i] + '/'

		elif argv[i] == '-save':
			i = check_argv(argv, i + 1)
			save_dir   = argv[i] if argv[i].endswith('/') else argv[i] + '/'
				
		elif argv[i] == '-show':
			show_images = True

		elif argv[i] == '-lines':
			show_lines = True

		else:
			raise Exception(2, "ERROR: Invalid command line argument [%d]: " % i + argv[i])
		
		i += 1

	params = (width, height, max_error, yolo_dir, elas_dir, images_dir, save_dir, show_images, show_lines)
	return params


if __name__ == "__main__":
	try:
		if not 2 <= len(sys.argv) <= 16:
			raise Exception(0, 'usage')

		ground_truth_dir = sys.argv[1] if sys.argv[1].endswith('/') else sys.argv[1] + '/'
		params = read_parameters(sys.argv)
		
		results = main(ground_truth_dir, params)
		print_stats(results)
		sys.exit(0)
	
	except Exception as error:
		if error.args[-1] != 'usage':
			print('\n' + error.args[-1] + '\n')
		
		print("\nUsage: python " + sys.argv[0] + "  <gt_dir>  -size <wxh>  -error <pixels>"
			"  -yolo <yolo_results_dir>  -elas <elas_results_dir>"
			"  -show  -lines  -img <images_dir>  -save <plot_results_dir>\n")
		print("Example: python " + sys.argv[0] + "  /lane_dataset/groundtruth/labels  -size 640x480  -error 30"
			"  -yolo /lane_dataset/yolo/labels  -elas /lane_dataset/elas/labels"
			"  -show  -lines  -img /lane_dataset/groundtruth/images  -save /results/\n")
		print("Note: Ground truth label records must be in the format: class line x y (in pixels)")
		print("      Elas result  label records must be in the format: class line x y (in pixels)")
		print("      Yolo result  label records must be in the format: class x y w h (in fractions)\n")

		if len(error.args) == 1:
			raise
		sys.exit(1)
