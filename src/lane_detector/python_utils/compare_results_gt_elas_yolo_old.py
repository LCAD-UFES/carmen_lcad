import sys
import os
import math
import cv2
import matplotlib.pyplot as plt

def read_groud_truth_points(gt_dir, gt_file_name):
	#print (gt_dir + gt_file_name)
	
	gt_file = open(gt_dir + gt_file_name, "r")
	file_content = gt_file.readlines()
	
	gt_points_list = []
	count = 0
	gt_points_left = []
	gt_points_right = []
	for line in file_content:
		line = line.strip().split()
		line[0] = int(line[0])
		line[1] = int(line[1])
		if count < 4:
			gt_points_left.append(line[:])
		else:
			gt_points_right.append(line[:])
		count = count + 1
		#print(gt_points_right)
	gt_points_list.append(gt_points_left[:])
	gt_points_list.append(gt_points_right[:])
	#print(gt_points_list)
	return gt_points_list

def transform_groud_truth_points_to_yolo(gt_points_list):
	yolo_points_list_left = []
	yolo_points_list_right = []
	yolo_points_list = []
	points = []
	for i in range(len(gt_points_list[0]) - 1):
		points.append(float((gt_points_list[0][i + 1][0] + gt_points_list[0][i][0])) / (2.0 * 640.0))
		points.append(float((gt_points_list[0][i + 1][1] + gt_points_list[0][i][1])) / (2.0 * 480.0))
		points.append(abs(float((gt_points_list[0][i + 1][1] - gt_points_list[0][i][1])) / 480.0))
		points.append(abs(float((gt_points_list[0][i + 1][0] - gt_points_list[0][i][0])) / 640.0))
		yolo_points_list_left.append(points[:])
		del points[:]
	
	for i in range(len(gt_points_list[1]) - 1):
		points.append(float((gt_points_list[1][i + 1][0] + gt_points_list[1][i][0])) / (2.0 * 640.0))
		points.append(float((gt_points_list[1][i + 1][1] + gt_points_list[1][i][1])) / (2.0 * 480.0))
		points.append(abs(float((gt_points_list[1][i + 1][1] - gt_points_list[1][i][1])) / 480.0))
		points.append(abs(float((gt_points_list[1][i + 1][0] - gt_points_list[1][i][0])) / 640.0))
		yolo_points_list_right.append(points[:])
		del points[:]
	
	yolo_points_list.append(yolo_points_list_left[:])
	yolo_points_list.append(yolo_points_list_right[:])
	return yolo_points_list

def transform_yolo_points_groundtruth_to_coordinates(gt_dir, gt_file_name):
	points = []
	gt_points_list = []
	gt_points_list_left = []
	gt_points_list_right = []
	gt_file = open(gt_dir + gt_file_name, "r")
	file_content = gt_file.readlines()
	
	i = 0
	for line in file_content:
		line = line.strip().split()
		if (i == 0):
			points.append(int((float(line[0]) + (float(line[2]) / 2)) * 640.0))
			points.append(int((float(line[1]) - (float(line[3]) / 2)) * 480.0))
			gt_points_list_left.append(points[:])
		del(points[:])
		i = i + 1
		points.append(int((float(line[0]) - (float(line[2]) / 2)) * 640.0))
		points.append(int((float(line[1]) + (float(line[3]) / 2)) * 480.0))
		gt_points_list_left.append(points[:])
		del(points[:])
		if (i == 3):
			break
	
	i = 0
	for line in file_content:
		line = line.strip().split()
		if (i == 3):
			points.append(int((float(line[0]) - (float(line[2]) / 2)) * 640.0))
			points.append(int((float(line[1]) - (float(line[3]) / 2)) * 480.0))
			gt_points_list_right.append(points[:])
		del (points[:])
		i = i + 1
		if (i > 3):
			points.append(int((float(line[0]) + (float(line[2]) / 2)) * 640.0))
			points.append(int((float(line[1]) + (float(line[3]) / 2)) * 480.0))
			gt_points_list_right.append(points[:])
			del (points[:])
		
	gt_points_list.append(gt_points_list_left[:])
	gt_points_list.append(gt_points_list_right[:])
	return gt_points_list

def dist(x1, y1, x2, y2):
	dx = float(x1) - float(x2)
	dy = float(y1) - float(y2)
	return math.sqrt((dx * dx) + (dy * dy))


def read_and_convert_4_points_coordinates(predictions_dir, gt_file_name, image_width, image_heigth):
	predictions_files_list = open(predictions_dir + gt_file_name, "r")
	content = predictions_files_list.readlines()
	
	prediction = []
	predictions_list = []
	predictions_list_left = []
	predictions_list_right = []
	xmin = 999999
	xmax = 0
	for line in content:
		line = line.replace('\n', '').rsplit(' ')
		if (int((float(line[1]) - (float(line[3]) / 2)) * image_width) < xmin):
			xmin = int((float(line[1]) - (float(line[3]) / 2)) * image_width)
		if (int((float(line[1]) + (float(line[3]) / 2)) * image_width) > xmax):
			xmax = int((float(line[1]) + (float(line[3]) / 2)) * image_width) 
    
	#print ("xmin = ",xmin)
	#print ("xmax = ",xmax)
	
	for line in content:
		line = line.replace('\n', '').rsplit(' ')
		if (int(float(line[1]) * image_width) > xmin + (xmax - xmin) / 2):
			prediction.append(int((float(line[1]) + (float(line[3]) / 2)) * image_width))
			prediction.append(int((float(line[2]) + (float(line[4]) / 2)) * image_heigth))
			prediction.append(int((float(line[1]) - (float(line[3]) / 2)) * image_width))
			prediction.append(int((float(line[2]) - (float(line[4]) / 2)) * image_heigth))	
			predictions_list_right.append(prediction[:])
		else:
			prediction.append(int((float(line[1]) - (float(line[3]) / 2)) * image_width))
			prediction.append(int((float(line[2]) + (float(line[4]) / 2)) * image_heigth))
			prediction.append(int((float(line[1]) + (float(line[3]) / 2)) * image_width))
			prediction.append(int((float(line[2]) - (float(line[4]) / 2)) * image_heigth))
			predictions_list_left.append(prediction[:])
		#print(predictions_list)
		del prediction[:]

	predictions_list.append(predictions_list_left[:])
	predictions_list.append(predictions_list_right[:])
	#print(predictions_list)

	return predictions_list


def find_image_path():
	for i, param in enumerate(sys.argv):
		if param == '-show':
			return sys.argv[i + 1]
	return ""


def show_image(gt_points, predictions_points, predictions_points_2, gt_file_path, images_path):	
	img = cv2.imread(images_path + gt_file_name.replace('txt', 'png'))
	print(images_path)
	
	
	#imprime os pontos do groundthruth em linhas
	for i in range(len(gt_points)):
		for j in (range(len(gt_points[i]) - 1)):
			tuple_of_points_0 = (int(gt_points[i][j][0]), int(gt_points[i][j][1]))
			tuple_of_points_1 = (int(gt_points[i][j+1][0]), int(gt_points[i][j+1][1]))
			cv2.line(img, tuple_of_points_0, tuple_of_points_1, (255,0,0), 1)
			j = j + 1
	
	#imprime os bound boxes da yolo
	for i in range(len(predictions_points[0])):
		tuple_of_points_0 = (int(predictions_points[0][i][0]), int(predictions_points[0][i][1]))
		tuple_of_points_1 = (int(predictions_points[0][i][2]), int(predictions_points[0][i][3]))
		cv2.rectangle(img, tuple_of_points_0, tuple_of_points_1, (255, 0, 255), 2)
	
	for i in range(len(predictions_points[1])):
		tuple_of_points_0 = (int(predictions_points[1][i][0]), int(predictions_points[1][i][1]))
		tuple_of_points_1 = (int(predictions_points[1][i][2]), int(predictions_points[1][i][3]))
		cv2.rectangle(img, tuple_of_points_0, tuple_of_points_1, (255, 0, 255), 2)


	
	#imprime os pontos do ELAS em circulos
	for i in range(len(predictions_points_2[0])):
		tuple_of_points_0 = (int(predictions_points_2[0][i][0]), int(predictions_points_2[0][i][1]))
		tuple_of_points_1 = (int(predictions_points_2[0][i][2]), int(predictions_points_2[0][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 255, 255), 5)
		cv2.circle(img, tuple_of_points_1, 1, (0, 255, 255), 5)
	
	for i in range(len(predictions_points_2[1])):
		tuple_of_points_0 = (int(predictions_points_2[1][i][0]), int(predictions_points_2[1][i][1]))
		tuple_of_points_1 = (int(predictions_points_2[1][i][2]), int(predictions_points_2[1][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 255, 255), 5)
		cv2.circle(img, tuple_of_points_1, 1, (0, 255, 255), 5)

	#imprime os pontos da yolo em circulos
	for i in range(len(predictions_points[0])):
		tuple_of_points_0 = (int(predictions_points[0][i][0]), int(predictions_points[0][i][1]))
		tuple_of_points_1 = (int(predictions_points[0][i][2]), int(predictions_points[0][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 0, 255), 2)
		cv2.circle(img, tuple_of_points_1, 1, (0, 0, 255), 2)
	
	for i in range(len(predictions_points[1])):
		tuple_of_points_0 = (int(predictions_points[1][i][0]), int(predictions_points[1][i][1]))
		tuple_of_points_1 = (int(predictions_points[1][i][2]), int(predictions_points[1][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 0, 255), 2)
		cv2.circle(img, tuple_of_points_1, 1, (0, 0, 255), 2)	
	
	while (1):
		cv2.imshow('Lane Detector Compute Error', img)
		key = cv2.waitKey(0) & 0xff
		
		if key == 10 or key == 13:      # Enter key
			#cv2.imwrite(gt_file_name.replace('txt', 'png'), img)
			break
		elif key == 27:    # ESC key
			sys.exit()


def distance_point_line(p1, p2, p):
	dy = float(p2[1]) - float(p1[1])
	dya = float(p[1]) - float(p1[1])
	
	pdya = dya / dy
	
	x = float(p1[0]) + ((float(p2[0]) - float(p1[0])) * pdya)
	
	error = abs(int(x) - p[0])
	
	return error, x


def distance_point_line2(p1, p2, p):
	x1 = p1[0]
	y1 = p1[1]
	x2 = p2[0]
	y2 = p2[1]
	ya = p[1]
	
	num = (x2 * (y1 - ya)) + (x1 * (ya - y2))
	den = y1 - y2
	
	x = float(num) / float(den)
	x = int(x)
	
	distance = abs(x - p[0])
	
	return distance


def find_index_to_line_error_down(gt_points_side, predictions_points):
	min_dist = 999999
	distance = 0.0
	index = 10
	
	for i in range(len(gt_points_side)):
		distance = dist(gt_points_side[i][0], gt_points_side[i][1], predictions_points[0], predictions_points[1])
		if distance < min_dist:
			min_dist = distance
			index = i
	return index

def find_index_to_line_error_up(gt_points_side, predictions_points):
	min_dist = 999999
	distance = 0.0
	index = 10
	
	for i in range(len(gt_points_side)):
		distance = dist(gt_points_side[i][0], gt_points_side[i][1], predictions_points[2], predictions_points[3])
		if distance < min_dist:
			min_dist = distance
			index = i
	return index


def compute_error(gt_points, predictions_points, aux):
	index_down = 10
	index_up = 10
	point = []
	error_left = 0
	error_right = 0
	error = False
	for i in range(len(predictions_points[0])):
		index_down = find_index_to_line_error_down(gt_points[0], predictions_points[0][i])
		point.append(predictions_points[0][i][0])
		point.append(predictions_points[0][i][1])
		for a in gt_points[0]:
			if a[0] < 0:
				error = True
		if error:
			error = False
			continue
		if index_down <= len(gt_points[0]) - 2:
			error_left += distance_point_line2(gt_points[0][index_down], gt_points[0][index_down+1], point)
		else:
			error_left += distance_point_line2(gt_points[0][index_down], gt_points[0][index_down-1], point)
		del(point[:])
		index_up = find_index_to_line_error_up(gt_points[0], predictions_points[0][i])
		point.append(predictions_points[0][i][2])
		point.append(predictions_points[0][i][3])
		if index_up <= len(gt_points) - 2:
			error_left += distance_point_line2(gt_points[0][index_up], gt_points[0][index_up+1], point)
		else:
			error_left += distance_point_line2(gt_points[0][index_up], gt_points[0][index_up-1], point)
		del(point[:])
	if len(predictions_points[0]) != 0:
		error_left = error_left / (len(predictions_points[0]) * 2)
	else:
		error_left = 0
	for i in range(len(predictions_points[1])):
		index_down = find_index_to_line_error_down(gt_points[1], predictions_points[1][i])
		point.append(predictions_points[1][i][0])
		point.append(predictions_points[1][i][1])
		for a in gt_points[1]:
			if a[0] < 0:
				error = True
		if error:
			error = False
			continue
		if index_down <= len(gt_points[1]) - 2:
			error_right += distance_point_line2(gt_points[1][index_down], gt_points[1][index_down+1], point)
		else:
			error_right += distance_point_line2(gt_points[1][index_down], gt_points[1][index_down-1], point)
		del(point[:])
		index_up = find_index_to_line_error_up(gt_points[1], predictions_points[1][i])
		point.append(predictions_points[1][i][2])
		point.append(predictions_points[1][i][3])
		if index_up <= len(gt_points) - 2:
			error_right += distance_point_line2(gt_points[1][index_up], gt_points[1][index_up+1], point)
		else:
			error_right += distance_point_line2(gt_points[1][index_up], gt_points[1][index_up-1], point)
		del(point[:])
	if len(predictions_points[1]) != 0:
		error_right = error_right / (len(predictions_points[1]) * 2)
	else:
		error_right = 0
	error_total = error_left + error_right
	return error_total

if __name__ == "__main__":
	#if len(sys.argv) < 5 or len(sys.argv) > 9:
		#print ("\nUse: python", sys.argv[0], "ground_truth_dir predictions1_dir predictions2_dir image_width image_heigth -show images_path (optional) -format jpg (optional)\n")
	#else:
		if not sys.argv[1].endswith('/'):
			sys.argv[1] += '/'
		if not sys.argv[2].endswith('/'):
			sys.argv[2] += '/'
		if not sys.argv[3].endswith('/'):
			sys.argv[3] += '/'
		image_width  = 640 #640 #int(sys.argv[4])
		image_heigth = 480 #480 #int(sys.argv[5])
		yolo_base = sys.argv[2]
		number_iterations_folder = [l for l in os.listdir(yolo_base)]
		number_iterations_folder = list(map(int, number_iterations_folder))
		number_iterations_folder.sort()
		print(number_iterations_folder)
		elas_base = sys.argv[3]
		gt_base = sys.argv[1]
		error = 0
		error_elas = 0
		cont = 0
		error_array = []
		error_array_2 = []
		array_numbers = []
		for l in number_iterations_folder:
			if (int(l) == 10000):
				continue
			folder_name = [a for a in os.listdir(yolo_base + str(l) + "/")]
			for f in folder_name:
				yolo_path = yolo_base + str(l) + "/" + str(f) + "/"
				print(yolo_path)
				gt_files_list = [b for b in os.listdir(yolo_path)]
				for gt_file_name in gt_files_list:
					if not gt_file_name.endswith('.txt'):
						continue
					gt_points = read_groud_truth_points(sys.argv[1] + str(f) + "/", gt_file_name)
					#gt_points_yolo = transform_groud_truth_points_to_yolo(gt_points)
					
					#print (gt_points_yolo)

					#gt_points_true = transform_yolo_points_groundtruth_to_coordinates(sys.argv[1], gt_file_name)

					#print (gt_points_true)
					predictions_points = read_and_convert_4_points_coordinates(yolo_path, gt_file_name, image_width, image_heigth)
					if (len(predictions_points[0]) == 0 or len(predictions_points[1]) == 0):
						continue
					predictions_points_2 = read_and_convert_4_points_coordinates(sys.argv[3] + str(f) + "/", gt_file_name, image_width, image_heigth)
					
					#print (predictions_points)
					#returned = compute_error(gt_points_true, predictions_points)
					returned = compute_error(gt_points, predictions_points, 0)
					returned_2 = compute_error(gt_points, predictions_points_2, 1)
					if (float(returned) < 500.0):
						error += returned
					else:
						continue
					cont += 1
					if (int(l) == 40000 and float(returned_2) < 500.0):
						error_elas += returned_2
					#if images_path:
						#show_image(gt_points, predictions_points, returned[1], returned_2[1], gt_file_name, images_path)
					#show_image(gt_points, predictions_points, predictions_points_2, gt_file_name, images_path)
				#print(number_of_folder)
			print (cont)
			print ('TOTAL Error Yolo: ' + str(error/cont))
			print ('TOTAL Error ELAS: ' + str((error_elas )/cont))
			error_array.append(int(error/cont))
			array_numbers.append(int(l))
			error = 0
			cont = 0
			error_elas = 0
		if yolo_base.endswith("val_2/"):
			yolo_base = yolo_base.replace("val_2", "test_2")
		elif yolo_base.endswith("val_1/"):
			yolo_base = yolo_base.replace("val_1", "test_1")
		for l in number_iterations_folder:
			if (int(l) == 10000):
				continue
			folder_name = [a for a in os.listdir(yolo_base + str(l) + "/")]
			for f in folder_name:
				yolo_path = yolo_base + str(l) + "/" + str(f) + "/"
				print(yolo_path)
				gt_files_list = [b for b in os.listdir(yolo_path)]
				for gt_file_name in gt_files_list:
					if not gt_file_name.endswith('.txt'):
						continue
					gt_points = read_groud_truth_points(sys.argv[1] + str(f) + "/", gt_file_name)
					#gt_points_yolo = transform_groud_truth_points_to_yolo(gt_points)
					
					#print (gt_points_yolo)

					#gt_points_true = transform_yolo_points_groundtruth_to_coordinates(sys.argv[1], gt_file_name)

					#print (gt_points_true)
					predictions_points = read_and_convert_4_points_coordinates(yolo_path, gt_file_name, image_width, image_heigth)
					if (len(predictions_points[0]) == 0 or len(predictions_points[1]) == 0):
						continue
					predictions_points_2 = read_and_convert_4_points_coordinates(sys.argv[3] + str(f) + "/", gt_file_name, image_width, image_heigth)
					
					#print (predictions_points)
					#returned = compute_error(gt_points_true, predictions_points)
					returned = compute_error(gt_points, predictions_points, 0)
					returned_2 = compute_error(gt_points, predictions_points_2, 1)
					if (float(returned) < 500.0):
						error += returned
					else:
						continue
					cont += 1
					if (int(l) == 40000 and float(returned_2) < 500.0):
						error_elas += returned_2
					#if images_path:
						#show_image(gt_points, predictions_points, returned[1], returned_2[1], gt_file_name, images_path)
					#show_image(gt_points, predictions_points, predictions_points_2, gt_file_name, images_path)
				#print(number_of_folder)
			print (cont)
			print ('TOTAL Error Yolo: ' + str(error/cont))
			print ('TOTAL Error ELAS: ' + str((error_elas )/cont))
			error_array_2.append(int(error/cont))
			error = 0
			cont = 0
			error_elas = 0
		plt.plot(array_numbers, error_array, label='validation')
		plt.plot( array_numbers, error_array_2,  label= 'test')
		plt.xlabel('Iteration')
		plt.ylabel('Average error of detection')
		titulo = "Error of detection"
		plt.title(str(titulo))
		plt.legend()
		plt.show()
		
