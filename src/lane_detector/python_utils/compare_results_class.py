import sys
import os
import math
import cv2
import matplotlib.pyplot as plt

def read_groud_truth_points(gt_dir, gt_file_name):
	#print (gt_dir + gt_file_name)
	
	gt_file = open(gt_dir + gt_file_name, "r")
	file_content = gt_file.readlines()
	
	gt_points_class = []
	count = 0
	gt_class_left = []
	gt_class_right = []
	for line in file_content:
		line = line.strip().split()
		if count < 2:
			gt_class_left.append(int(line[0]))
		else:
			gt_class_right.append(int(line[0]))
		count = count + 1
		#print(gt_points_right)
	gt_points_class.append(gt_class_left[:])
	gt_points_class.append(gt_class_right[:])
	#print(gt_points_class)
	return gt_points_class

def read_and_convert_4_points_coordinates(predictions_dir, gt_file_name, image_width, image_heigth):
	print (predictions_dir + gt_file_name)
	predictions_files_list = open(predictions_dir + gt_file_name, "r")
	content = predictions_files_list.readlines()
	class_number = []
	class_number_right = []
	class_number_left = []
	xmin = 999999
	xmax = 0
	ymax = 0
	ymin = 999999
	for line in content:
		line = line.replace('\n', '').rsplit(' ')
		if (int((float(line[1]) - (float(line[3]) / 2)) * image_width) < xmin):
			xmin = int((float(line[1]) - (float(line[3]) / 2)) * image_width)
		if (int((float(line[1]) + (float(line[3]) / 2)) * image_width) > xmax):
			xmax = int((float(line[1]) + (float(line[3]) / 2)) * image_width) 
		if (int((float(line[2]) - (float(line[4]) / 2)) * image_heigth) < ymin):
			ymin = int((float(line[2]) - (float(line[4]) / 2)) * image_heigth)
		if (int((float(line[2]) + (float(line[4]) / 2)) * image_heigth) > ymax):
			ymax = int((float(line[2]) + (float(line[4]) / 2)) * image_heigth) 
	#print ("xmin = ",xmin)
	#print ("xmax = ",xmax)
	for line in content:
		line = line.replace('\n', '').rsplit(' ')
		if (int(float(line[1]) * image_width) > xmin + (xmax - xmin) / 2 and 
			int(float(line[2]) * image_heigth) > ymin + (ymax - ymin) / 2):
			class_number_right.append(int(line[0]))
		if (int(float(line[1]) * image_width) < xmin + (xmax - xmin) / 2 and 
			int(float(line[2]) * image_heigth) > ymin + (ymax - ymin) / 2):
			class_number_left.append(int(line[0]))
		#print(predictions_list)
	class_number.append(class_number_left[:])
	class_number.append(class_number_right[:])
	#print(predictions_list)
	del class_number_left[:]
	del class_number_right[:]
	return class_number


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

def compute_error_class(gt_points, predictions_points):
	error = 0
	if predictions_points[0]:
		if gt_points[0][0] != predictions_points[0][0]:
			error += 1
	if predictions_points[1]:
		if gt_points[1][0] != predictions_points[1][0]:
			error += 1
	error = error / 2
	return error

if __name__ == "__main__":
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
		cont = 0
		error_elas = 0
		error_yolo = 0
		class_number_yolo = []
		class_number_elas = []
		array_numbers = []
		error_array = []
		
		for l in number_iterations_folder:
			folder_name = [a for a in os.listdir(yolo_base + str(l) + "/")]
			for f in folder_name:
				yolo_path = yolo_base + str(l) + "/" + str(f) + "/"
				gt_files_list = [b for b in os.listdir(yolo_path)]
				for gt_file_name in gt_files_list:
					if not gt_file_name.endswith('.txt'):
						continue
					gt_points_class = read_groud_truth_points(sys.argv[1] + str(f) + "/", gt_file_name)
					class_number_yolo = read_and_convert_4_points_coordinates(yolo_path, gt_file_name, image_width, image_heigth)
					print (class_number_yolo)
					class_number_elas = read_and_convert_4_points_coordinates(sys.argv[3] + str(f) + "/", gt_file_name, image_width, image_heigth)
					"""for i in range(len(class_number_elas)):
						for j in class_number_elas[i]:
							print(j)
						print('\n')"""
					returned = compute_error_class(gt_points_class, class_number_yolo)
					cont += 1
					error_yolo += returned
					returned_2 = compute_error_class(gt_points_class, class_number_elas)
					error_elas += returned_2
			print ("Error yolo ", error_yolo/cont)
			print ("Error elas ", error_elas/cont)
			error_array.append(float(error_yolo/cont * 100))
			array_numbers.append(int(l))
			error_yolo = 0
			error_elas = 0
		print(error_array)
		plt.plot(array_numbers, error_array)
		plt.xlabel('Iteration')
		plt.ylabel('Percentage')
		titulo = "Error of classification"
		plt.title(str(titulo))
		plt.show()
