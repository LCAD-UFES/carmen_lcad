import sys
import os
import math
import cv2

# Mudar esta funcao para ler os pontos do ground truth e gerar um lista dupla (direita esquerda)
#check
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

def dist(x1, y1, x2, y2):
	dx = x1 - x2
	dy = y1 - y2
	
	return math.sqrt((dx * dx) + (dy * dy))


def read_and_convert_4_points_coordinates(predictions_dir, gt_file_name, image_width, image_heigth):
	print (predictions_dir + gt_file_name)
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
	# Modificar essa funcao para retornar uma lista dupla (todos os pontos direita e todos os pontos da esquerda)
	#check

	return predictions_list


def find_image_path():
	for i, param in enumerate(sys.argv):
		if param == '-show':
			return sys.argv[i + 1]
	return ""


#def show_image(gt_points, predictions_points, chosen_points_list, chosen_points_list_2, gt_file_path, images_path):
def show_image(gt_points, predictions_points,  gt_file_path, images_path):	
	img = cv2.imread(images_path + gt_file_name.replace('txt', 'png'))
	print(images_path)

	#for p in predictions_points:
		#cv2.rectangle(img, (p[0]-1, p[1]-1), (p[0]+1, p[1]+1), (0,0,255), 2)
	for i in range(len(predictions_points[0])):
		tuple_of_points_0 = (int(predictions_points[0][i][0]), int(predictions_points[0][i][1]))
		tuple_of_points_1 = (int(predictions_points[0][i][2]), int(predictions_points[0][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 0, 255), 10)
		cv2.circle(img, tuple_of_points_1, 1, (0, 0, 255), 10)
	
	for i in range(len(predictions_points[1])):
		tuple_of_points_0 = (int(predictions_points[1][i][0]), int(predictions_points[1][i][1]))
		tuple_of_points_1 = (int(predictions_points[1][i][2]), int(predictions_points[1][i][3]))
		cv2.circle(img, tuple_of_points_0, 1, (0, 255, 0), 10)
		cv2.circle(img, tuple_of_points_1, 1, (0, 255, 0), 10)
	
	#imprime os pontos do groundthruth em linhas
	for i in range(len(gt_points)):
		for j in (range(len(gt_points[i]) - 1)):
			tuple_of_points_0 = (int(gt_points[i][j][0]), int(gt_points[i][j][1]))
			tuple_of_points_1 = (int(gt_points[i][j+1][0]), int(gt_points[i][j+1][1]))
			cv2.line(img, tuple_of_points_0, tuple_of_points_1, (255,0,0), 1)
			j = j + 1

	
	#for p in chosen_points_list:
	#	cv2.rectangle(img, (p[0]-1, p[1]-1), (p[0]+1, p[1]+1), (0,255,255), 2)
		
	#for p in chosen_points_list_2:
	#	cv2.rectangle(img, (p[0]-1, p[1]-1), (p[0]+1, p[1]+1), (0,0,255), 2)
		
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
	
	return distance, x


def comppute_point_to_line_error(line_p1, line_p2, predictions_points):
	min_dist = 999999
	
	for p in predictions_points:
		distance = dist(line_p1[0], line_p1[1], p[0], p[1])
		
		if distance < min_dist:
			point = p
			min_dist = distance
			
	returned = distance_point_line2(line_p1, line_p2, point)
	point[0] = returned[1]
	
	return returned[0], point


def compute_error(gt_points, predictions_points):
	error = 0
	chosen_points_list = []
	
	returned = comppute_point_to_line_error(gt_points[0], gt_points[1], predictions_points)   # Top Left
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[1], gt_points[2], predictions_points)
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[2], gt_points[3], predictions_points)
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[3], gt_points[2], predictions_points)   # Botton Left
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[4], gt_points[5], predictions_points)   # Top Right
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[5], gt_points[6], predictions_points)
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[6], gt_points[7], predictions_points)
	error += returned[0]
	chosen_points_list.append(returned[1])
	
	returned = comppute_point_to_line_error(gt_points[7], gt_points[6], predictions_points)   # Botton Right
	error += returned[0]
	chosen_points_list.append(returned[1])

	print ('Error: ' + str(error))
	
	return error, chosen_points_list


if __name__ == "__main__":
	#if len(sys.argv) < 5 or len(sys.argv) > 9:
		#print ("\nUse: python", sys.argv[0], "ground_truth_dir predictions1_dir predictions2_dir image_width image_heigth -show images_path (optional) -format jpg (optional)\n")
	#else:
		if not sys.argv[1].endswith('/'):
			sys.argv[1] += '/'
		if not sys.argv[2].endswith('/'):
			sys.argv[2] += '/'
		#if not sys.argv[3].endswith('/'):
			#sys.argv[3] += '/'
			
		image_width  = 640 #640 #int(sys.argv[4])
		image_heigth = 480 #480 #int(sys.argv[5])
		
		images_path = sys.argv[2].replace('labels', 'images')
		
		gt_files_list = [l for l in os.listdir(sys.argv[1])]
		gt_files_list.sort()
		
		error = 0
		cont = 0
		for gt_file_name in gt_files_list:
			if not gt_file_name.endswith('.txt'):
				continue
				
			gt_points = read_groud_truth_points(sys.argv[1], gt_file_name)
			#print(gt_points[0][1])
			predictions_points = read_and_convert_4_points_coordinates(sys.argv[2], gt_file_name, image_width, image_heigth)
			
			#print(predictions_points[0][0][1])
			#predictions_points_2 = read_and_convert_4_points_coordinates(sys.argv[3], gt_file_name, image_width, image_heigth)
			
			#returned = compute_error(gt_points, predictions_points)
			
			#returned_2 = compute_error(gt_points, predictions_points_2)
			
			#error += returned[0]
			cont += 1
			
			#if images_path:
				#show_image(gt_points, predictions_points, returned[1], returned_2[1], gt_file_name, images_path)
			show_image(gt_points, predictions_points, gt_file_name, images_path)
		#print ('TOTAL Error: ' + str(error/cont))
		
		
		
