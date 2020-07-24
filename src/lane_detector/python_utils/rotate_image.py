#/home/genilsoncruz/Downloads/BR_S01/labels /home/genilsoncruz/Downloads/BR_S01/images 640x480 45

import sys
import os
import cv2
import numpy as np
from glob import glob
import math

def rotate_image_v2(original_image, angle):
	height, width = original_image.shape[:2]
	image_center = (width / 2, height / 2)	
	rotation_matrix = cv2.getRotationMatrix2D(image_center, angle, 1)
	rotated_imagem = cv2.warpAffine(original_image, rotation_matrix, (width, height))
	return rotated_imagem
	
def rotate_image(original_image, angle):
	height, width = original_image.shape[:2]
	image_center = (width / 2, height / 2)
	
	rotation_matrix = cv2.getRotationMatrix2D(image_center, angle, 1)
	
	radians = math.radians(angle)
	sin = math.sin(radians)
	cos = math.cos(radians)
	bound_w = int((height * abs(sin)) + (width * abs(cos)))
	bound_h = int((height * abs(cos)) + (width * abs(sin)))
	
	rotation_matrix[0, 2] += ((bound_w / 2) - image_center[0])
	rotation_matrix[1, 2] += ((bound_h / 2) - image_center[1])
	
	padding_x = (bound_w - width) / 2 + 1
	padding_y = (bound_h - height) / 2 + 1
	
	rotated_imagem = cv2.warpAffine(original_image, rotation_matrix, (bound_w, bound_h))
	return rotated_imagem, padding_x, padding_y #, bound_w, bound_h

def rotate_point(x, y, angle):
    radians = math.radians(angle)
    sin =  round(math.sin(radians), 8)
    cos = round(math.cos(radians), 8)    
    new_x = int(x * abs(cos) - y * abs(sin))
    new_y = int(x * abs(sin) + y * abs(cos))    
    return new_x, new_y

def rotate(x, y, angle, origem_x, origem_y, w, h):
	# Inverte Eixo Y - Para transformar as coordenadas da tela para o plano de rotacao no ponto e na origem
	delta_y = h - y * 2
	delta_origem_y = h - origem_y * 2
	
	new_x = x
	new_y = y + delta_y
	
	new_origem_x = origem_x
	new_origem_y = origem_y + delta_origem_y
	
	# Transladar ponto para eixo de rotacao
	new_t_x = new_x - new_origem_x
	new_t_y = new_y - new_origem_y
	
	# Rotaciona ponto
	new_r_x, new_r_y = rotate_point(new_t_x, new_t_y, angle)
	
	#print(1, x, y)
	#print(2, origem_x, origem_y)
	#print(3, delta_y,delta_origem_y)
	#print(4, new_x, new_y)
	#print(5, new_origem_x, new_origem_y)
	#print(6, new_t_x, new_t_y)
	#print(7, new_r_x, new_r_y)
	
	# Transladar para o eixo original
	new_r_x += new_origem_x
	new_r_y += new_origem_y
	
	#print(8, new_r_x, new_r_y)
	
	# Inverte eixo y
	new_r_y = h - new_r_y
	
	#print(9, new_r_x, new_r_y)
	
	delta_r_x = new_r_x - x
	delta_r_y = new_r_y - y
	
	#print(10, delta_r_x, delta_r_y)

	return new_r_x, new_r_y

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


def main(labels_path, images_path, image_width, image_height, angle):
	#print("Parametros:\n")
	#print("labels_path: ", labels_path, "\n")
	#print("images_path: ", images_path, "\n")
	#print("image_width: ", image_width, "\n")
	#print("image_height: ", image_height, "\n")
	#print("rotation: ", rotation, "\n")
	label_file_list = glob(labels_path + '*')
	if not label_file_list:
		print("\nERROR: No labels files found: " + labels_path + "\n")
		return 1

	for label_file in sorted(label_file_list, cmp = file_name_compare):
		if check_file_name(os.path.basename(label_file)) < 0:
			print('ERROR: FILE NAME FORMAT: ' + label_file)
			continue

		print(label_file)
		img_file = get_image_file_name(label_file, labels_path, images_path, 'png')
		if not os.path.isfile(img_file):
			print('ERROR: FILE NOT FOUND: ' + img_file)
			img = np.full((image_height, image_width, 3), (255,255,255), np.uint8)
		else:
			new_labels_path = os.path.dirname(label_file) + "_angle_" + str(angle) + "/"
			if not os.path.exists(new_labels_path):
				os.mkdir(new_labels_path)
			new_label_file = new_labels_path + os.path.basename(label_file)	
			if os.path.isfile(new_label_file):
				os.remove(new_label_file)
			print(new_label_file)
		
			
			new_images_path = os.path.dirname(img_file) + "_angle_" + str(angle) + "/"
			if not os.path.exists(new_images_path):
				os.mkdir(new_images_path)
			new_img_file = new_images_path + os.path.basename(img_file)
			if os.path.isfile(new_img_file):
				os.remove(new_img_file)
			
			img = cv2.imread(img_file)
			new_img, padding_x, padding_y = rotate_image(img, angle)
			new_cut_img = rotate_image_v2(img, angle) 
			cv2.imwrite(new_img_file, new_img)
			new_img = cv2.imread(new_img_file)
		
		i = 0
		f = open(label_file)
		
		# Each line of the file contains the coordinates of one predicted bounding box
		# Line format: class x y w h
		#   Field 0: object class
		#   Field 1: x coordinate of the bounding box's center (in a fraction of image_width)
		#   Field 2: y coordinate of the bounding box's center (in a fraction of image_height)
		#   Field 3: the bounding box's width  (in a fraction of image_width)
		#   Field 4: the bounding box's height (in a fraction of image_height)
		
		f = open(label_file)
		new_f = open(new_label_file, "w")
		
		for line in f:
			bbox = line.split()
			if len(bbox) == 5:
				obj_class = bbox[0]
				x, y, w, h = [ float(data) for data in bbox[1:] ]
				print(x,y,w,h)
				x_left   = int((x - (w / 2)) * image_width)
				x_right  = int((x + (w / 2)) * image_width)
				y_top    = int((y - (h / 2)) * image_height)
				y_bottom = int((y + (h / 2)) * image_height)
				
				print('[%d] - ORIGINAL - x_left=[%d] - y_top=[%d] - x_right=[%d] - y_bottom=[%d]' %(i, x_left, y_top, x_right, y_bottom))
				cv2.rectangle(img, (x_left, y_top), (x_right, y_bottom), (255,0,0), 2)
				cv2.putText(img, obj_class + ' [%d]' % i, ((x_left + 3), (y_top - 6)), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,0], 1);
				
				#origem_x = x_left
				#origem_y = y_bottom
				
				h, w = img.shape[:2]
				
				origem_x = int(w / 2)
				origem_y = int(h / 2)				
				
				p1_new_x, p1_new_y = rotate(x_left, y_bottom, angle, origem_x, origem_y, w, h)
				p2_new_x, p2_new_y = rotate(x_left, y_top, angle, origem_x, origem_y, w, h)
				p3_new_x, p3_new_y = rotate(x_right, y_top, angle, origem_x, origem_y, w, h)
				p4_new_x, p4_new_y = rotate(x_right, y_bottom, angle, origem_x, origem_y, w, h)
				
				cv2.line(new_cut_img, (p1_new_x, p1_new_y), (p2_new_x, p2_new_y), (0,255,0), 2)
				cv2.line(new_cut_img, (p2_new_x, p2_new_y), (p3_new_x, p3_new_y), (0,255,0), 2)
				cv2.line(new_cut_img, (p3_new_x, p3_new_y), (p4_new_x, p4_new_y), (0,255,0), 2)
				cv2.line(new_cut_img, (p4_new_x, p4_new_y), (p1_new_x, p1_new_y), (0,255,0), 2)
				
				new_f.write(str(w) + "x" + str(h) + " >" + " p1=(" + str(p1_new_x) + "," + str(p1_new_y) + ")" + " p2=(" + str(p2_new_x) + "," + str(p2_new_y) + ")" + " p3=(" + str(p3_new_x) + "," + str(p3_new_y) + ")" + " p4=(" + str(p4_new_x) + "," + str(p4_new_y) + ")" + "\n")
				
				x_left   = x_left + padding_x
				y_top    = y_top + padding_y
				x_right  = x_right + padding_x
				y_bottom = y_bottom + padding_y
				h, w = new_img.shape[:2]
								
				origem_x = int(w / 2)
				origem_y = int(h / 2)				
				
				p1_new_x, p1_new_y = rotate(x_left, y_bottom, angle, origem_x, origem_y, w, h)
				p2_new_x, p2_new_y = rotate(x_left, y_top, angle, origem_x, origem_y, w, h)
				p3_new_x, p3_new_y = rotate(x_right, y_top, angle, origem_x, origem_y, w, h)
				p4_new_x, p4_new_y = rotate(x_right, y_bottom, angle, origem_x, origem_y, w, h)
				
				cv2.line(new_img, (p1_new_x, p1_new_y), (p2_new_x, p2_new_y), (0,255,0), 2)
				cv2.line(new_img, (p2_new_x, p2_new_y), (p3_new_x, p3_new_y), (0,255,0), 2)
				cv2.line(new_img, (p3_new_x, p3_new_y), (p4_new_x, p4_new_y), (0,255,0), 2)
				cv2.line(new_img, (p4_new_x, p4_new_y), (p1_new_x, p1_new_y), (0,255,0), 2)
								
				new_f.write(str(w) + "x" + str(h) + " >" + " p1=(" + str(p1_new_x) + "," + str(p1_new_y) + ")" + " p2=(" + str(p2_new_x) + "," + str(p2_new_y) + ")" + " p3=(" + str(p3_new_x) + "," + str(p3_new_y) + ")" + " p4=(" + str(p4_new_x) + "," + str(p4_new_y) + ")" + "\n")
				
				i += 1
			elif bbox:
				print('ERROR: Invalid prediction file record: ' + line)
			
		f.close()
		
		new_f.close()
		
		while True:
			cv2.imshow('Original Image ' + str(img.shape[:2][1]) + "x" + str(img.shape[:2][0]), img)
			cv2.imshow('Cut Rotated Image ' + str(new_cut_img.shape[:2][1]) + "x" + str(new_cut_img.shape[:2][0]), new_cut_img)
			cv2.imshow('Rotated Image ' + str(new_img.shape[:2][1]) + "x" + str(new_img.shape[:2][0]), new_img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:    # Enter key
				break
			if key == 27:    # ESC key
				return 0
	return 0


if __name__ == "__main__":
	if len(sys.argv) == 5:
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
			elif main(sys.argv[1], sys.argv[2], int(s[0]), int(s[1]), int(sys.argv[4])) == 0:
				sys.exit()
			
	print("\nUsage: python " + sys.argv[0] + " labels_path images_path image_size(wxh)\n")
	print("Example: python " + sys.argv[0] + ' "/lane_dataset/yolo/*/labels/" "/lane_dataset/gt/*/images/" 640x480\n')
	print("Note: label lines must be in format: class x y w h (in fractions)\n")
