import sys
import os
import cv2
import numpy as np
from glob import glob
from numpy import string_
import shutil

def flip_horizontal_image(original_image):
	# Perform flip
	rotated_image = cv2.flip(original_image, 1)	
	
	return rotated_image 

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


def main(labels_path, images_path, image_width, image_height, flip):
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
			#print(img_file)
			#img = cv2.imread(img_file)
			
			new_labels_path = os.path.dirname(label_file) + "_flip_" + str(flip) + "/"
			#print(new_labels_path)
			if not os.path.exists(new_labels_path):
				os.mkdir(new_labels_path)
			new_label_file = new_labels_path + os.path.basename(label_file)	
			if os.path.isfile(new_label_file):
				os.remove(new_label_file)
			#shutil.copyfile(label_file, new_label_file)
			print(new_label_file)
		
			
			new_images_path = os.path.dirname(img_file) + "_flip_" + str(flip) + "/"
			#print(new_images_path)
			if not os.path.exists(new_images_path):
				os.mkdir(new_images_path)
			new_img_file = new_images_path + os.path.basename(img_file)
			if os.path.isfile(new_img_file):
				os.remove(new_img_file)
			#shutil.copyfile(img_file, new_img_file)
			
			img = cv2.imread(img_file)
			#new_img = rotate_image(img, rotation)
			#new_img = img
			new_img = flip_horizontal_image(img) 
			cv2.imwrite(new_img_file, new_img)
			#img = cv2.imread(new_img_file)
			#cv2.imshow('Rotate Image', new_img)						
		
		i = 0
		f = open(label_file)
		new_f = open(new_label_file, "w")
		
		# Each line of the file contains the coordinates of one predicted bounding box
		# Line format: class x y w h
		#   Field 0: object class
		#   Field 1: x coordinate of the bounding box's center (in a fraction of image_width)
		#   Field 2: y coordinate of the bounding box's center (in a fraction of image_height)
		#   Field 3: the bounding box's width  (in a fraction of image_width)
		#   Field 4: the bounding box's height (in a fraction of image_height)
		
		for line in f:
			bbox = line.split()
			if len(bbox) == 5:
				obj_class = bbox[0]
				x, y, w, h = [ float(data) for data in bbox[1:] ]
				x_left   = int((x - (w / 2)) * image_width)
				x_right  = int((x + (w / 2)) * image_width)
				y_top    = int((y - (h / 2)) * image_height)
				y_bottom = int((y + (h / 2)) * image_height)
				
				new_x_left   = image_width - x_right 
				new_x_right  = image_width - x_left 
				
				new_x = (new_x_left / float(image_width)) + float(w / 2)  #=(E2/640)+(C2/2)
				#new_y = (new_x_left / image_width) + (w / 2)
				
				#print(str(i) + " " + str(x_left) + " " + str(x))
				#print(str(i) + " " + str(new_x_left) + " " + str(new_x) + " " + str(image_width) + " " + str(new_x_left / float(image_width)) + " " + str(w) + " " + str(w//2))
				#print(str(i) + " " + str(x_right) + " " + str(x))
				#print(str(i) + " " + str(new_x_right) + " " + str(new_x))
								
				cv2.rectangle(new_img, (new_x_left, y_top), (new_x_right, y_bottom), (255,0,0), 2)
				cv2.putText(new_img, obj_class + ' [%d]' % i, ((new_x_left + 3), (y_top - 6)), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,0], 1);
				
				new_f.write("0" + " " + str(new_x) + " " + str(y) + " " + str(w) + " " + str(h) + "\n")
				
				i += 1
			elif bbox:
				print('ERROR: Invalid prediction file record: ' + line)
			
		f.close()
		
		new_f.close()

		while True:
			cv2.imshow('Rotate Image', new_img)
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
