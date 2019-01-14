import sys
import os
import cv2
import numpy as np
from glob import glob


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


def main(labels_path, images_path, image_width, image_height):
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
			print(img_file)
			img = cv2.imread(img_file)

		i = 0
		f = open(label_file)
		
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
				
				cv2.rectangle(img, (x_left, y_top), (x_right, y_bottom), (255,0,0), 2)
				cv2.putText(img, obj_class + ' [%d]' % i, ((x_left + 3), (y_top - 6)), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,0], 1);
				i += 1
			elif bbox:
				print('ERROR: Invalid prediction file record: ' + line)
			
		f.close()

		while True:
			cv2.imshow('Yolo View', img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:    # Enter key
				break
			if key == 27:    # ESC key
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
			
	print("\nUsage: python " + sys.argv[0] + " labels_path images_path image_size(wxh)\n")
	print("Example: python " + sys.argv[0] + ' "/lane_dataset/yolo/*/labels/" "/lane_dataset/gt/*/images/" 640x480\n')
	print("Note: label lines must be in format: class x y w h (in fractions)\n")
