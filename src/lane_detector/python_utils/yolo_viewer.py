import sys
import os
import cv2


#def replace_format(path):
	#i = len(path)
	#while (i > 0):
		#if path[i] == '.'
		#path[i + 1] = 't'
		#path[i + 2] = 'x'
		#path[i + 3] = 't'
		


def main(labels_path, images_path, image_dimensions):
	for label_file in os.listdir(labels_path):
		f = open(labels_path + label_file, "r")
		print label_file
		print images_path + label_file.replace('txt', 'png')
		img = cv2.imread(images_path + label_file.replace('txt', 'png'))
		
		for line in f:
			if len(line) < 10:
				continue
			
			box = line.strip().split()
			x1 = int((float(box[1]) - (float(box[3]) / 2)) * image_dimensions[0])
			y1 = int((float(box[2]) - (float(box[4]) / 2)) * image_dimensions[1])
			x2 = int((float(box[1]) + (float(box[3]) / 2)) * image_dimensions[0])
			y2 = int((float(box[2]) + (float(box[4]) / 2)) * image_dimensions[1])

			#print str(x1) + ' ' +  str(y1) + ' ' + str(x2) + ' ' + str(y2)
			#cv2.putText(img, line[0], (x1, y + t_size[1]), cv2.FONT_HERSHEY_PLAIN, 1, [225,255,255], 1);
			cv2.rectangle(img, (x1, y1), (x2, y2), (255,0,0), 2)

		while (1):
			cv2.imshow('Yolo View', img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:      # Enter key
				break
			elif key == 27:    # ESC key
				return


if __name__ == "__main__":
	if len(sys.argv) > 3 and len(sys.argv) < 8:
		if not sys.argv[1].endswith('/'):
			sys.argv[1] += '/'
		if not sys.argv[2].endswith('/'):
			sys.argv[2] += '/'
			
		s = sys.argv[3].rsplit('x')
		image_dimensions = [int(s[0]), int(s[1])]
		
		main(sys.argv[1], sys.argv[2], image_dimensions)
	else:
		print("\nusage: python labels/path/ images/path/ image_width image_heigth -format jpg\n")