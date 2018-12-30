import sys
import os
import cv2


def main(gt_path, images_path):
	for gt_file in os.listdir(gt_path):
		img_file = images_path + gt_file.replace('.txt', '.png')
		if not os.path.isfile(img_file):
			continue
		
		img = cv2.imread(img_file)
		gt = open(gt_path + gt_file, "r")
		
		i = 0
		for line in gt:
			if len(line) < 3:
				continue
			i += 1
			x,y = [ int(data) for data in line.split() ]

			cv2.circle(img, (x, y), 5, (255,0,0), -1)
			cv2.putText(img, str(i), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, [0,255,0], 1);

		gt.close()
		while (1):
			cv2.imshow('GT View', img)
			key = cv2.waitKey(0) & 0xff
			
			if key == 10:      # Enter key
				break
			elif key == 27:    # ESC key
				return


if __name__ == "__main__":
	if len(sys.argv) != 3:
		print("\nUsage: python " + sys.argv[0] + " gt_path images_path\n")
		exit(1)

	if not sys.argv[1].endswith('/'):
		sys.argv[1] += '/'
	if not sys.argv[2].endswith('/'):
		sys.argv[2] += '/'
		
	main(sys.argv[1], sys.argv[2])
