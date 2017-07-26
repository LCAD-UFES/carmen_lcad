import cv2
import sys

if len(sys.argv) < 2:
    print "Usage:\n", sys.argv[0],'<file.png> ...'
    exit(0)

for i in range(1, len(sys.argv)):
    img = cv2.imread(sys.argv[i], cv2.WINDOW_NORMAL)
    cv2.imshow(sys.argv[i], img)
    txt = open(sys.argv[i][:-3] + 'txt', 'w')
    for y in range(len(img)):
        for x in range(len(img[y])):
            txt.write(str(img[y][x]) + '\t')
        txt.write('\n')
    txt.close()
    cv2.waitKey(0)
