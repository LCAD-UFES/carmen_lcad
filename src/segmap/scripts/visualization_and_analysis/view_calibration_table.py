
import numpy as np
import sys
import cv2

def read_calibration_table(path):
    table = np.zeros((32, 256))
    
    for j in range(256):
        table[:, j] = j
    
    data = [l.rstrip().rsplit() for l in open(path, 'r').readlines()]
    data = np.array(data).astype('float')
    
    return data


def draw_tile(img, row, column, color, height, width):
    left_top = (column * width, row * height)
    right_bottom = ((column + 1) * width, (row + 1) * height)
    # content
    cv2.rectangle(img, left_top, right_bottom, (color, color, color), -1)
    # border
    # cv2.rectangle(img, left_top, right_bottom, (255, 255, 255), 1)


def create_calibration_table_image(table):
    TILE_HEIGHT = 20
    TILE_WIDTH = 10
    
    img = np.zeros((33 * TILE_HEIGHT, 256 * TILE_WIDTH, 3)).astype(np.uint8)

    for i in range(33):
        for j in range(256):
            draw_tile(img, i, j, j, TILE_HEIGHT, TILE_WIDTH)

    for sample in table:
        laser = int(sample[0])
        raw_intensity = int(sample[1])
        count = int(sample[-2])
        calibrated_intensity = int(sample[-1])
        
        if (count > 10):
            draw_tile(img, laser + 1, raw_intensity, calibrated_intensity, TILE_HEIGHT, TILE_WIDTH)
        else:
            draw_tile(img, laser + 1, raw_intensity, 0, TILE_HEIGHT, TILE_WIDTH)

    return img


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("\nUse python %s <path calibration table>\n" % sys.argv[0])
    else:
        table = read_calibration_table(sys.argv[1])
        img = create_calibration_table_image(table)        
        cv2.imwrite("calibration_table.png", img)
        print("Saved 'calibration_table.png'")
        cv2.imshow("calibration table", img)
        cv2.waitKey(-1)

