import cv2
import os

input_folder = '/media/gabriel/SeagateExpansionDrive/pngs_log_20180907-2'
output_folder = '/media/gabriel/SeagateExpansionDrive/resized_pngs_log_20180907-2'

img_list = os.listdir(input_folder)

for name in img_list:
    img = cv2.imread(os.path.join(input_folder, name), cv2.IMREAD_COLOR)
    # print(img.shape)
    new_img = cv2.resize(img, (640,480))
    cv2.imwrite(os.path.join(output_folder, name), new_img)
