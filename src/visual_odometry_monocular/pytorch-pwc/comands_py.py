import os
import numpy as np


list_images = os.listdir("/media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/pngs_mrs")
#os.chdir("/home/lume/carmen_lcad/src/utilities/convert_log_images/saida_png")
list_images.sort()

for x in range(267, len(list_images) -1):
    print(x)
    """print("python3 predict_new_data.py saida_flow  MaskFlownet.yaml --image_1 /home/lume/carmen_lcad/src/utilities/convert_log_images/saida_png/" + list_images[x] + " \
        --image_2 /home/lume/carmen_lcad/src/utilities/convert_log_images/saida_png/" + list_images[x + 1] + " -g 0 -c 8caNov12")
    os.system("python3 predict_new_data.py saida_flow  MaskFlownet.yaml --image_1 /home/lume/carmen_lcad/src/utilities/convert_log_images/saida_png/" + list_images[x] + " \
        --image_2 /home/lume/carmen_lcad/src/utilities/convert_log_images/saida_png/" + list_images[x + 1] + " -g 0 -c 8caNov12")"""
    print("python3 run.py --model default --one /media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/pngs_mrs/" + list_images[x] + \
         " --two /media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/pngs_mrs/" + list_images[x + 1] + " --out ./flow_mrs/" + "{:06}".format(x) + ".flo")
    os.system("python3 run.py --model default --one /media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/pngs_mrs/" + list_images[x] + \
         " --two /media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/pngs_mrs/" + list_images[x + 1] + " --out ./flow_mrs/" + "{:06}".format(x) + ".flo")
