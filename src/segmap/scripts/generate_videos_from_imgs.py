import os
from multiprocessing import Pool

imgs_dir = '/dados/localizer_imgs/'

def generate_video(data_path):
    global imgs_dir
    cmd = "ffmpeg -framerate 25 -pattern_type glob -i '%s/%s/*.png' -c:v libx264 -r 30 -crf 20 -pix_fmt yuv420p video_localizer_%s.mp4"
    cmd = cmd % (imgs_dir, data_path, data_path)
    os.system(cmd)

dirs = os.listdir(imgs_dir)
process_pool = Pool(len(dirs)) 
process_pool.map(generate_video, dirs)
