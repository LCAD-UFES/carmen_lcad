
import glob, os

paths = ['/dados/data/data_log_aeroporto_vila_velha_20170726-2.txt/semantic/', '/dados/data/data_log_aeroporto_vila_velha_20170726.txt/semantic/',
    '/dados/data/data_log_sao_paulo_brt_20170827.txt/semantic/', '/dados/data/data_log_volta_da_ufes-20180112-2.txt/semantic/',  
    '/dados/data/data_log_volta_da_ufes-20180112.txt/semantic/', '/dados/data/data_log_volta_da_ufes-20180907-2.txt/semantic/']

for path in paths:
    print(path)
    imgs_path = glob.glob(path + "seg*png")
    for img_path in imgs_path:
        cmd = "mv " + img_path + " " + img_path.replace("seg_", "")
        os.system(cmd)

