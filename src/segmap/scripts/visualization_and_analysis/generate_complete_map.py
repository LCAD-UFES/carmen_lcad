
import os
import cv2
import numpy as np
from multiprocessing import Pool


def save_complete_map(dataset):
    d = '/dados/maps/map_' + dataset 
    
    origins = np.array([f.replace('.png', '').rsplit('_')[1:3] for f in os.listdir(d) if f[-3:] == 'png']).astype(np.float)
    min_w, max_w = np.min(origins[:, 0]), np.max(origins[:, 0])
    min_h, max_h = np.min(origins[:, 1]), np.max(origins[:, 1])

    concated = []
    for h in np.arange(min_h, max_h+50, 50):
        imgs = []
        for w in np.arange(min_w, max_w+50, 50):
            name = d + '/semantic_%f_%f.png' % (w, h)
            if os.path.exists(name):
                imgs.append(cv2.imread(name))
            else:
                imgs.append(np.ones((250, 250, 3)) * 128)
        concated.append(np.concatenate(imgs, axis=1))
    cv2.imwrite('img_map_%s.png' % dataset, np.concatenate(concated, axis=0))


if __name__ == "__main__":
    experiments = [
        {'map': 'data_log_sao_paulo_brt_20170827.txt', 'test': ['data_log_sao_paulo_brt_20170827-2.txt']},
        {'map': "data_log_volta_da_ufes-20180112.txt", 
        'test': ["data_log_volta_da_ufes-20180112-2.txt", "data_log_volta_da_ufes-20180907-2.txt", 
                  "data_log-volta-da-ufes-20181206.txt", "data_log-volta-da-ufes-noite-20181130.txt"]},
        {'map': "data_log_dante_michelini-20181116.txt", 'test': ["data_log_dante_michelini-20181116-pista-esquerda.txt"]},
        {'map': 'data_log_aeroporto_vila_velha_20170726-2.txt', 'test': ['data_log_aeroporto_vila_velha_20170726.txt']},
        {'map': 'data_log-estacionamento-ambiental-20181208.txt', 
         'test': ['data_log-volta-da-ufes-20181207-estacionamento_ambiental.txt', 'data_log-volta-da-ufes-20181206-estacionamento-test.txt']}, 
        {'map': 'data_log-jardim-da-penha-mapeamento-20181208.txt', 'test': ['data_log-jardim_da_penha-20181207-2.txt', 'data_log-jardim_da_penha-20181207.txt']}, 
        {'map': 'data_log_estacionamentos-20181130.txt', 'test': ['data_log-volta-da-ufes-20181206-honofre-test.txt', 'data_log_estacionamentos-20181130-test.txt']},
    ]

    mapping_datasets = ['data_log-estacionamento-ambiental-20181208.txt']
    #mapping_datasets = [x['map'] for x in experiments]
    #print('Mapping datasets:', mapping_datasets)
    process_pool = Pool(len(mapping_datasets)) 
    process_pool.map(save_complete_map, mapping_datasets)



