import os
import numpy as np


experiments = [
        {'map': "log_volta_da_ufes-20180112.txt", 'test': ["log-volta-da-ufes-20181206.txt", "log-volta-da-ufes-noite-20181130.txt"]},
        {'map': 'log_aeroporto_vila_velha_20170726-2.txt', 'test': ['log_aeroporto_vila_velha_20170726.txt']},
        {'map': 'log-jardim-da-penha-mapeamento-20181208.txt', 'test': ['log-jardim_da_penha-20181207.txt']}, 
        {'map': "log_dante_michelini-20181116.txt", 'test': ["log_dante_michelini-20181116-pista-esquerda.txt"]},
#        {'map': 'log-estacionamento-ambiental-20181208.txt', 'test': [
#        	'log-volta-da-ufes-20181207-estacionamento_ambiental.txt', 
        	#'log-volta-da-ufes-20181206-estacionamento-test.txt'
#        ]}, 
#        {'map': 'log_estacionamento_honofre-20190601-mapeamento.txt', 'test': [
        	#'log-volta-da-ufes-20181206-honofre-test.txt', 
#        	'log_estacionamento-honofre-20181130-test.txt'
#        ]},
#        {'map': 'log_sao_paulo_brt_20170827.txt', 'test': ['log_sao_paulo_brt_20170827-2.txt']},
    ]


def read_poses(log_name, poses_file):
	path = '/dados/data2/data_' + log_name + '/' + poses_file
	with open(path, 'r') as f:
		lines = [l.rstrip().rsplit() for l in f.readlines()]
	lines = np.array(lines).astype(np.float)
	return lines


def find_nearest_pose(ref, poses):
	return np.argmin([(ref[1] - p[1]) ** 2 + (ref[2] - p[2]) ** 2 for p in poses])


def find_nearest_time(ref, times):
	arg = np.argmin([abs(ref[4] - t) for t in times])
	return times[arg]


def save_img(iter_id, time, log_name):
	cmd = "cp /dados/data/data_%s/semantic/result_%lf-r.png seg_imgs/%03d-%s_%lf.png" % (log_name, time, iter_id, log_name, time)
	print(cmd)
	os.system(cmd)

def get_image_times(log_name):
	return [float(f.replace("result_", "").replace("-r.png", "")) for f in os.listdir('/dados/data/data_%s/semantic/' % log_name)]


for e in experiments:
	m = e['map']
	t = e['test']
	
	m_poses = read_poses(m, 'graphslam.txt')
	m_times = get_image_times(m)
	t_poses = [read_poses(tt, 'graphslam_to_map.txt') for tt in t]
	# result_1542364066.625743-r.png
	t_times = [get_image_times(tt) for tt in t]
	
	step = int(len(m_poses) / 20.0)
	
	for i in range(0, len(m_poses), step):
		ref = m_poses[i]
		tm = find_nearest_time(ref, m_times)
		save_img(i, tm, m)
		for j in range(len(t_poses)):
			tt = t_poses[j]
			ln = t[j]
			nn = find_nearest_pose(ref, tt)
			#print(ref, tt[nn])
			tm = find_nearest_time(tt[nn], t_times[j])
			save_img(i, tm, ln)
			#input()
			
	print(m_poses)
	print(t_poses)
	
