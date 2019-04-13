
import os
import sys


experiments = [
        {'map': "log_volta_da_ufes-20180112.txt", 
        'test': ["log-volta-da-ufes-20181206.txt", 
                 #"log_volta_da_ufes-20180112-2.txt", 
                 #"log_volta_da_ufes-20180907-2.txt", 
                 #"log-volta-da-ufes-noite-20181130.txt"
                 ]},
        {'map': 'log_aeroporto_vila_velha_20170726-2.txt', 'test': ['log_aeroporto_vila_velha_20170726.txt']},
        #{'map': 'log-jardim-da-penha-mapeamento-20181208.txt', 
            #'test': [ #'log-jardim_da_penha-20181207-2.txt', 
                     #'log-jardim_da_penha-20181207.txt']}, 
        
        #{'map': "log_dante_michelini-20181116.txt", 'test': ["log_dante_michelini-20181116-pista-esquerda.txt"]},
        #{'map': 'log-estacionamento-ambiental-20181208.txt', 
         #'test': ['log-volta-da-ufes-20181207-estacionamento_ambiental.txt', 'log-volta-da-ufes-20181206-estacionamento-test.txt']}, 
        #{'map': 'log_estacionamentos-20181130.txt', 'test': ['log-volta-da-ufes-20181206-honofre-test.txt', 'log_estacionamentos-20181130-test.txt']},
        #{'map': 'log_sao_paulo_brt_20170827.txt', 'test': ['log_sao_paulo_brt_20170827-2.txt']},
    ]
    

DATA_DIR = "/dados/data2/"
GPS_XY_STD = 2.0
GPS_H_STD = 10.0
GICP_LOOPS_XY_STD = 0.3
GICP_LOOPS_H_STD = 1.0
PF_LOOPS_XY_STD = 0.02
PF_LOOPS_H_STD = 3.0
#IGNORE_POINTS_ABOVE = 0.0
#IGNORE_POINTS_BELOW = -1.5
#IGNORE_POINTS_ABOVE = -0.5
#IGNORE_POINTS_BELOW = -100
SKIP_WHEN_VELOCITY_IS_BELOW = 1.0

def create_output_dir(log_path):
    log_name = log_path.rsplit("/")[-1]
    output_dir = DATA_DIR + "/data_" + log_name + "/"
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    return output_dir


def run_command(cmd):
    print('\n\n--------------\n')
    print(cmd)
    ret = os.system(cmd)
    
    # graphslam gives a segfault in the end
    if ('graphslam' not in cmd) and (ret != 0):
        print("Error: Failed to execute command.")
        sys.exit(0)
