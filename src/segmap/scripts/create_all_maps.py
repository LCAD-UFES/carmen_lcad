
import os
from multiprocessing import Pool


def run_command(cmd):
    os.system(cmd)


def create_maps(experiments):
    cmds = ["./mapper " + e['map'] for e in experiments]
    process_pool = Pool(len(cmds)) 
    process_pool.map(run_command, cmds)


if __name__ == "__main__":
    experiments = [
        {'map': "data_log_volta_da_ufes-20180112.txt", 
         'test': ["data_log_volta_da_ufes-20180112-2.txt", "data_log_volta_da_ufes-20180907-2.txt", 
                  "data_log-volta-da-ufes-20181206.txt", "data_log-volta-da-ufes-noite-20181130.txt"]},
        {'map': "data_log_dante_michelini-20181116.txt", 'test': ["data_log_dante_michelini-20181116-pista-esquerda.txt"]},
        {'map': 'data_log_aeroporto_vila_velha_20170726-2.txt', 'test': ['data_log_aeroporto_vila_velha_20170726.txt']},
        {'map': 'data_log-estacionamento-ambiental-20181208.txt', 
         'test': ['data_log-volta-da-ufes-20181207-estacionamento_ambiental.txt', 'data_log-volta-da-ufes-20181206-estacionamento-test.txt']}, 
        {'map': 'data_log-jardim-da-penha-mapeamento-20181208.txt', 'test': ['data_log-jardim_da_penha-20181207-2.txt', 'data_log-jardim_da_penha-20181207.txt']}, 
        {'map': 'data_log_estacionamentos-20181130.txt', 'test': ['data_log-volta-da-ufes-20181206-honofre-test.txt', 'data_log_estacionamentos-20181130-test.txt']},
        {'map': 'data_log_sao_paulo_brt_20170827.txt', 'test': ['data_log_sao_paulo_brt_20170827-2.txt']}
    ]

    create_maps(experiments)
