import os, sys


def train_model(txt_rddf, image_folder):
    dados_rddf = []
    if('/' == image_folder[-1]):
        image_folder = image_folder[:-1]
    with open(txt_rddf, 'r') as f:
        dados_rddf = [line.strip().split(" ") for line in f]

    for i in range(len(dados_rddf)):
        if(False == os.path.isfile(image_folder + '/' + dados_rddf[i][5] + '-r.png')):
            print(image_folder + '/' + dados_rddf[i][5] + '-r.png')
    
    return  dados_rddf, image_folder

#train_model("/dados/rddf_predict/te.txt", "/dados/log_png_1003")
if(len(sys.argv)<3):
    train_model('/dados/rddf_predict/listen_2019-11-29_11:32:36', '/dados/log_png_1003/')    
else:
    train_model(sys.argv[1], sys.argv[2])
