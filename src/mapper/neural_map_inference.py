import os
import sys
import torch
import model as M
import numpy as np
from PIL import Image
import cv2
from torchvision import datasets, transforms
from posix import wait
from cv2 import waitKey


device = torch.device("cuda:0")
carmen_home = os.getenv("CARMEN_HOME")
saved_path = carmen_home + '/src/neural_mapper/pytorch_neural_mapper/saved_models/69imgs_epoch17.model'
images_converted = 0
model = M.FCNN(n_output=3)
model.load_state_dict(torch.load(saved_path))
model = model.eval()

def png2tensor(img):
    img2tensor = transforms.ToTensor()
    return img2tensor(img)

def tensor2png(tensor):
    trans = transforms.ToPILImage()
    return trans(tensor)


def save_to_debug(data, imgPred):
    print("aqui foi\n")
    imgData = data[0][3].numpy()
    img2 = (imgData)*255/3
    img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    img_map = np.zeros((600,600,3), np.uint8)
    img_map[np.where(((img2 != [0])).all(axis = 2))] = np.array([0,255,0])
    imPredShow = tensor2rgbimage(imgPred)
    imgs_comb = np.hstack((img_map, imPredShow))
    cv2.imwrite(str(images_converted) + '_Combined.png',  imgs_comb)
    

def tensor2rgbimage(tensor):
    img = tensor.permute(1,2,0).numpy()
    height, width = img.shape[:2]
    img_map = np.zeros((width,height,3), np.uint8)
    #print(np.where((img == [1]).all(axis = 2)))
    img_map[np.where((img == [0]).all(axis = 2))] = np.array([255,120,0])
    img_map[np.where((img == [1]).all(axis = 2))] = np.array([255,255,255])
    img_map[np.where((img == [2]).all(axis = 2))] = np.array([0,0,0])

    return img_map

def load_data2(batch_size, batch_idx, dataset_list, data_path, target_path):
    batch_weight = np.zeros(n_classes)
    dataset_size = len(dataset_list)
    n = math.floor(dataset_size/batch_size)
    data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
    target = torch.zeros(batch_size, img_x_dim, img_y_dim, dtype=torch.int64)

    for j in range(batch_size):
        # + 1 se indice comeca em 1 
#         print(batch_idx*batch_size + j)
        data[j][0] = png2tensor(data_path + str(dataset_list[batch_idx*batch_size + j]) + '_max.png')[0]# + 1) + '_max.png')
        data[j][1] = png2tensor(data_path + str(dataset_list[batch_idx*batch_size + j]) + '_mean.png')[0]# + 1) + '_mean.png')
        data[j][2] = png2tensor(data_path + str(dataset_list[batch_idx*batch_size + j]) + '_min.png')[0]# + 1) + '_min.png')
        data[j][3] = png2tensor(data_path + str(dataset_list[batch_idx*batch_size + j]) + '_numb.png')[0]# + 1) + '_numb.png')
        data[j][4] = png2tensor(data_path + str(dataset_list[batch_idx*batch_size + j]) + '_std.png')[0]# + 1) + '_std.png')
        tmp, new_weights = png2target(target_path + str(dataset_list[batch_idx*batch_size + j]) + '_label.png')
        target[j] = tmp[0]
        batch_weight = batch_weight + new_weights
    max_weight = max(batch_weight)
    batch_weight = max_weight/batch_weight
    return data, target , batch_weight


def process_image(carmen_image1, carmen_image2, carmen_image3, carmen_image4, carmen_image5):
    data = torch.zeros(1, 5, 600, 600) #dimensoes do tensor que usamos
    # quebrar em 5 imagens
    # img_teste = Image.fromarray(carmen_image)
    # data = torch.from_numpy(carmen_image).float().to('cuda:0')
    print('AMEM! \n')
    print(carmen_image1.shape)

    #cv2.imshow("img1", carmen_image1)
    #cv2.waitKey(1)
    
    img1 = Image.fromarray(carmen_image1)
    img2 = Image.fromarray(carmen_image2)
    img3 = Image.fromarray(carmen_image3)
    img4 = Image.fromarray(carmen_image4)
    img5 = Image.fromarray(carmen_image5)
    
    #img1.show("img1_bla")
    
#     img.save("teste.png")
#      img2tensor(img)img2tensor = transforms.ToTensor()
#     np.asarray(carmen_image)
#     cv2.waitKey(0)
    print("Converteu to PIL!")
    sys.stdout.flush()

    data[0][0] = png2tensor(img1)[0] #max
    data[0][1] = png2tensor(img2)[0] #mean
    data[0][2] = png2tensor(img3)[0] #min
    data[0][3] = png2tensor(img4)[0] #numb
    data[0][4] = png2tensor(img5)[0] #std
    print('Converteu! \n')
    output = model(data)
    ##Tratar depois as probabilidades certinho
    pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
    imgPred = pred[0]
    # imgPred = (imgPred + 1)*255/3
    imgPred = imgPred.cpu().float()
    mapper = imgPred.permute(1,2,0).numpy()
    mapper = mapper.astype(np.uint8)
    save_to_debug(data, imgPred)
    
    return mapper

################Para Debug
# if __name__ == '__main__':
#   c1 = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/circle_NOTACUMULATED/teste/data/1323_max.png'
#   c2 = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/circle_NOTACUMULATED/teste/data/1323_mean.png'
#   c3 = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/circle_NOTACUMULATED/teste/data/1323_min.png'
#   c4 = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/circle_NOTACUMULATED/teste/data/1323_numb.png'
#   c5 = '/media/vinicius/NewHD/Datasets/Neural_Mapper_dataset/circle_NOTACUMULATED/teste/data/1323_std.png'
#   
#   img = process_image(c1,c2,c3,c4,c5)
# #   # print(img)
#   height, width = img.shape[:2]
#   img_map = np.zeros((width,height,3), np.uint8)
#   img_map[np.where((img == [0]).all(axis = 2))] = np.array([255,120,0])
#   img_map[np.where((img == [1]).all(axis = 2))] = np.array([0,0,0])
#   img_map[np.where((img == [2]).all(axis = 2))] = np.array([255,255,255])
# 
#   cv2.imshow("Janela", img_map)

#   cv2.waitKey(0)












