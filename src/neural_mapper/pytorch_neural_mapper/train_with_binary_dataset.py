#
# Main script for training a model for semantic segmentation of
# roads from Velodyne point clouds into Occupancy Grid Map
#
# Use the config.ini file to set the variables
# Requirements: pytorch
#
from typing import TextIO

import torch
import torch.optim as optim
import numpy as np
import torch.nn.functional as F

import math
from random import shuffle

import configparser

import cv2

# our nn model.py
import model as M


# def load_train_data(dataset_list, bachsize):
#     file = open(file_name)
#     content = file.read()
#     content_list = content.split('\n')
#
#     def load_data2(batch_size, batch_idx, dataset_list, data_path, target_path):
#         batch_weight = np.zeros(n_classes)
#         dataset_size = len(dataset_list)
#         n = math.floor(dataset_size / batch_size)
#         data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
#         target = torch.zeros(batch_size, img_x_dim, img_y_dim, dtype=torch.int64)
#
#         for j in range(batch_size):
#             # + 1 se indice comeca em 1
#             # print(batch_idx*batch_size + j)
#             data[j][0] = png2tensor(data_path + str(dataset_list[batch_idx * batch_size + j]) + '_max.png')[
#                 0]  # + 1) + '_max.png')
#             data[j][1] = png2tensor(data_path + str(dataset_list[batch_idx * batch_size + j]) + '_mean.png')[
#                 0]  # + 1) + '_mean.png')
#             data[j][2] = png2tensor(data_path + str(dataset_list[batch_idx * batch_size + j]) + '_min.png')[
#                 0]  # + 1) + '_min.png')
#             data[j][3] = png2tensor(data_path + str(dataset_list[batch_idx * batch_size + j]) + '_numb.png')[
#                 0]  # + 1) + '_numb.png')
#             data[j][4] = png2tensor(data_path + str(dataset_list[batch_idx * batch_size + j]) + '_std.png')[
#                 0]  # + 1) + '_std.png')
#             tmp, new_weights = png2target(target_path + str(dataset_list[batch_idx * batch_size + j]) + '_label.png')
#             target[j] = tmp[0]
#             batch_weight = batch_weight + new_weights
#         max_weight = max(batch_weight)
#         batch_weight = max_weight / batch_weight
#         # normalize weights
#         # max_w = max(batch_weight)
#         # min_w = min(batch_weight)
#         # batch_weight = (batch_weight - 1)*10/(max_w - 1) + 1
#         # print(weights)
#         # ziped = list(zip(dataset, weights))
#         # shuffle(ziped)
#         # dataset, weights = zip(*ziped)
#         return data, target, batch_weight

def tensor2rgbimage(tensor):
    img = tensor.permute(1,2,0).numpy()
    height, width = img.shape[:2]
    img_map = np.zeros((width,height,3), np.uint8)
    #print(np.where((img == [1]).all(axis = 2)))
    img_map[np.where((img == [0]).all(axis = 2))] = np.array([255,120,0])
    img_map[np.where((img == [2]).all(axis = 2))] = np.array([255,255,255])
    img_map[np.where((img == [1]).all(axis = 2))] = np.array([0,0,0])

    return img_map


def load_labels_fromfile_as_img(dataset_config):
    for n in train_list:
        file = (dataset_config['target_path'] + n + '_label')
        numpy_file = np.fromfile(file, dtype=float)
        reshaped = numpy_file.reshape(600, 600)
        img = labels_to_img(reshaped)
    return img


def labels_to_img(numpy_image):
    reshaped = numpy_image
    img = np.zeros((600, 600, 3), np.uint8)
    for i in range(600):
        # print("")
        for j in range(600):
            if reshaped[i][j] == 0:
                img[i][j] = np.array([255, 120, 0])
            elif reshaped[i][j] == 1.0:
                img[i][j] = np.array([255, 255, 255])
            elif reshaped[i][j] == 2.0:
                img[i][j] = np.array([0, 0, 0])
    return img


def show_dataset(dataset_tensor):

    cv2.imshow('Max', convert_metric_map_to_image(dataset_tensor[1][0], 'max'))
    cv2.imshow('Mean', convert_metric_map_to_image(dataset_tensor[1][1], 'mean'))
    cv2.imshow('Min', convert_metric_map_to_image(dataset_tensor[1][2], 'min'))
    cv2.imshow('Numb', convert_metric_map_to_image(dataset_tensor[1][3], 'numb'))
    cv2.imshow('Std', convert_metric_map_to_image(dataset_tensor[1][4], 'std'))
    cv2.waitKey(0)


def fixed_normalize(img, map, new_max_value, max_value, min):
    img_map = np.zeros((600, 600, 3), np.uint8)

    for i in range(600):
        for j in range(600):
            new_val = map[j][i].item()
            if new_val < -1:
                img_map[j][i][0] = -1
                img_map[j][i][1] = -1
                img_map[j][i][2] = -1
            cel_pixel = (new_val - min) * new_max_value / (max_value - min)
            img[j][i][0] = cel_pixel
            img[j][i][1] = cel_pixel
            img[j][i][2] = cel_pixel
    return img


def convert_metric_map_to_image(metric_map, metric_type):
    map_min = -1.0
    if metric_type == 'max':
        map_max = 5.0
    elif metric_type == 'numb':
        map_max = 50
    elif metric_type == 'min':
        map_max = 5
    elif metric_type == 'mean':
        map_max = 5
    elif metric_type == 'std':
        map_max = 20
    img = np.zeros((600, 600, 3), np.uint8)
    img = fixed_normalize(img, metric_map, 255.0, map_max, map_min)
    return img


def getDatasetList(file_name):
    file = open(file_name)
    content = file.read().splitlines()
    # content_list = content.split('\n')
    # for i in content:
    #     print(i)
    return content


def file_to_tensor(img_x_dim, img_y_dim, file):
    numpy_file = np.fromfile(file, dtype=float)
    data_tensor = torch.from_numpy(numpy_file.reshape(img_x_dim, img_y_dim))
    return data_tensor


def load_bach_data(dataset_list, data_path, target_path, last_element, batch_size,
                   input_dimensions, img_x_dim, img_y_dim):
    # batch_weight = np.zeros(n_classes)#??
    dataset_size = len(dataset_list)

    if (dataset_size - (last_element + batch_size)) <= 0:
        batch_size = (dataset_size - last_element)
        # print("sobrou: ", batch_size)

    data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
    target = torch.zeros(batch_size, img_x_dim, img_y_dim, dtype=torch.int64)

    for j in range(batch_size):
        # + 1 se indice comeca em 1
        # print(batch_size)
        # print("lastele: ", last_element, " j:", j)

        data[j][0] = (file_to_tensor(img_x_dim, img_y_dim, data_path + str(dataset_list[last_element]) + '_max'))
        data[j][1] = (file_to_tensor(img_x_dim, img_y_dim, data_path + str(dataset_list[last_element]) + '_mean'))
        data[j][2] = (file_to_tensor(img_x_dim, img_y_dim, data_path + str(dataset_list[last_element]) + '_min'))
        data[j][3] = (file_to_tensor(img_x_dim, img_y_dim, data_path + str(dataset_list[last_element]) + '_numb'))
        data[j][4] = (file_to_tensor(img_x_dim, img_y_dim, data_path + str(dataset_list[last_element]) + '_std'))

        tmp = file_to_tensor(img_x_dim, img_y_dim, (target_path + str(dataset_list[last_element]) + '_label2'))
        target[j] = tmp
        last_element = last_element + 1
        # cv2.imshow('window', labels_to_img(target[j].numpy()))
        # cv2.waitKey(0)
        # batch_weight = batch_weight + new_weights
    # cv2.imshow('Max', convert_metric_map_to_image(data[0][0], 'max'))
    # max_weight = max(batch_weight)
    # batch_weight = max_weight / batch_weight
    # normalize weights
    # max_w = max(batch_weight)
    # min_w = min(batch_weight)
    # batch_weight = (batch_weight - 1)*10/(max_w - 1) + 1
    # print(weights)
    # ziped = list(zip(dataset, weights))
    # shuffle(ziped)
    # dataset, weights = zip(*ziped)
    return data, target, last_element


def train(interval_save_model, iterations, model, device, train_list, dataset_config, optimizer, epoch, batch_size):
    #class_weights = torch.FloatTensor(weights).cuda()

    model.train()
    last_element = 0
    for batch_idx in range(iterations):
        if last_element < (len(train_list)):
            # print("Loading batch ", batch_idx, " of ", iterations, '\n The last element loaded was: ', last_element)
            data, target, last_element = load_bach_data(train_list, dataset_config['train_path'],
                                                        dataset_config['target_path'],
                                                        last_element, batch_size, input_channels,
                                                        img_width, img_height)

            # show_dataset(data)
            # cv2.imshow('Target', labels_to_img(target[batch_idx].numpy()))
            # cv2.waitKey(-1)
            #TODO: NORMALIZAR OS DADOS
            #TODO: Padronizar os dados
            data = data.to(device)
            target = target.to(device)

            # print("Fowarding")
            output = model(data)
            #print(weights)
            #weights = [1, 1, 5]
            # batch_weight = torch.FloatTensor(weights).cuda()
            # out_loss = F.cross_entropy(output, target, weight=batch_weight)
            # print("Calculating Cross_entropy")

            out_loss = F.cross_entropy(output, target)
            # print(out_loss.item())
            # print("Cleaning grads")
            optimizer.zero_grad()
            # print("Backward")

            out_loss.backward()
            # print("weights update")
            optimizer.step()
            if batch_idx % interval_save_model == 0:
                print('Loss: ', out_loss.item(), '\tTrain Epoch: {} [{}/{} ({:.0f}%)]'.format(epoch, batch_idx * len(data), len(train_list),
                    epoch, batch_idx * len(data), len(train_list),
                    100. * batch_idx * len(data) / len(train_list)))
            if epoch % 10 == 0:
                pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
                imgPred = pred[0]
                imgPred = imgPred.cpu().float()
                cv2.imshow('Predicted', labels_to_img(imgPred[0].numpy()))
                cv2.waitKey(10)
                #imgTarget = torch.FloatTensor(1, 424, 424)
                #imgTarget[0] = target[0]
                #imgTarget = imgTarget.cpu().float()
                #saveImage(imgPred, debug_img_path + '/predic_epoch' + str(epoch) + '.png')
                #saveImage(imgTarget, debug_img_path + '/target_epoch' + str(epoch) + '.png')
                # showOutput2(imgPred, "pred")
                #showOutput(imgTarget)



if __name__ == '__main__':

    # Load config and initializations
    config_file = configparser.ConfigParser()
    config_file.read('config.ini')
    dataset_config = config_file['DATASET']
    dnn_config = config_file['DNN']

    train_list = getDatasetList(dataset_config['train_list'])
    test_list = getDatasetList(dataset_config['test_list'])

    batch_size = int(dnn_config['batch_size'])
    iterations = math.ceil(len(train_list) / batch_size)
    input_channels = int(dataset_config['channels'])
    img_width = int(dataset_config['img_width'])
    img_height = int(dataset_config['img_height'])
    use_cuda = int(dnn_config['use_cuda'])
    device_number = dnn_config['device']
    epochs = int(dnn_config['epochs'])
    interval_save_model = int(dnn_config['interval_save_model'])
    random_seed = int(dnn_config['manual_seed'])
    # CUDA_LAUNCH_BLOCKING = 1

    use_cuda = use_cuda and torch.cuda.is_available()
    # TODO: Arrumar pra escolher pelo parametro
    device = torch.device("cuda") #torch.device(("cuda:" + device_number) if use_cuda else "cpu")
    print(torch.cuda.current_device(), torch.cuda.get_device_capability(int(device_number)))
    print('Using Device: ', device)

    # optimizer parameter?
    torch.manual_seed(random_seed)

    if dataset_config.getboolean('shuffle_data'):
        shuffle(train_list)
        shuffle(test_list)
        print('Data Shuffled')

    # load model
    model = M.FCNN(n_input=input_channels, n_output=int(dnn_config['classes'])).to(device)
    print('Model loaded', model)

    if dnn_config['use_trained_model'] is not "":
        model.load_state_dict(torch.load(dnn_config['use_trained_model']))
        print("Using model: ", dnn_config['use_trained_model'])

    optimizer = optim.Adam(model.parameters(), lr=float(dnn_config['learning_rate']))

    print('Iniciando treino por ', epochs, 'epocas')

    for epoch in range(1, epochs + 1):
        print(epochs, 'Epoch')
        train(interval_save_model, iterations, model, device, train_list, dataset_config, optimizer, epoch, batch_size)
        #test(args, model, device, test_list, epoch, args.test_batch_size)

        if epoch % dnn_config.getint('interval_save_model') == 0:
            print('Saving model at:', dnn_config['save_models'])
            torch.save(model.state_dict(), dnn_config['save_models'] + str(epoch) + '.model')

    print("cabou")
    torch.save(model.state_dict(), dnn_config['save_models'] + str(epoch) + '.model')
    config_log_file_name = dnn_config['save_log_files'] + 'config' + str(epochs) + '.ini'
    with open(config_log_file_name, 'w') as configfile:  # type: TextIO
        config_file.write(configfile)
    #
    # # print("Batch_size: ", batch_size, " batch_interations: ", batch_iterations, " Train_set: ", len(dataset_list))
    # for i in range(epochs):
    #
    #
    #
    # show_dataset(data)

    # load_model()
    #
    # load_train_data(dataset_train_list, bachsize)
    # load_test_data(dataset_test_list)
    #
    # train()
    # test()
    # debug()
    # Exemplo como gravar o arquivo de configuração usado em um arquivo
    # config_log_file_name = metrics_config['save_log_files'] + 'config' +
    # with open('config2.ini', 'w') as configfile:
    #     config_file.write(configfile)
