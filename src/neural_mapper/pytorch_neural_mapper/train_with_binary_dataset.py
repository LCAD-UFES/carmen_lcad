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


def fixed_normalize_to_img(map, new_max_value, max_value, min):
    img_map = np.zeros((600, 600, 3), np.uint8)

    for i in range(600):
        for j in range(600):
            new_val = map[j][i].item()
            # print(new_val)
            if new_val < min:
                img_map[j][i][0] = min
                img_map[j][i][1] = min
                img_map[j][i][2] = min
            cel_pixel = (new_val - min) * new_max_value / (max_value - min)
            img_map[j][i][0] = cel_pixel
            img_map[j][i][1] = cel_pixel
            img_map[j][i][2] = cel_pixel
    return img_map


def fixed_normalize(map, new_max_value, max_value, min_value):
    img_map = np.zeros((600, 600))

    for i in range(600):
        for j in range(600):
            new_val = map[j][i].item()
            # print(new_val)
            if new_val < min_value:
                img_map[j][i] = min_value
            cel_pixel = (new_val - min_value) * new_max_value / (max_value - min_value)
            img_map[j][i] = cel_pixel
    return img_map


def print_values(map):
    for i in range(600):
        for j in range(600):
            new_val = map[j][i].item()
            print(new_val)

def get_min_and_max_values_to_normalize(metric_type):
    map_min = -10.0
    if metric_type == 'max':
        map_max = 1.852193
    elif metric_type == 'numb':
        map_max = 64.0
        map_min = 0.0
    elif metric_type == 'min':
        map_max = 1.852193
    elif metric_type == 'mean':
        map_max = 1.852193
    elif metric_type == 'std':
        map_max = 15.0
        map_min = -1.0

    return map_max, map_min


def convert_metric_map_to_image(metric_map, metric_type):
# TODO Pegar das variáveis
    map_max, map_min = get_min_and_max_values_to_normalize(metric_type)
    img = fixed_normalize_to_img(metric_map, 255.0, map_max, map_min)
    return img


def getDatasetList(file_name):
    file = open(file_name)
    content = file.read().splitlines()
    # content_list = content.split('\n')
    # for i in content:
    #     print(i)
    return content


def file_raw_to_tensor(img_x_dim, img_y_dim, file, metric_type):
    numpy_file = np.fromfile(file, dtype=float)
    max_value, min_value = get_min_and_max_values_to_normalize(metric_type)
    reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
    normalized_data = fixed_normalize(reshaped, 1.0, max_value, min_value)
    data_tensor = torch.from_numpy(normalized_data)
    return data_tensor


def file_to_tensor(img_x_dim, img_y_dim, file):
    numpy_file = np.fromfile(file, dtype=float)
    reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
    data_tensor = torch.from_numpy(reshaped)
    return data_tensor


def label_file_to_tensor(img_x_dim, img_y_dim, file):
    numpy_file = np.fromfile(file, dtype=float)

    reshaped = numpy_file.reshape(img_x_dim, img_y_dim)
    weight = np.array([len(np.where(reshaped == t)[0]) for t in np.unique(reshaped)])

    data_tensor = torch.from_numpy(reshaped)

    return data_tensor, weight


def load_bach_data(dataset_list, data_path, target_path, last_element, batch_size,
                   input_dimensions, img_x_dim, img_y_dim, n_classes):
    batch_weight = np.zeros(n_classes)
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

        tmp, new_weights = label_file_to_tensor(img_x_dim, img_y_dim, (target_path + str(dataset_list[last_element]) + '_label'))
        target[j] = tmp
        last_element = last_element + 1
        batch_weight = batch_weight + new_weights
        max_weight = max(batch_weight)
        batch_weight = max_weight / batch_weight

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
    return data, target, last_element, batch_weight


def train(interval_save_model, iterations, model, device, train_list, dataset_config, dnn_config, optimizer, epoch, batch_size, n_classes, img_width, img_height, input_channels):
    #class_weights = torch.FloatTensor(weights).cuda()

    model.train()
    last_element = 0
    for batch_idx in range(iterations):
        if last_element < (len(train_list)):
            # print("Loading batch ", batch_idx, " of ", iterations, '\n The last element loaded was: ', last_element)
            data, target, last_element, weights = load_bach_data(train_list, dataset_config['train_path'],
                                                        dataset_config['target_path'],
                                                        last_element, batch_size, input_channels,
                                                        img_width, img_height, n_classes)

            # show_dataset(data)
            # cv2.imshow('Target', labels_to_img(target[batch_idx].numpy()))
            # cv2.waitKey(-1)
            #TODO: Dados já estão sendo gerados de 0 a 1

            #TODO: Padronizar os dados
            data = data.to(device)
            target = target.to(device)
            # print("Fowarding")
            output = model(data)
            #print(weights)
            #weights = [1, 1, 5]
            batch_weight = torch.FloatTensor(weights).cuda()
            out_loss = F.cross_entropy(output, target, weight=batch_weight)
            # print("Calculating Cross_entropy")
            # print(out_loss.item())
            # print("Cleaning grads")
            optimizer.zero_grad()
            # print("Backward")
            out_loss.backward()
            # print("weights update")
            optimizer.step()
            if batch_idx % interval_save_model == 0:
                log_treino = ('Loss: {:.10f} Train Epoch: {} [{}/{} ({:.0f}%)]\n'.format(out_loss.item(), epoch, batch_idx * len(data), len(train_list),
                    epoch, batch_idx * len(data), len(train_list),
                    100. * batch_idx * len(data) / len(train_list)))
                arq = open(dnn_config['save_log_files']+'Train_averange.txt', 'a')
                arq.write(log_treino)
                arq.close()
                print(log_treino)

            if epoch % 50 == 0:
                pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
                imgPred = pred[0]
                imgPred = imgPred.cpu().float()
                cv2.imwrite('/dados/neural_mapper/data_13-08-19/results/img' + str(epoch) + '.png', labels_to_img(imgPred[0].numpy()))


def test(model, device, test_list, epoch, batch_size, dataset_config, dnn_config, n_classes, img_width, img_height, input_channels):
    print("Testing!")
    model.eval()
    test_loss = 0
    correct = 0
    last_element = 0
    with torch.no_grad():
        for batch_idx in range(math.floor(len(test_list)/batch_size)):
            if last_element < (len(train_list)):
                data, target, last_element, weights = load_bach_data(test_list, dataset_config['test_path'],
                                                            dataset_config['target_path'],
                                                            last_element, batch_size, input_channels,
                                                            img_width, img_height, n_classes)
                data = data.to(device)
                target = target.long().to(device)
                output = model(data)
                batch_weight = torch.FloatTensor(weights).cuda()
                test_loss += F.cross_entropy(output, target, reduction="sum").item() # sum up batch loss
                pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
                correct += pred.eq(target.view_as(pred)).sum().item()
        #    test_loss /= len(test_loader.dataset)
        test_loss /= (len(test_list)*batch_size*img_width*img_height)
        texto = ('Test set: epoch {} Average loss {:.10f}, Accuracy: {}/{} {:.6f}\n'.format(epoch,
            test_loss, correct, len(test_list)*img_width*img_height,
            correct / (len(test_list)*img_width*img_height)))
        arq = open(dnn_config['save_log_files']+'Test_averange.txt', 'a')
        arq.write(texto)
        
        arq.close()
        print(texto)
        
    
if __name__ == '__main__':

    # TODO Fazer um função pra isso:
    # Load config and initializations
    config_file = configparser.ConfigParser()
    config_file.read('config.ini')
    dataset_config = config_file['DATASET']
    dnn_config = config_file['DNN']

    train_list = getDatasetList(dataset_config['train_list'])
    test_list = getDatasetList(dataset_config['test_list'])
    #TODO Escrever no parametro [max_normalize_std] e max_normalize_numb, os valores pra esse dataset

    batch_size = int(dnn_config['batch_size'])
    decay_step_size = int(dnn_config['step_size'])
    decay_rate = int(dnn_config['decay_rate'])
    iterations = math.ceil(len(train_list) / batch_size)
    input_channels = int(dataset_config['channels'])
    n_classes = int(dnn_config['classes'])
    img_width = int(dataset_config['img_width'])
    img_height = int(dataset_config['img_height'])
    use_cuda = int(dnn_config['use_cuda'])
    device_number = dnn_config['device']
    epochs = int(dnn_config['epochs'])
    interval_save_model = int(dnn_config['interval_save_model'])
    random_seed = int(dnn_config['manual_seed'])
    learning_rate = float(dnn_config['learning_rate'])
    dropout_prob = float(dnn_config['dropout_prob'])
    # CUDA_LAUNCH_BLOCKING = 1

    use_cuda = use_cuda and torch.cuda.is_available()
    # TODO: Arrumar pra escolher pelo parametro
    device = torch.device("cuda") # torch.device(("cuda:" + device_number) if use_cuda else "cpu")
    print(torch.cuda.current_device(), torch.cuda.get_device_capability(int(device_number)))
    print('Using Device: ', device)

    # optimizer parameter?
    torch.manual_seed(random_seed)

    if dataset_config.getboolean('shuffle_data'):
        shuffle(train_list)
        shuffle(test_list)
        print('Data Shuffled')

    # load model
    model = M.FCNN(n_input=input_channels, n_output=n_classes, prob_drop=dropout_prob).to(device)
    print('Model loaded', model)

    if dnn_config['use_trained_model'] is not "":
        model.load_state_dict(torch.load(dnn_config['use_trained_model']))
        print("Using model: ", dnn_config['use_trained_model'])
    print("Learning rate: ", learning_rate)
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    print('Iniciando treino por ', epochs, 'epocas')

    for epoch in range(1, epochs + 1):
        train(interval_save_model, iterations, model, device, train_list,
              dataset_config, dnn_config, optimizer, epoch, batch_size, n_classes, img_width, img_height, input_channels)
        test(model, device, test_list, epoch, batch_size, dataset_config, dnn_config, n_classes, img_width, img_height, input_channels)

        if epoch % decay_step_size == 0:
            first = 1
            for g in optimizer.param_groups:
                if first:
                    print("Learning rate decay from: ", str(g['lr']), "to: ", g['lr'] / decay_rate)
                    first = 0
                g['lr'] /= decay_rate

        if epoch % dnn_config.getint('interval_save_model') == 0:
            print('Saving model at:', dnn_config['save_models'])
            torch.save(model.state_dict(), dnn_config['save_models'] + str(epoch) + '.model')

    print("cabou")
    torch.save(model.state_dict(), dnn_config['save_models'] + str(epoch) + '.model')
    config_log_file_name = dnn_config['save_log_files'] + 'config' + str(epochs) + '.ini'
    with open(config_log_file_name, 'w') as configfile:  # type: TextIO
        config_file.write(configfile)
