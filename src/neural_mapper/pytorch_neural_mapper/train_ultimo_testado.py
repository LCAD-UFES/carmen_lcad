#
# Main script for training a model for semantic segmentation of 
# roads from Velodyne point clouds
#
# Andre Seidel Oliveira
#

from __future__ import print_function
import argparse
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.legacy.nn as nn2
import torch.optim as optim
from torchvision import datasets, transforms
from PIL import Image
import math
import model as M
from random import shuffle
import matplotlib.pyplot as plt
import numpy as np
import cv2

n_imgs = 69
n_train = 10
n_test = 10
#n_imgs = 10
#n_train = 10
#n_test = 5

train_start_index = 0
test_start_index = n_train

img_y_dim = 600
img_x_dim = 600

train_data_path = '/dados/circle_NOTACUMULATED/rotate_treino/data/'
train_target_path = '/dados/circle_NOTACUMULATED/rotate_treino/labels/'
test_data_path = '/dados/circle_NOTACUMULATED/teste/data/'
test_target_path = '/dados/circle_NOTACUMULATED/teste/labels/'

debug_img_path = 'debug_imgs/'

input_dimensions = 5
n_classes = 3

def png2tensor(file_name):
    img2tensor = transforms.ToTensor()
    img = Image.open(file_name)
    return img2tensor(img)

def png2target(file_name):
    img2tensor = transforms.ToTensor()
    img = Image.open(file_name)
    #print((img2tensor(img)*255 - 1))
    target = (img2tensor(img)*255 - 1)

    weight = np.array([len(np.where(target.numpy() == t)[0]) for t in np.unique(target.numpy())])

    #print(weight)
    return target, weight
    #return img2tensor(img)

def tensor2png(tensor):
    trans = transforms.ToPILImage()
    return trans(tensor)

def tensor2rgbimage(tensor):
    img = tensor.permute(1,2,0).numpy()
    height, width = img.shape[:2]
    img_map = np.zeros((width,height,3), np.uint8)
    #print(np.where((img == [1]).all(axis = 2)))
    img_map[np.where((img == [0]).all(axis = 2))] = np.array([255,120,0])
    img_map[np.where((img == [1]).all(axis = 2))] = np.array([0,0,0])
    img_map[np.where((img == [2]).all(axis = 2))] = np.array([255,255,255])

    return img_map

def showOutput(tensor):
    img = tensor2png(tensor)
    img.show()

def showOutput2(tensor):
    img = tensor2rgbimage(tensor)
    cv2.imshow('image',img)
    cv2.waitKey(1)

def saveImage(tensor, file_name):
    img = tensor2png(tensor)
    img.save(file_name)

def load_data(batch_size, file_name):
    dataset_list = getDatasetList(file_name)
    shuffle(dataset_list)
    dataset = []
    weights = []
    batch_weight = np.zeros(n_classes)
    dataset_size = len(dataset_list)
    n = math.floor(dataset_size/batch_size)
    for i in range(n):
        data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
        target = torch.zeros(batch_size, img_x_dim, img_y_dim, dtype=torch.int64)

        for j in range(batch_size):
            # + 1 se indice comeca em 1 
            print(i*batch_size + j)
            data[j][0] = png2tensor(data_path + str(dataset_list[i*batch_size + j]) + '_max.png')[0]# + 1) + '_max.png')
            data[j][1] = png2tensor(data_path + str(dataset_list[i*batch_size + j]) + '_mean.png')[0]# + 1) + '_mean.png')
            data[j][2] = png2tensor(data_path + str(dataset_list[i*batch_size + j]) + '_min.png')[0]# + 1) + '_min.png')
            data[j][3] = png2tensor(data_path + str(dataset_list[i*batch_size + j]) + '_numb.png')[0]# + 1) + '_numb.png')
            data[j][4] = png2tensor(data_path + str(dataset_list[i*batch_size + j]) + '_std.png')[0]# + 1) + '_std.png')
            tmp, new_weights = png2target(target_path + str(dataset_list[i*batch_size + j]) + '_label.png')
            target[j] = tmp[0]
            batch_weight = batch_weight + new_weights
        max_weight = max(batch_weight)
        batch_weight = max_weight/batch_weight
        # normalize weights
        #max_w = max(batch_weight)
        #min_w = min(batch_weight)
        #batch_weight = (batch_weight - 1)*10/(max_w - 1) + 1
        weights.append(batch_weight)
        row = []
        row.append(data)
        row.append(target)
        dataset.append(row)
        #print(weights)
    #ziped = list(zip(dataset, weights))
    #shuffle(ziped)
    #dataset, weights = zip(*ziped)
    return dataset, weights

def load_data2(batch_size, batch_idx, dataset_list, data_path, target_path):
    batch_weight = np.zeros(n_classes)
    dataset_size = len(dataset_list)
    n = math.floor(dataset_size/batch_size)
    data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
    target = torch.zeros(batch_size, img_x_dim, img_y_dim, dtype=torch.int64)

    for j in range(batch_size):
        # + 1 se indice comeca em 1 
        #print(batch_idx*batch_size + j)
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
    # normalize weights
    #max_w = max(batch_weight)
    #min_w = min(batch_weight)
    #batch_weight = (batch_weight - 1)*10/(max_w - 1) + 1
    #print(weights)
    #ziped = list(zip(dataset, weights))
    #shuffle(ziped)
    #dataset, weights = zip(*ziped)
    return data, target, batch_weight

def load_train_data(batch_size, train_data_size):
    dataset = []

    n = math.floor(train_data_size/batch_size)
    for i in range(n):
        data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
        target = torch.zeros(batch_size, img_x_dim, img_y_dim)

        for j in range(batch_size):
            data[j][0] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_max.png')
            data[j][1] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_mean.png')
            data[j][2] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_min.png')
            data[j][3] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_numb.png')
            data[j][4] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_std.png')
            target[j] = png2tensor(target_path + str(batch_size*i + j + train_start_index) + '_view.png')

        row = []
        row.append(data)
        row.append(target)
        dataset.append(row)
    return dataset


def load_test_data(batch_size, test_data_size):
    dataset = []

    n = math.floor(test_data_size/batch_size)
    for i in range(n):
        data = torch.zeros(batch_size, input_dimensions, img_x_dim, img_y_dim)
        target = torch.zeros(batch_size, img_x_dim, img_y_dim)

        for j in range(batch_size):
            data[j][0] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_max.png')
            data[j][1] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_mean.png')
            data[j][2] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_min.png')
            data[j][3] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_numb.png')
            data[j][4] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_std.png')
            target[j] = png2tensor(target_path + str(batch_size*i + j + test_start_index) + '_view.png')

        row = []
        row.append(data)
        row.append(target)
        dataset.append(row)
    return dataset

def train(args, model, device, train_loader, optimizer, epoch, weights):
    #class_weights = torch.FloatTensor(weights).cuda()

    model.train()
    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.long().to(device)
        optimizer.zero_grad()
        output = model(data)
        batch_weight = torch.FloatTensor(weights[batch_idx]).cuda()
        out_loss = F.cross_entropy(output, target, weight=batch_weight)
        out_loss.backward()
        optimizer.step()
        if batch_idx % args.log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
    #                epoch, batch_idx * len(data), len(train_loader.dataset),
                epoch, batch_idx * len(data), len(train_loader)*args.batch_size,
                100. * batch_idx / len(train_loader), out_loss.item()))

            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            imgPred = pred[0]
            imgPred = imgPred.cpu().float()
            #imgTarget = torch.FloatTensor(1, 424, 424)
            #imgTarget[0] = target[0]
            #imgTarget = imgTarget.cpu().float()
            #saveImage(imgPred, debug_img_path + '/predic_epoch' + str(epoch) + '.png')
            #saveImage(imgTarget, debug_img_path + '/target_epoch' + str(epoch) + '.png')
            showOutput2(imgPred)
            #showOutput(imgTarget)

def train2(args, model, device, train_loader, optimizer, epoch, batch_size):
    #class_weights = torch.FloatTensor(weights).cuda()

    model.train()
    for batch_idx in range(math.floor(len(train_loader)/batch_size)):
        data, target, weights = load_data2(batch_size, batch_idx, train_loader, train_data_path, train_target_path)
        data, target = data.to(device), target.long().to(device)
        optimizer.zero_grad()
        output = model(data)
        #print(weights)
        #weights = [1, 1, 5]
        batch_weight = torch.FloatTensor(weights).cuda()
        out_loss = F.cross_entropy(output, target, weight=batch_weight)
        out_loss.backward()
        optimizer.step()
        if batch_idx % args.log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
    #                epoch, batch_idx * len(data), len(train_loader.dataset),
                epoch, batch_idx * len(data), len(train_loader),
                100. * batch_idx *len(data)/ len(train_loader), out_loss.item()))

            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            imgPred = pred[0]
            imgPred = imgPred.cpu().float()
            #imgTarget = torch.FloatTensor(1, 424, 424)
            #imgTarget[0] = target[0]
            #imgTarget = imgTarget.cpu().float()
            #saveImage(imgPred, debug_img_path + '/predic_epoch' + str(epoch) + '.png')
            #saveImage(imgTarget, debug_img_path + '/target_epoch' + str(epoch) + '.png')
            showOutput2(imgPred)
            #showOutput(imgTarget)

def test2(args, model, device, test_loader, epoch, batch_size):
    model.eval()
    test_loss = 0
    correct = 0
    with torch.no_grad():
        for batch_idx in range(math.floor(len(test_loader)/batch_size)):
            data, target, weights = load_data2(batch_size, batch_idx, test_loader, test_data_path, test_target_path)
            data, target = data.to(device), target.long().to(device)
            output = model(data)
            batch_weight = torch.FloatTensor(weights).cuda()
            test_loss += F.cross_entropy(output, target, reduction="sum").item() # sum up batch loss
            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            correct += pred.eq(target.view_as(pred)).sum().item()
    #    test_loss /= len(test_loader.dataset)
    test_loss /= (len(test_loader)*batch_size*600*600)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
        test_loss, correct, len(test_loader)*600*600,
        100. * correct / (len(test_loader)*600*600)))

def test(args, model, device, test_loader, epoch, weights):
    model.eval()
    test_loss = 0
    correct = 0
    with torch.no_grad():
        for batch_idx, (data, target) in enumerate(test_loader):
            data, target = data.to(device), target.long().to(device)
            output = model(data)
            batch_weight = torch.FloatTensor(weights[batch_idx]).cuda()
            test_loss += F.cross_entropy(output, target, reduction="sum", WEIGHT=batch_weight).item() # sum up batch loss
            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            correct += pred.eq(target.view_as(pred)).sum().item()

    #    test_loss /= len(test_loader.dataset)
    test_loss /= (len(test_loader)*test_loader[0][1].size(0)*test_loader[0][1].size(1)*test_loader[0][1].size(2))
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
        test_loss, correct, len(test_loader)*test_loader[0][1].size(0)*test_loader[0][1].size(1)*test_loader[0][1].size(2),
        100. * correct / (len(test_loader)*test_loader[0][1].size(0)*test_loader[0][1].size(1)*test_loader[0][1].size(2))))

def getDatasetList(file_name):
    file = open(file_name)
    content = file.read()
    content_list = content.split('\n')
    return content_list


if __name__ == '__main__':
    # Training settings
    parser = argparse.ArgumentParser(description='PyTorch Neural Mapper')
    parser.add_argument('--batch-size', type=int, default=5, metavar='N',
                        help='input batch size for training (default: 64)')
    parser.add_argument('--test-batch-size', type=int, default=2, metavar='N',
                        help='input batch size for testing (default: 1000)')
    parser.add_argument('--epochs', type=int, default=100, metavar='N',
                        help='number of epochs to train (default: 10)')
    parser.add_argument('--lr', type=float, default=0.001, metavar='LR',
                        help='learning rate (default: 0.01)')
    parser.add_argument('--momentum', type=float, default=0.5, metavar='M',
                        help='SGD momentum (default: 0.5)')
    parser.add_argument('--no-cuda', action='store_true', default=False,
                        help='disables CUDA training')
    parser.add_argument('--seed', type=int, default=1, metavar='S',
                        help='random seed (default: 1)')
    parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                        help='how many batches to wait before logging training status')
    parser.add_argument('--use-model', type=str, default=None, metavar='N',
                        help='Enter the name of a trained model (.model file)')
    parser.add_argument('--show-plots', action='store_true', default=False,
                        help='Enables loss plots')
    parser.add_argument('--training-logs', action='store_true', default=True,
                        help='Enables loss plots')
    parser.add_argument('--dataset-index', action='store_true', default='dataset_index.txt',
                        help='Dataset index list')

    args = parser.parse_args()
    use_cuda = not args.no_cuda and torch.cuda.is_available()

    torch.manual_seed(args.seed)

    device = torch.device("cuda" if use_cuda else "cpu")

    kwargs = {'num_workers': 1, 'pin_memory': True} if use_cuda else {}

    # Training and testing loss plots
    # test_loss_plot, = plt.plot(range(1, args.epochs), [], 'bs')

    train_list = getDatasetList("treino.txt")
    test_list = getDatasetList("teste.txt")

    shuffle(train_list)
    shuffle(test_list)
    # train_loader = load_train_data(args.batch_size, n_train)
    # test_loader = load_test_data(args.test_batch_size, n_test)

    #train_set, train_weights = load_data(args.batch_size, "treino.txt")
    #test_set, test_weights = load_data(args.test_batch_size, "teste.txt")

    print("Train loader dataset size: " + str(len(train_list)))
    #print("Train loader data dimensions: " + str(train_set[0][0].size()), "Train loader target dimensions: " + str(train_set[0][1].size()))

    print("Test loader dataset size: " + str(len(test_list)))    
    #print("Test loader data dimensions: " + str(test_set[0][0].size()), "Test loader target dimensions: " + str(test_set[0][1].size()))

    model = M.FCNN(n_input=input_dimensions, n_output=n_classes).to(device)
    if(args.use_model is not None):
        model.load_state_dict(torch.load('saved_models/'+args.use_model))
        print("Using model " + args.use_model)
    
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    #weights[1] = weights[1]*50
    #print("PESOS:")
    #print (train_weights)
    #weights = [1., 100.]
    #class_weights = torch.FloatTensor(weights).cuda()

    for epoch in range(1, args.epochs + 1):
        train2(args, model, device, train_list, optimizer, epoch, args.batch_size)
        test2(args, model, device, test_list, epoch, args.test_batch_size)
        if(epoch % 1 == 0):
            torch.save(model.state_dict(), 'saved_models/' + str(n_imgs)  + "imgs_epoch" + str(epoch) + '.model')

'''
    for epoch in range(1, args.epochs + 1):
        train(args, model, device, train_set, optimizer, epoch, train_weights)
        test(args, model, device, test_set, epoch, test_weights)
        if(epoch % 10 == 0):
            torch.save(model.state_dict(), 'saved_models/' + str(n_imgs)  + "imgs_epoch" + str(epoch) + '.model')


    data = train_loader[0][0]
    target = train_loader[0][1]
    data, target = data.to(device), target.long().to(device)
    output = model(data)
    pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability

    #imgOut1 = torch.Tensor(1, 424, 424)
    #imgOut1[0] = output[0][0]

    #imgOut1 = imgOut1.cpu().float()
    imgPred = pred[0]
    imgPred = imgPred.cpu().float()
    print(target.size())
    imgTarget = torch.FloatTensor(1, 424, 424)
    imgTarget[0] = target[0]
    imgTarget = imgTarget.cpu().float()
    #saveImage(imgOut1, 'output1.png')
    saveImage(imgPred, 'prediction.png')
    saveImage(imgTarget-, 'target.png')
    #showOutput(imgOut1)
    showOutput(imgPred)
    showOutput(imgTarget)

    '''