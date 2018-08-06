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

train_start_index = 0
test_start_index = 60
n_imgs = 79
n_train = 60
n_test = 19

img_x_dim = 424
img_y_dim = 424

data_dim = 5

data_path = '/dados/neural_mapper/60mts/data/'
target_path = '/dados/neural_mapper/60mts/labels/'

def png2tensor(file_name):
	img2tensor = transforms.ToTensor()
	img = Image.open(file_name)
	return img2tensor(img)

def load_train_data(batch_size, train_data_size):
	dataset = []

	n = math.floor(n_train/batch_size)
	for i in range(n):
		data = torch.zeros(batch_size, data_dim, img_x_dim, img_y_dim)
		target = torch.zeros(batch_size, img_x_dim, img_y_dim)

		for j in range(batch_size):
			data[j][0] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_max.png')
			data[j][1] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_mean.png')
			data[j][2] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_min.png')
			data[j][3] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_numb.png')
			data[j][4] = png2tensor(data_path + str(batch_size*i + j + train_start_index + 1) + '_std.png')
			target[j] = png2tensor(target_path + str(batch_size*i + j + train_start_index) + '_label.png')

		row = []
		row.append(data)
		row.append(target)
		dataset.append(row)
	return dataset


def load_test_data(batch_size, test_data_size):
	dataset = []

	n = math.floor(n_test/test_data_size)
	for i in range(n):
		data = torch.zeros(batch_size, data_dim, img_x_dim, img_y_dim)
		target = torch.zeros(batch_size, img_x_dim, img_y_dim)

		for j in range(batch_size):
			data[j][0] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_max.png')
			data[j][1] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_mean.png')
			data[j][2] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_min.png')
			data[j][3] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_numb.png')
			data[j][4] = png2tensor(data_path + str(batch_size*i + j + test_start_index + 1) + '_std.png')
			target[j] = png2tensor(target_path + str(batch_size*i + j + test_start_index) + '_label.png')

		row = []
		row.append(data)
		row.append(target)
		dataset.append(row)
	return dataset

def train(args, model, device, train_loader, optimizer, epoch):
    model.train()
    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.long().to(device)
        optimizer.zero_grad()
        output = model(data)
        loss = F.cross_entropy(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx % args.log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                100. * batch_idx / len(train_loader), loss.item()))

def test(args, model, device, test_loader):
    model.eval()
    test_loss = 0
    correct = 0
    with torch.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.long().to(device)
            output = model(data)
            test_loss += F.cross_entropy(output, target, size_average=False).item() # sum up batch loss
            pred = output.max(1, keepdim=True)[1] # get the index of the max log-probability
            correct += pred.eq(target.view_as(pred)).sum().item()

    test_loss /= len(test_loader.dataset)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
        test_loss, correct, len(test_loader.dataset),
        100. * correct / len(test_loader.dataset)))

if __name__ == '__main__':
	# Training settings
	parser = argparse.ArgumentParser(description='PyTorch Neural Mapper')
	parser.add_argument('--batch-size', type=int, default=5, metavar='N',
	                    help='input batch size for training (default: 64)')
	parser.add_argument('--test-batch-size', type=int, default=1, metavar='N',
	                    help='input batch size for testing (default: 1000)')
	parser.add_argument('--epochs', type=int, default=10, metavar='N',
	                    help='number of epochs to train (default: 10)')
	parser.add_argument('--lr', type=float, default=0.01, metavar='LR',
	                    help='learning rate (default: 0.01)')
	parser.add_argument('--momentum', type=float, default=0.5, metavar='M',
	                    help='SGD momentum (default: 0.5)')
	parser.add_argument('--no-cuda', action='store_true', default=False,
	                    help='disables CUDA training')
	parser.add_argument('--seed', type=int, default=1, metavar='S',
	                    help='random seed (default: 1)')
	parser.add_argument('--log-interval', type=int, default=10, metavar='N',
	                    help='how many batches to wait before logging training status')

	args = parser.parse_args()
	use_cuda = not args.no_cuda and torch.cuda.is_available()

	torch.manual_seed(args.seed)

	device = torch.device("cuda" if use_cuda else "cpu")

	kwargs = {'num_workers': 1, 'pin_memory': True} if use_cuda else {}
	train_data_size = 10
	test_data_size = 5
	train_loader = load_train_data(args.batch_size, train_data_size)
	test_loader = load_test_data(args.test_batch_size, test_data_size)
	print(train_loader[0][0].size, train_loader[0][1].size)

	model = M.FCNN().to(device)

	print(type(model.parameters()))

	optimizer = optim.Adam(model.parameters(), lr=args.lr)

	for epoch in range(1, args.epochs + 1):
	    train(args, model, device, train_loader, optimizer, epoch)
	    test(args, model, device, test_loader)
