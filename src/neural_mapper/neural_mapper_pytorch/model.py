"""
import torch.legacy.nn as nn

# Modelo da rede neural
def FCNN():
	num_classes = 2
	n_layers_enc = 32
	n_layers_ctx = 128
	n_input = 5
	prob_drop = 0.25
	layers = []
	# Encoder
	model = nn.Sequential()
	pool = nn.SpatialMaxPooling(2,2,2,2)
	model.add(nn.SpatialConvolution(n_input, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model.add(nn.ELU())
	model.add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model.add(nn.ELU())
	model.add(pool)
	# Context Module
	model.add(nn.SpatialDilatedConvolution(n_layers_enc, n_layers_ctx, 3, 3, 1, 1, 1, 1, 1, 1))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 2, 2, 2, 2))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 4, 4, 4, 4))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 8, 8, 8, 8))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 16, 16, 16, 16))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 32, 32, 32, 32))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 64, 64, 64, 64))
	model.add(nn.ELU())
	model.add(nn.SpatialDropout(prob_drop))
	model.add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_enc, 1, 1))
	model.add(nn.ELU())	# Nao havia no paper
	# Decoder
	model.add(nn.SpatialMaxUnpooling(pool))
	model.add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model.add(nn.ELU())
	model.add(nn.SpatialConvolution(n_layers_enc, num_classes, 3, 3, 1, 1, 1, 1))
	model.add(nn.ELU())
	model.add(nn.SoftMax()) # Nao havia no paper
	return model
"""
import torch
import torch.nn as nn
import torch.nn.functional as F


# LiDAR road detection FCNN as described in https://arxiv.org/pdf/1703.03613.pdf
class FCNN(nn.Module):
	def __init__(self, n_input=5, n_output=3, prob_drop=0.25):
		print("Print inside model.py: prob_drop = ", prob_drop)
		super(FCNN, self).__init__()
		self.num_classes = n_output
		self.n_layers_enc = 32
		self.n_layers_ctx = 128
		self.n_input = n_input
		self.prob_drop = prob_drop

		# Encoder
		self.enc1 = nn.Conv2d(self.n_input, self.n_layers_enc, 3, stride=1, padding=1, dilation=1)
		self.enc2 = nn.Conv2d(self.n_layers_enc, self.n_layers_enc, 3, stride=1, padding=1, dilation=1)
		# Context
		self.con1 = nn.Conv2d(self.n_layers_enc, self.n_layers_ctx, 3, stride=1, padding=1, dilation=1)
		self.con2 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=2, dilation=2)
		self.con3 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=4, dilation=4)
		self.con4 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=8, dilation=8)
		self.con5 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=16, dilation=16)
		self.con6 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=32, dilation=32)
		self.con7 = nn.Conv2d(self.n_layers_ctx, self.n_layers_ctx, 3, stride=1, padding=64, dilation=64)
		self.con8 = nn.Conv2d(self.n_layers_ctx, self.n_layers_enc, 1, stride=1, padding=0, dilation=1)
		# Decoder
		self.dec1 = nn.Conv2d(self.n_layers_enc, self.n_layers_enc, 3, stride=1, padding=1, dilation=1)
		self.dec2 = nn.Conv2d(self.n_layers_enc, self.num_classes, 3, stride=1, padding=1, dilation=1)        

	def forward(self, x):
		#max_pool2d(input, kernel_size, stride=None, padding=0, dilation=1, ceil_mode=False, return_indices=False)[
		# Encoder
		x = F.elu(self.enc1(x))
		x = F.elu(self.enc2(x))
		# Max pooling over a (2, 2) window, stride 2
		x, indices = F.max_pool2d(x, 2, stride=2, return_indices=True)
		#print("Out Encoder")
		#print(x.size())
		# Context
		#dropout(input, p=0.5, training=False, inplace=False)
		x = F.dropout2d(F.elu(self.con1(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con2(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con3(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con4(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con5(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con6(x)), self.prob_drop)
		#print(x.size())
		x = F.dropout2d(F.elu(self.con7(x)), self.prob_drop)
		#print(x.size())
		x = self.con8(x)
		#print("Out Context")
		#print(x.size())
		# Decoder
		# max_unpool2d(input, indices, kernel_size, stride=None, padding=0, output_size=None)
		x = F.max_unpool2d(x, indices, (2,2), stride=2)
		x = F.elu(self.dec1(x))
		x = F.elu(self.dec2(x))
		#Como estamos usando a Cross-Entropy, ela mesmo aplica o log_softmax, NÂO é necessário usar log softmax nem softmax aqui.
# 		x = F.log_softmax(x, dim=1)
		#Usei a softmax para melhor usar a saída no mapa probabilistico 
		prob = F.softmax(x, dim=1)
		# x = x.view(-1, self.num_flat_features(x))
		#print("Out Decoder")
		#print(x.size())
		return x, prob

	def num_flat_features(self, x):
		size = x.size()[1:]  # all dimensions except the batch dimension
		num_features = 1
		for s in size:
			num_features *= s
		return num_features

'''
net = FCNN()
print(net)

input = torch.randn(1, 5, 424, 424)
out = net(input)
print(out.size())
'''
