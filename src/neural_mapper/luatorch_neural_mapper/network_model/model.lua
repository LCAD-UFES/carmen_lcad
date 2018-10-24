require 'nn'
require 'cunn'
--require 'cudnn'

function GetModel()

num_classes = 2
n_layers_enc = 32
n_layers_ctx = 128
n_input = 5
prob_drop = 0.25

--backend = cudnn

---Encoder 
pool = nn.SpatialMaxPooling(2,2,2,2)

net = nn.Sequential()
net:add(nn.SpatialConvolution(n_input, n_layers_enc, 3, 3, 1, 1, 1, 1))
net:add(nn.ELU())
net:add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
net:add(nn.ELU())
net:add(pool)

---Context module 
net:add(nn.SpatialDilatedConvolution(n_layers_enc, n_layers_ctx, 3, 3, 1, 1, 1, 1, 1, 1))
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 1, 2, 1, 2))
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 2, 4, 2, 4));
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 4, 8, 4, 8));
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 8, 16, 8, 16));
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 16, 32, 16, 32));
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 32, 64, 32, 64));
net:add(nn.ELU())
net:add(nn.SpatialDropout(prob_drop))
net:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_enc, 1, 1));
net:add(nn.ELU())

---Decoder
net:add(nn.SpatialMaxUnpooling(pool))
net:add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
net:add(nn.ELU())
net:add(nn.SpatialConvolution(n_layers_enc, num_classes, 3, 3, 1, 1, 1, 1))

return net


end

model = GetModel()



