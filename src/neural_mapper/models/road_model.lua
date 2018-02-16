require 'nn'
require 'cunn'
require 'cudnn'

torch.setdefaulttensortype('torch.FloatTensor')

----------------------------------------------------------------------
print '==> define parameters'

local histClasses = opt.datahistClasses
local classes = opt.dataClasses
local conClasses = opt.dataconClasses

----------------------------------------------------------------------
print '==> construct model'

function GetModel()
	num_classes = 2
	n_layers_enc = 32
	n_layers_ctx = 128
	n_input = 5
	prob_drop = 0.25


	---Encoder 
	pool = nn.SpatialMaxPooling(2,2,2,2)

	local model = nn.Sequential()
	model:add(nn.SpatialConvolution(n_input, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model:add(nn.ELU())
	model:add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model:add(nn.ELU())
	model:add(pool)

	---Context module 
	model:add(nn.SpatialDilatedConvolution(n_layers_enc, n_layers_ctx, 3, 3, 1, 1, 1, 1, 1, 1))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 2, 2, 2, 2))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 4, 4, 4, 4))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 8, 8, 8, 8))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 16, 16, 16, 16))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 32, 32, 32, 32))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_ctx, 3, 3, 1, 1, 64, 64, 64, 64))
	model:add(nn.ELU())
	model:add(nn.SpatialDropout(prob_drop))
	model:add(nn.SpatialDilatedConvolution(n_layers_ctx, n_layers_enc, 1, 1));
--	model:add(nn.ELU()) -- Nao aparece no papper

	---Decoder
	model:add(nn.SpatialMaxUnpooling(pool))
	model:add(nn.SpatialConvolution(n_layers_enc, n_layers_enc, 3, 3, 1, 1, 1, 1))
	model:add(nn.ELU())
	model:add(nn.SpatialConvolution(n_layers_enc, num_classes, 3, 3, 1, 1, 1, 1))
	model:add(nn.ELU())  
  model:add(nn.SoftMax())

	return model
end

local model = GetModel()

-- Loss: NLL
print('defining loss function:')
local normHist = histClasses / histClasses:sum()
local classWeights = torch.Tensor(#classes):fill(1)
--[[
for i = 1, #classes do
   if histClasses[i] < 1 or i == 1 then -- ignore unlabeled
      classWeights[i] = 0
   else
      classWeights[i] = 1 / (torch.log(1.2 + normHist[i]))
   end
end
]]
loss = cudnn.SpatialCrossEntropyCriterion(classWeights)
--loss = nn.SpatialCrossEntropyCriterion(classWeights)


----------------------------------------------------------------------
print '==> here is the model:'
print(model)

-- return package:
return {
   model = model,
   loss = loss,
}



