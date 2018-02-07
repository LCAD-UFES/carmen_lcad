----------------------------------------------------------------------
-- Main script for training a model for semantic segmentation
--
-- Abhishek Chaurasia, Eugenio Culurciello
-- Sangpil Kim, Adam Paszke
----------------------------------------------------------------------

require 'pl'
require 'nn'
require 'image'

----------------------------------------------------------------------
-- Local repo files
local opts = require 'opts'

-- Get the input arguments parsed and stored in opt
opt = opts.parse(arg)

torch.setdefaulttensortype('torch.FloatTensor')

-- print('==> switching to CUDA')
require 'cudnn'
require 'cunn'
--print(cutorch.getDeviceProperties(opt.devid))
cutorch.setDevice(opt.devid)
print("Folder created at " .. opt.save)
os.execute('mkdir -p ' .. opt.save)


----------------------------------------------------------------------
print '==> load modules'
local data, chunks, ft
if opt.dataset == 'cv' then
   data  = require 'data/loadCamVid'
elseif opt.dataset == 'cs' then
   data = require 'data/loadCityscape'
elseif opt.dataset == 'su' then
   data = require 'data/loadSUN'
elseif opt.dataset == 'kitti' then
   data = require 'data/loadKITTI'
else
   error ("Dataset loader not found. (Available options are: cv/cs/su/kitti")
end

print 'saving opt as txt and t7'
local filename = paths.concat(opt.save,'opt.txt')
local file = io.open(filename, 'w')
for i,v in pairs(opt) do
    file:write(tostring(i)..' : '..tostring(v)..'\n')
end
file:close()
torch.save(path.join(opt.save,'opt.t7'),opt)

----------------------------------------------------------------------
print '==> training!'
local epoch = 1

not_initialized_model = paths.dofile(opt.model)

-- reset weights
local method = 'xavier'
local model = require('weight-init')(not_initialized_model.model, method)

t = not_initialized_model
t.model = model

t.loss:cuda()
t.model:cuda()

local train = require 'train_neural_mapper'
local test  = require 'test_neural_mapper'

--[[
print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
print(data:size())
print(datax.trainData.labels:size())
print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

imagem = datax.trainData.data[1][4]:add(0.1)
image.save("caco.png", imagem)
labelImg = datax.trainData.labels[1]:add(-1)
image.save("caco2.png", labelImg)
]]

while epoch < opt.maxepoch do
   local trainConf, model, loss = train(data.trainData, opt.dataClasses, epoch)
   test(data.testData, opt.dataClasses, epoch, trainConf, model, loss )
   trainConf = nil
   collectgarbage()
   epoch = epoch + 1
end
