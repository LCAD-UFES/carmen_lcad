----------------------------------------------------------------------
-- Main script for training a model for semantic segmentation of 
-- roads from Velodyne point clouds
--
-- Alberto F. De Souza, ...
----------------------------------------------------------------------

require 'pl'
require 'nn'
require 'image'
require 'cudnn'
require 'cunn'

----------------------------------------------------------------------

local opts = require 'opts'

-- Get the input arguments parsed and stored in opt
opt = opts.parse(arg)

torch.setdefaulttensortype('torch.FloatTensor')

-- print('==> switching to CUDA')
--print(cutorch.getDeviceProperties(opt.devid))
cutorch.setDevice(opt.devid)
print("Folder created at " .. opt.save)
os.execute('mkdir -p ' .. opt.save)

----------------------------------------------------------------------
print '=======> Loading the dataset'
local data
if opt.dataset == 'kitti' then
   data = require 'data/loadKITTI'
else
   error ("Dataset loader not found. (Available options are: cv/cs/su/kitti")
end

print '=======> Saving opt as txt and t7'
local filename = paths.concat(opt.save,'opt.txt')
local file = io.open(filename, 'w')
for i,v in pairs(opt) do
    file:write(tostring(i)..' : '..tostring(v)..'\n')
end
file:close()
torch.save(path.join(opt.save,'opt.t7'),opt)

----------------------------------------------------------------------
print '=======> Training!'
local epoch = 1

not_initialized_model = paths.dofile(opt.model)

-- Initialize weights
local method = 'xavier'
local model = require('weight-init')(not_initialized_model.model, method)

t = not_initialized_model
t.model = model

t.loss:cuda()
t.model:cuda()

local train = require 'train_neural_mapper'
local test  = require 'test_neural_mapper'

while epoch < opt.maxepoch do
   local trainConf, model, loss = train(data.trainData, opt.dataClasses, epoch)
   test(data.testData, opt.dataClasses, epoch, trainConf, model, loss)
   trainConf = nil
   collectgarbage()
   epoch = epoch + 1
end
