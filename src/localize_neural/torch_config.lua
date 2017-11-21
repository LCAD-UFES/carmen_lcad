require 'torch'
require 'image'
require 'nn'
require 'cutorch'
require 'cunn'
require 'cudnn'

cutorch.setDevice(1)

torch.setnumthreads(1)    -- In server we usually want to run single-threaded
                          -- so the server itself can run multiple threads
net = nil

function load (model)    -- this function will be called by C++ constructor
    net = torch.load(model)
    net = net:cuda()
    net:evaluate()
end

function forward (curr_data, base_data)
    local output = net:forward({curr_data, base_data})
    return output:float()
end
