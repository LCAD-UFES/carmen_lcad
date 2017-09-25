require 'torch'
require 'image'
require 'nn'

torch.setdefaulttensortype('torch.FloatTensor')

torch.setnumthreads(1)    -- In server we usually want to run single-threaded
                          -- so the server itself can run multiple threads
net = nil

function load (model)    -- this function will be called by C++ constructor
    net = torch.load(model)
    net:evaluate()
end

function forward (curr_data, base_data)
    return net:forward{curr_data,base_data}
end
