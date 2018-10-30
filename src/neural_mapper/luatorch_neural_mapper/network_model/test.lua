require 'nn'
require 'image'
require 'cudnn'
require 'cunn'

require("model")
require("get_data")

--- recebe dados de entrada e saida para treinar
function Forward(inputData)

res = model:forward(inputData)

if itorch then
   itorch.image(res)
else
   print('skipping visualization because the script is not run from itorch')
end


end

local data,_ = GetBatchWithType('Dataset','um',0)---image.load('um_000000.png',3,'double') 

if itorch then
   itorch.image(data[1])
else
   print('skipping visualization because the script is not run from itorch')
end

-- print(#data)
Forward(data[1])

