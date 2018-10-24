----------------------------------------------------------------------
-- This script
--   + constructs mini-batches on the fly
--   + computes model error
--   + optimizes the error using several optmization
--     methods: SGD, L-BFGS, ADAM.
--
-- Written by  : Abhishek Chaurasia, Eugenio Culurciello
-- Dated       : January 2016
-- Last Updated: June, 2016
----------------------------------------------------------------------

require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods
require 'string'
torch.setdefaulttensortype('torch.FloatTensor')

local loss = t.loss

----------------------------------------------------------------------
print '==> defining some tools'

----------------------------------------------------------------------

-- Retrieve parameters and gradients:
-- this extracts and flattens all the trainable parameters of the mode
-- into a 1-dim vector
local w, dE_dw
local model = t.model
print '==> flattening model parameters'
w,dE_dw = model:getParameters()
----------------------------------------------------------------------
print '==> defining training procedure'

local confusion
if opt.dataconClasses then
  print('Class \'Unlabeled\' is ignored in confusion matrix')
  confusion = optim.ConfusionMatrix(opt.dataconClasses)
else
  confusion = optim.ConfusionMatrix(opt.dataClasses)
end

local optimState = {
  learningRate = opt.learningRate,
  momentum = opt.momentum,
  learningRateDecay = opt.learningRateDecay
}

----------------------------------------------------------------------
local function train(trainData, classes, epoch)

  --[[
   xx = torch.Tensor(1, opt.channels, opt.imHeight, opt.imWidth)
   xx[1] = trainData.data[1]
   image.save("caco.png", xx[1][5]:add(0.1))

   local y = model:forward(xx:cuda())
   print(y:size())
   labelImg = y[1][1]:add(-1)
   image.save("caco2.png", labelImg)
   labelImg = y[1][2]:add(-1)
   image.save("caco3.png", labelImg)
  ]]
  if epoch % opt.lrDecayEvery == 0 then optimState.learningRate = optimState.learningRate * opt.learningRateDecay end

  -- local vars
  local time = sys.clock()


  -- total loss error
  local err
  local totalerr = 0

  -- This matrix records the current confusion across classes
  -- shuffle at each epoch
  local shuffle = torch.randperm(trainData:size())

  model:training()
  -- do one epoch
  local x = torch.Tensor(opt.batchSize, opt.channels, opt.imHeight, opt.imWidth)
  local yt = torch.Tensor(opt.batchSize, opt.labelHeight, opt.labelWidth)

  print("==> Training: epoch # " .. epoch .. ' [batchSize = ' .. opt.batchSize .. ']')
  for t = 1, trainData:size(), opt.batchSize do
    -- disp progress
    xlua.progress(t, trainData:size())

    -- batch fits?
    if (t + opt.batchSize - 1) > trainData:size() then
      break
    end

    -- create mini batch
    local idx = 1
    for i = t,t+opt.batchSize-1 do
      x[idx] = trainData.data[shuffle[i]]
      yt[idx] = trainData.labels[shuffle[i]]
      idx = idx + 1
    end

    --[[print(x:size())
      imagem = x[1][1]:add(0.1)
      imgName = string.format("max%d.png", shuffle[1])
      image.save(imgName, imagem)
      imagem2 = yt[1]:add(-1)
      imgName2 = string.format("gt%d.png", shuffle[1])
      image.save(imgName2, imagem2)
        ]]

    -- create closure to evaluate f(X) and df/dX
    local eval_E = function(w)
      -- reset gradients
      model:zeroGradParameters()

      -- evaluate function for complete mini batch
      x = x:cuda()
      local y = model:forward(x)
      --print(model)
      -- estimate df/dW

      --[[ /DEBUG
      xx = torch.Tensor(1, opt.channels, opt.imHeight, opt.imWidth)
      xx = x
      
      image.save("debug_images/caco" .. t  .. ".png", xx[1][1]:add(0.1))
      labelImg = y[1][1]
      image.save("debug_images/caco" .. t  .. "-2.png", labelImg)
      labelImg = y[1][2]
      image.save("debug_images/caco" .. t  .. "-3.png", labelImg)
      labelImg = yt[1]:clone()
      labelImg:add(-1)
      image.save("debug_images/caco" .. t  .. "-4.png", labelImg)

      -- DEBUG/]]

      yt = yt:cuda()
      y = y:cuda()
      err = loss:forward(y,yt)            -- updateOutput
      local dE_dy = loss:backward(y,yt)   -- updateGradInput
      dE_dy = dE_dy:cuda()
      model:backward(x,dE_dy)
      -- Don't add this to err, so models with different WD
      -- settings can be easily compared. optim functions
      -- care only about the gradient anyway (adam/rmsprop)
      dE_dw:add(opt.weightDecay, w)

      -- return f and df/dX
      return err, dE_dw
    end

    -- optimize on current mini-batch
    local _, errt = optim.adam(eval_E, w, optimState)

    if opt.printNorm == true then
      local norm = opt.learningRate * dE_dw:norm() / w:norm()
      print(string.format('train err: %f, norm : %f epoch: %d   lr: %f  ', errt[1], norm, epoch, opt.learningRate))
    end
    -- update confusion
    if opt.noConfusion == 'all' then
      model:evaluate()
      local y = model:forward(x):transpose(2, 4):transpose(2, 3)
      y = y:reshape(y:numel()/y:size(4), #classes):sub(1, -1, 2, #opt.dataClasses)
      local _, predictions = y:max(2)
      predictions = predictions:view(-1)
      local k = yt:view(-1)
      if opt.dataconClasses then k = k - 1 end
      confusion:batchAdd(predictions, k)
      model:training()
    end

    totalerr = totalerr + err

  end

  -- time taken
  time = sys.clock() - time
  time = time / trainData:size()
  print("\n==> time to learn 1 sample = " .. (time*1000) .. 'ms')

  -- print average error in train dataset
  totalerr = totalerr / (trainData:size()/opt.batchSize)
  print('Train Error: ', totalerr )

  trainError = totalerr
  collectgarbage()
  return confusion, model, loss
end

-- Export:
return train
