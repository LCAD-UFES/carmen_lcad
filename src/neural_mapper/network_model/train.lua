

require 'nn'
require 'image'
require 'optim'
require 'xlua'    -- xlua provides useful tools, like progress bars
require 'torch'   -- torch
require 'cudnn'
require 'cutorch'

require("model")
require("get_data")
require("get1stat")

if not opt then
   print '==> processing options'
   cmd = torch.CmdLine()
   cmd:text()
   cmd:text('FNN Training/Optimization')
   cmd:text()
   cmd:text('Options:')
   cmd:option('-save', 'results', 'subdirectory to save/log experiments in')
   cmd:option('-visualize', false, 'visualize input data and weights during training')
   cmd:option('-plot', false, 'live plot')
   cmd:option('-optimization', 'ADAM', 'optimization method: SGD | ASGD | CG | LBFGS | ADAM | ADAMAX')
   cmd:option('-learningRate', 0.01, 'learning rate at t=0')
   cmd:option('-batchSize',1, 'mini-batch size (1 = pure stochastic)')
   cmd:option('-weightDecay', 0, 'weight decay (SGD only)')
   cmd:option('-momentum', 0, 'momentum (SGD only)')
   cmd:option('-t0', 1, 'start averaging at t0 (ASGD only), in nb of epochs')
   cmd:option('-maxIter', 2, 'maximum nb of iterations for CG and LBFGS')
   cmd:option('-trsize', 10, 'Training size')
   cmd:option('-kittiType','um','Kitti type')
   cmd:text()
   opt = cmd:parse(arg or {})
end
----------------------------------------------------------------------
print '==> defining some tools'

torch.setdefaulttensortype('torch.CudaTensor')

print '==> defining some tools'

-- classes
local classes = {'1','2'}

criterion = cudnn.SpatialCrossEntropyCriterion()

-- CUDA?
-- if opt.type == 'cuda' then
model:cuda()
criterion:cuda()
--end


-- This matrix records the current confusion across classes
confusion = optim.ConfusionMatrix(classes)

-- Log results to files
trainLogger = optim.Logger(paths.concat(opt.save, 'train.log'))
testLogger = optim.Logger(paths.concat(opt.save, 'test.log'))

-- Retrieve parameters and gradients:
-- this extracts and flattens all the trainable parameters of the mode
-- into a 1-dim vector
if model then
   parameters,gradParameters = model:getParameters()
end

-- print(parameters:size())



trsize = opt.trsize

----------------------------------------------------------------------
print '==> configuring optimizer'

if opt.optimization == 'CG' then
   optimState = {
      maxIter = opt.maxIter
   }
   optimMethod = optim.cg

elseif opt.optimization == 'LBFGS' then
   optimState = {
      learningRate = opt.learningRate,
      maxIter = opt.maxIter,
      nCorrection = 10
   }
   optimMethod = optim.lbfgs

elseif opt.optimization == 'SGD' then
   optimState = {
      learningRate = opt.learningRate,
      weightDecay = opt.weightDecay,
      momentum = opt.momentum,
      learningRateDecay = 1e-7
   }
   optimMethod = optim.sgd

elseif opt.optimization == 'ASGD' then
   optimState = {
      eta0 = opt.learningRate,
      t0 = trsize * opt.t0
   }
   optimMethod = optim.asgd

elseif opt.optimization == 'ADAM' then
	optimState = {
	  learningRate = opt.learningRate,
      weightDecay = opt.weightDecay,
      learningRateDecay = 1e-7,
      beta1 = 0.9,
      beta2 = 0.999,
      epsilon = 1e-8
	}
	optimMethod = optim.adam

elseif opt.optimization == 'ADAMAX' then
	print('ADAMAX')
	optimState = {
	  learningRate = opt.learningRate,
      beta1 = 0.9,
      beta2 = 0.999,
      epsilon = 1e-8
	}
	optimMethod = optim.adamax

else
   error('unknown optimization method')
end
------------------------------------------------------------

function train()
   -- epoch tracker
   epoch = epoch or 1

   -- local vars
   local time = sys.clock()

   -- set model to training mode (for modules that differ in training and testing, like Dropout)
   model:training()

   -- do one epoch
   print('==> doing epoch on training data:')
   print("==> online epoch # " .. epoch .. ' [batchSize = ' .. opt.batchSize .. ']')
   for t = 0,trsize,opt.batchSize do
      -- disp progress
      xlua.progress(t, trsize)

   	  -- get batch
      inputs, targets = GetBatchWithType('Dataset', opt.kittiType, t, math.min(t+opt.batchSize-1,trsize-1), 1)

       local f = function (x)
               -- get new parameters
               if x ~= parameters then
                  parameters:copy(x)
               end

               -- reset gradients
               gradParameters:zero()

               -- f is the average of all criterions
               local f = 0
               -- print(inputs[1]:size())
              
               output = model:forward(inputs)
               --print(output:size(2))
               --for i=1, 5 do 
               --        print(output[1][targets[1][1][i]][1][i]) 
               --end
                       
            --   for i = 1,opt.batchSize do      
               err = criterion:forward(output, targets)
               f = f + err

               -- estimate df/dW
               df_do = criterion:backward(output, targets)
              -- print('inputs')
           	  -- print(inputs[i][1][k][l])
           	  -- print('df_do:')
           	--print(df_do)


               -- update confusion
               confusion:add(output, targets)
            
               model:backward(inputs, df_do)
               
               -- normalize gradients and f(X)
               gradParameters:div(#inputs)
               f = f/#inputs

               -- return f and df/dX
               return f,gradParameters
               
          end

      -- optimize on current mini-batch
      if optimMethod == optim.asgd then
         _,_,average = optimMethod(feval, parameters, optimState)
      else

         optimMethod(f, parameters, optimState)
      end
       
    end

       -- time taken
   time = sys.clock() - time
   time = time / trsize
   print("\n==> time to learn 1 sample = " .. (time*1000) .. 'ms')

   -- print confusion matrix
   print(confusion)

   -- update logger/plot
   trainLogger:add{['% mean class accuracy (train set)'] = confusion.totalValid * 100}
   if opt.plot then
      trainLogger:style{['% mean class accuracy (train set)'] = '-'}
      trainLogger:plot()
   end

   -- save/log current model
   local filename = paths.concat(opt.save, 'model.net')
   os.execute('mkdir -p ' .. sys.dirname(filename))
   print('==> saving model to '..filename)
   torch.save(filename, model)

   -- next epoch
   confusion:zero()
   epoch = epoch + 1

end

------------------------------------------------------------------------------
train()





