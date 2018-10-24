----------------------------------------------------------------------
-- This script implements a test procedure, to report accuracy
-- on the test data.
--
-- Written by  : Abhishek Chaurasia, Eugenio Culurcielo
-- Dated       : January 2016
-- Last updated: June, 2016
----------------------------------------------------------------------

require 'xlua'    -- xlua provides useful tools, like progress bars
require 'optim'   -- an optimization package, for online and batch methods

torch.setdefaulttensortype('torch.FloatTensor')

----------------------------------------------------------------------
-- Logger:
errorLogger = optim.Logger(paths.concat(opt.save, 'error.log'))
coTotalLogger = optim.Logger(paths.concat(opt.save, 'confusionTotal.log'))
coAveraLogger = optim.Logger(paths.concat(opt.save, 'confusionAvera.log'))
coUnionLogger = optim.Logger(paths.concat(opt.save, 'confusionUnion.log'))

print '==> defining test procedure'
local teconfusion, filename

if opt.dataconClasses then
   teconfusion = optim.ConfusionMatrix(opt.dataconClasses)
else
   teconfusion = optim.ConfusionMatrix(opt.dataClasses)
end

-- Batch test:
local x = torch.Tensor(opt.batchSize, opt.channels, opt.imHeight, opt.imWidth)
local yt = torch.Tensor(opt.batchSize, opt.labelHeight, opt.labelWidth)
x = x:cuda()
yt = yt:cuda()

-- test function
function test(testData, classes, epoch, trainConf, model, loss )
   ----------------------------------------------------------------------
   -- local vars
   local time = sys.clock()
   -- total loss error
   local err = 0
   local totalerr = 0
  
   -- This matrix records the current confusion across classes

   model:evaluate()

   -- test over test data
   print('==> Testing:')
   for t = 1, testData:size(), opt.batchSize do
      -- disp progress
      xlua.progress(t, testData:size())

      -- batch fits?
      if (t + opt.batchSize - 1) > testData:size() then
         break
      end

      -- create mini batch
      local idx = 1
      for i = t,t+opt.batchSize-1 do
         x[idx] = testData.data[i]
         yt[idx] = testData.labels[i]
         idx = idx + 1
      end

      -- test sample
      local y = model:forward(x)

      err = loss:forward(y,yt)
      if opt.noConfusion == 'tes' or opt.noConfusion == 'all' then
         local y = y:transpose(2, 4):transpose(2, 3)
         y = y:reshape(y:numel()/y:size(4), #classes):sub(1, -1, 2, #opt.dataClasses)
         local _, predictions = y:max(2)
         predictions = predictions:view(-1)
         local k = yt:view(-1)
         if opt.dataconClasses then k = k - 1 end
         teconfusion:batchAdd(predictions, k)
      end
      
      -- /DEBUG
      if epoch % 5 == 0 then
        print("Measuring statistics")

        xx = torch.Tensor(1, opt.channels, opt.imHeight, opt.imWidth)
        xx = x
        image.save("debug_images/input-" .. t .. ".png", xx[1][1]:add(0.1))
        
        tp = 0
        tn = 0
        fp = 0
        fn = 0
        for batchInd = 1, opt.batchSize do
			local _,predict = y[batchInd]:max(1)
			predict = predict:float()
			print(predict)
		    predictImg = torch.div(predict, 3)
		    labelImg = torch.div(yt[batchInd], 3)
		    
        --[[             
          for i=1, opt.imHeight do
            for j = 1, opt.imWidth do
              pointVect = torch.Tensor(3)
              pointVect[1] = y[batchInd][1][i][j]
			  pointVect[2] = y[batchInd][2][i][j]
              pointVect[3] = y[batchInd][3][i][j]
              predictVal, predictInd = pointVect:max()
              print(predictVal, predictInd)
              predict[batchInd][i][j] = predictInd
              predictImg[i][j] = predictInd*3
              labelImg[i][j] = yt[batchInd][i][j]*3
              ytn = (yt[batchInd][i][j] - 1)

              --positive cases
              if predict[batchInd][i][j] == 1 then
                -- true positive
                
                if ytn < 0 then
                  if ytn > 1 then
                    print(ytn)
                  end
                end
                
                if predict[batchInd][i][j] == ytn then
                  tp = tp + 1
                -- false positive
                else
                  fp = fp + 1
                end
              -- negative cases
              else
                -- true negative
                if predict[batchInd][i][j] == ytn then
                  tn = tn + 1
                -- false negative
                else
                  fn = fn + 1
                end
              end
            end
          end]]
          
          image.save("debug_images/predict-" .. t  .. "-" .. epoch ..".png", predictImg)
          image.save("debug_images/gt-" .. t  .. ".png", labelImg)
        end
	  --[[
      b = 1
      precission = tp/(tp+fp)
      recall = tp/(tp + fn)
      acc = (tp + tn)/(tp + tn + fp + fn)
      fmeasure = (1 + (b*b))*precission*recall/((b*b*precission)+ recall)
      print("================ MEASURES:")
      print("Recall = " .. recall)
      print("Precission = " .. precission)
      print("Accuracy = " .. acc)
      print("Fmesure = " .. fmeasure)
      
      numPixels = (opt.imHeight * opt.imWidth)
      print("tp = " .. (tp*100/numPixels) .. " %" .. " fp = " .. (fp*100/numPixels) .. " %" .. " tn = " .. (tn*100/numPixels) .. " %" .. " fn = " .. (fn*100/numPixels) .. " %")
      ]]
      end
      -- DEBUG/
      
      totalerr = totalerr + err
      collectgarbage()
   end

   -- timing
   time = sys.clock() - time
   time = time / testData:size()
   print("\n==> time to test 1 sample = " .. (time*1000) .. 'ms')


   -- print average error in train dataset
   totalerr = totalerr / (testData:size() / opt.batchSize)
   print('Test Error: ', totalerr )
   -- save/log current net
   errorLogger:add{['Training error'] = trainError,
                   ['Testing error'] = totalerr}
   if opt.plot then
      errorLogger:style{['Training error'] = '-',
      ['Testing error'] = '-'}
      errorLogger:plot()
   end
   if totalerr < testData.preverror then
      filename = paths.concat(opt.save, 'model-best.net')
      print('==> saving model to '..filename)

      torch.save(filename, model:clearState())
      -- update to min error:
      if opt.noConfusion == 'tes' or opt.noConfusion == 'all' then
         coTotalLogger:add{['confusion total accuracy'] = teconfusion.totalValid * 100 }
         coAveraLogger:add{['confusion average accuracy'] = teconfusion.averageValid * 100 }
         coUnionLogger:add{['confusion union accuracy'] = teconfusion.averageValid * 100 }

         filename = paths.concat(opt.save,'confusion-'..epoch..'.t7')
         print('==> saving confusion to '..filename)
         torch.save(filename,teconfusion)

         filename = paths.concat(opt.save, 'confusionMatrix-best.txt')
         print('==> saving confusion matrix to ' .. filename)
         local file = io.open(filename, 'w')
         file:write("--------------------------------------------------------------------------------\n")
         file:write("Training:\n")
         file:write("================================================================================\n")
         file:write(tostring(trainConf))
         file:write("\n--------------------------------------------------------------------------------\n")
         file:write("Testing:\n")
         file:write("================================================================================\n")
         file:write(tostring(teconfusion))
         file:write("\n--------------------------------------------------------------------------------")
         file:close()
      end
      filename = paths.concat(opt.save, 'best-number.txt')
      local file = io.open(filename, 'w')
      file:write("----------------------------------------\n")
      file:write("Best test error: ")
      file:write(tostring(totalerr))
      file:write(", in epoch: ")
      file:write(tostring(epoch))
      file:write("\n----------------------------------------\n")
      file:close()
      if totalerr < testData.preverror then testData.preverror = totalerr end
   end
   -- cudnn.convert(model, nn)
   local filename = paths.concat(opt.save, 'model-'..epoch..'.net')
   --os.execute('mkdir -p ' .. sys.dirname(filename))
   print('==> saving model to '..filename)
   torch.save(filename, model:clearState())
   if opt.noConfusion == 'tes' or opt.noConfusion == 'all' then
      -- update to min error:
      filename = paths.concat(opt.save, 'confusionMatrix-' .. epoch .. '.txt')
      print('==> saving confusion matrix to ' .. filename)
      local file = io.open(filename, 'w')
      file:write("--------------------------------------------------------------------------------\n")
      file:write("Training:\n")
      file:write("================================================================================\n")
      file:write(tostring(trainConf))
      file:write("\n--------------------------------------------------------------------------------\n")
      file:write("Testing:\n")
      file:write("================================================================================\n")
      file:write(tostring(teconfusion))
      file:write("\n--------------------------------------------------------------------------------")
      file:close()
      filename = paths.concat(opt.save, 'confusion-'..epoch..'.t7')
      print('==> saving test confusion object to '..filename)
      torch.save(filename,teconfusion)
      --resetting confusionMatrix
      trainConf:zero()
      teconfusion:zero()
   end
   if totalerr < testData.preverror then testData.preverror = totalerr end

   print('\n') -- separate epochs

   collectgarbage()
end

-- Export:
return test
