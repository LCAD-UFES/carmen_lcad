----------------------------------------------------------------------
-- kitti data loader,
-- Andre Seidel,
-- January 2018
----------------------------------------------------------------------

require 'torch'   -- torch
require 'image'   -- to visualize the dataset
require("data/open_kitti_files")

-- Local repo files
local opts = require '../opts'

-- Get the input arguments parsed and stored in opt
opt = opts.parse(arg)

torch.setdefaulttensortype('torch.FloatTensor')
----------------------------------------------------------------------
-- kitti dataset:

local trsize, tesize

-- 80% of the images for training
trsize = 5--227 -- kitti train images
tesize = 1--59  -- kitti validation images

local classes = {'not-road','Road'}
local conClasses = {'Road'} -- 19 classes

local nClasses = #classes

--------------------------------------------------------------------------------

-- Ignoring unnecessary classes
print '==> remapping classes'
local classMap = {

                  [0]  =  {1}, -- not road
                  [1]  =  {2}, -- road
                              }

-- From here #class will give number of classes even after shortening the list
-- nClasses should be used to get number of classes in original list

-- saving training histogram of classes
local histClasses = torch.Tensor(#classes):zero()

print('==> number of classes: ' .. #classes)
print('classes are:')
print(classes)

--------------------------------------------------------------------------------
print '==> loading kitti dataset'
local trainData, testData
local loadedFromCache = false
paths.mkdir(paths.concat(opt.cachepath, 'kitti'))
local kittiCachePath = paths.concat(opt.cachepath, 'kitti', 'data.t7')

if opt.cachepath ~= "none" and paths.filep(kittiCachePath) then
   local dataCache = torch.load(kittiCachePath)
   trainData = dataCache.trainData
   testData = dataCache.testData
   histClasses = dataCache.histClasses
   loadedFromCache = true
   dataCache = nil
   collectgarbage()
else
   -- initialize data structures:
   trainData = {
      data = torch.FloatTensor(trsize, opt.channels, opt.imHeight, opt.imWidth),
      labels = torch.FloatTensor(trsize, opt.labelHeight, opt.labelWidth),
      preverror = 1e10, -- a really huge number
      size = function() return trsize end
   }

   testData = {
      data = torch.FloatTensor(tesize, opt.channels, opt.imHeight, opt.imWidth),
      labels = torch.FloatTensor(tesize, opt.labelHeight, opt.labelWidth),
      preverror = 1e10, -- a really huge number
      size = function() return tesize end
   }

   print('==> loading training files');

   local dpathRoot = opt.datapath
   print('loading from: ' .. dpathRoot)
   
   assert(paths.dirp(dpathRoot), 'No training folder found at: ' .. opt.datapath)
   --load training and testingimages and labels:
   j=1
   for i=0, 4 do--93 do
      trainData.data[j], labelFile = GetBatchWithType(dpathRoot,'um',i,i)
      --labelFile:apply(function(x) return classMap[x][1] end)
      -- Syntax: histc(data, bins, min, max)
      histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)
      
      trainData.labels[j] = labelFile
      j = j + 1
      xlua.progress(j, trsize)
      collectgarbage()
   end
   --[[
   for i=0, 94 do
      trainData.data[j], labelFile = GetBatchWithType(dpathRoot,'umm',i,i)
      --labelFile:apply(function(x) return classMap[x][1] end)
      -- Syntax: histc(data, bins, min, max)
      histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)
      
      trainData.labels[j] = labelFile
      j = j + 1
      xlua.progress(j, trsize)
      collectgarbage()
   end
   for i=0, 37 do
      trainData.data[j], labelFile = GetBatchWithType(dpathRoot,'uu',i,i)
      --labelFile:apply(function(x) return classMap[x][1] end)
      -- Syntax: histc(data, bins, min, max)
      histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)
      
      trainData.labels[j] = labelFile
      j = j + 1
      xlua.progress(j, trsize)
      collectgarbage()
   end
   ]]
   print('==> loading testing files');
   
   j = 1
   for i= 0, 0 do--38, 96 do
      testData.data[j], labelFile = GetBatchWithType(dpathRoot,'um',i,i)
      --labelFile:apply(function(x) return classMap[x][1] end)
      -- Syntax: histc(data, bins, min, max)
      histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)
      
      testData.labels[j] = labelFile
      j = j + 1
      xlua.progress(j, tesize)
      collectgarbage()
   end
end

if opt.cachepath ~= "none" and not loadedFromCache then
   print('==> saving data to cache: ' .. kittiCachePath)
   local dataCache = {
      trainData = trainData,
      testData = testData,
      histClasses = histClasses
   }
   torch.save(kittiCachePath, dataCache)
   dataCache = nil
   collectgarbage()
end

----------------------------------------------------------------------
print '==> verify statistics'

-- It's always good practice to verify that data is properly
-- normalized.

--for image = 1, trsize do
--   for c = 1, opt.channels do
--      for i = 1, opt.labelHeight do
--         for j = 1, opt.labelWidth do
--            if math.abs(trainData.data[image][c][i][j]) > 10.0 then
--               print("image " .. image .. ", c " .. c .. ", i " .. i .. ", j " .. j .. ", td " .. trainData.data[image][c][i][j])
--            end
--         end
--      end
--   end
--end

local siz = trainData.data:size()
print("trainData.data:size() = ")
print(siz)
for i = 1, opt.channels do
   local trainMean = trainData.data[{ {},i }]:mean()
   local trainStd = trainData.data[{ {},i }]:std()

   local testMean = testData.data[{ {},i }]:mean()
   local testStd = testData.data[{ {},i }]:std()

   print('training data, channel-'.. i ..', mean: ' .. trainMean)
   print('training data, channel-'.. i ..', standard deviation: ' .. trainStd)

   print('test data, channel-'.. i ..', mean: ' .. testMean)
   print('test data, channel-'.. i ..', standard deviation: ' .. testStd)
end

----------------------------------------------------------------------

local classes_td = {[1] = 'classes,targets\n'}
for _,cat in pairs(classes) do
   table.insert(classes_td, cat .. ',1\n')
end

filePath = paths.concat(opt.save, 'categories.txt')

local file = io.open(filePath, 'w')
file:write(table.concat(classes_td))
file:close()

-- Exports
opt.dataClasses = classes
opt.dataconClasses  = conClasses
opt.datahistClasses = histClasses

return {
   trainData = trainData,
   testData = testData,
   mean = trainMean,
   std = trainStd
}
