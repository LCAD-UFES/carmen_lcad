----------------------------------------------------------------------
-- Neural Mapper data loader,
-- Andre Seidel,
-- January 2018
----------------------------------------------------------------------

require 'torch'   -- torch
require 'image'   -- to visualize the dataset
require 'nn' 
require 'math'

--- retorna um banco de dados pego (csv de ini, fin)
function getSample(path, sideSize, pos)

	torch.setdefaulttensortype('torch.FloatTensor')

	print(size)
	local inputs = torch.Tensor(5, sideSize, sideSize)
	local target = torch.Tensor(1, sideSize, sideSize)

	local filePathMax = string.format("%s/data/%d_max.png", path, pos)
	local max = image.load(filePathMax, 1, 'float')
	-- image.save("teste.png", max[1])
	inputs[1] = max

	local filePathmin = string.format("%s/data/%d_min.png", path, pos)
	local min = image.load(filePathmin, 1, 'float')
	inputs[2] = min

	local filePathmean = string.format("%s/data/%d_mean.png", path, pos)
	local mean = image.load(filePathmean, 1, 'float')
	inputs[3] = mean

	local filePathnumb = string.format("%s/data/%d_numb.png", path, pos)
	local numb = image.load(filePathnumb, 1, 'float')
	inputs[4] = numb

	local filePathstd = string.format("%s/data/%d_std.png", path, pos)
	local std = image.load(filePathstd, 1, 'float')
	inputs[5] = std

	local filePathgt = string.format("%s/labels/%d_label.png", path, pos)
	local label = image.load(filePathgt, 1, 'byte')
	target[1] = label
	image.save("teste.png", label)
	print(label)
	
	return inputs, target
end

-- Local repo files

local opts = require '../opts'

-- Get the input arguments parsed and stored in opt
opt = opts.parse(arg)

torch.setdefaulttensortype('torch.FloatTensor')

sideSize = math.floor(opt.velodyneRadius*math.sqrt(2)/(0.2) + 0.5)
----------------------------------------------------------------------
-- kitti dataset:

local trsize, tesize

-- 80% of the images for training
trsize = 3--227 -- kitti train images
tesize = 1--59  -- kitti validation images

local classes = {'unknown','available', 'obstacle'}
local conClasses = {'unknown','available', 'obstacle'} -- 3 classes

local nClasses = #classes

--------------------------------------------------------------------------------

-- From here #class will give number of classes even after shortening the list
-- nClasses should be used to get number of classes in original list

-- saving training histogram of classes
local histClasses = torch.Tensor(#classes):zero()

print('==> number of classes: ' .. #classes)
print('classes are:')
print(classes)

--------------------------------------------------------------------------------
print '==> loading Neural Mapper dataset'
local trainData, testData
local loadedFromCache = false
paths.mkdir(paths.concat(opt.cachepath, 'neuralMapper'))
local neuralMapperCachePath = paths.concat(opt.cachepath, 'neuralMapper', 'data.t7')

if opt.cachepath ~= "none" and paths.filep(neuralMapperCachePath) then
   local dataCache = torch.load(neuralMapperCachePath)
   trainData = dataCache.trainData
   testData = dataCache.testData
   histClasses = dataCache.histClasses
   loadedFromCache = true
   dataCache = nil
   collectgarbage()
else
   -- initialize data structures:
   trainData = {
      data = torch.FloatTensor(trsize, opt.channels, sideSize, sideSize),
      labels = torch.FloatTensor(trsize, sideSize, sideSize),
      preverror = 1e10, -- a really huge number
      size = function() return trsize end
   }

   testData = {
      data = torch.FloatTensor(tesize, opt.channels, sideSize, sideSize),
      labels = torch.FloatTensor(tesize, sideSize, sideSize),
      preverror = 1e10, -- a really huge number
      size = function() return tesize end
   }

   print('==> loading training files');

   local dpathRoot = opt.datapath
   print('loading from: ' .. dpathRoot)
   
   assert(paths.dirp(dpathRoot), 'No training folder found at: ' .. opt.datapath)
   --load training and testingimages and labels:
	for i=1, trsize do--93 do
	  trainData.data[i], labelFile = getSample(dpathRoot,sideSize,i)
	  --labelFile:apply(function(x) return classMap[x][1] end)
	  -- Syntax: histc(data, bins, min, max)
	  histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)
	  
	  trainData.labels[i] = labelFile
	  xlua.progress(i, trsize)
	  collectgarbage()
	end

	print('==> loading testing files');

	for i= 1, tesize do--38, 96 do
		testData.data[i], labelFile = getSample(dpathRoot,sideSize,i)
		--labelFile:apply(function(x) return classMap[x][1] end)
		-- Syntax: histc(data, bins, min, max)
		histClasses = histClasses + torch.histc(labelFile, #classes, 1, #classes)

		testData.labels[i] = labelFile
		xlua.progress(i, tesize)
		collectgarbage()
	end
end

if opt.cachepath ~= "none" and not loadedFromCache then
   print('==> saving data to cache: ' .. neuralMapperCachePath)
   local dataCache = {
      trainData = trainData,
      testData = testData,
      histClasses = histClasses
   }
   torch.save(neuralMapperCachePath, dataCache)
   dataCache = nil
   collectgarbage()
end

----------------------------------------------------------------------
print '==> verify statistics'

-- It's always good practice to verify that data is properly
-- normalized.

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
print(filePath)
print(classes_td)

local file = io.open(filePath, 'w')
print(file)
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
