require 'nn' 
require 'image'


--- retorna um banco de dados pego (csv de ini, fin)
function GetBatchWithType(path,dataType, ini, fim)
   
   torch.setdefaulttensortype('torch.FloatTensor')
   
   local inputs = torch.Tensor((fim-ini)+1, 5, 400, 200)
   local targets = torch.Tensor((fim-ini)+1, 1, 400, 200)
   
   c = 1
   for j = ini, fim do
       local classMap = {
                     [-1] = {1},
                     [0]  =  {1}, -- not road
                     [1]  =  {2},
                     [2]  =  {1},
                     [3]  =  {1}, -- road
                                 }
   
   
   	--- torch.Tensor(LIN, COL)
   	local statistics = torch.Tensor(5, 400, 200)
   	local target = torch.Tensor(1, 400, 200)
   
   	local filePathMax = string.format("%s/%s_%06d_max.csv", path, dataType, j)
   	-- print(filePathMax)
   	-- Read data from CSV to tensor
   	local csvFile = io.open(filePathMax, 'r')
   
   	local max = torch.Tensor(1, 400, 200)
   
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(',')
   		i = i + 1
   
   
   		for key, val in ipairs(l) do
   		    max[1][i][key] = val
   		end
   	end
   
   	-- if itorch then
   	    -- itorch.image(max)
   	-- else
   		-- print('skipping visualization because the script is not run from itorch')
   	-- end
   	csvFile:close()
   
   	statistics[1] = max[1]
   
   	local filePathmin = string.format("%s/%s_%06d_min.csv", path, dataType, j)
   	-- print(filePathmin)
   	-- Read data from CSV to tensor
   	local csvFile = io.open(filePathmin, 'r')
   
   	--- torch.Tensor(LIN, COL)
   	local min = torch.Tensor(1, 400, 200)
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(',')
   		i = i + 1
   
   
   		for key, val in ipairs(l) do
   		    min[1][i][key] = val
   		end
   	end
   
   	-- if itorch then
   	    -- itorch.image(min)
   	-- else
   		-- print('skipping visualization because the script is not run from itorch')
   	-- end
   
   
   	statistics[2] = min[1]
   	csvFile:close()
   
   	local filePathmean = string.format("%s/%s_%06d_mean.csv", path, dataType, j)
   	-- print(filePathmean)
   	-- Read data from CSV to tensor
   	local csvFile = io.open(filePathmean, 'r')
   
   	--- torch.Tensor(LIN, COL)
   	local mean = torch.Tensor(1, 400, 200)
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(',')
   		i = i + 1
   
   		for key, val in ipairs(l) do
   		    mean[1][i][key] = val
   		end
   	end
   
   	-- if itorch then
   	    -- itorch.image(mean)
   	-- else
   		-- print('skipping visualization because the script is not run from itorch')
   	-- end
   
   	statistics[3] = mean[1]
   	csvFile:close()
   
   	local filePathmedian = string.format("%s/%s_%06d_median.csv", path, dataType, j)
   	-- print(filePathmedian)
   	-- Read data from CSV to tensor
   	local csvFile = io.open(filePathmedian, 'r')
   
   	--- torch.Tensor(LIN, COL)
   	local median = torch.Tensor(1, 400, 200)
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(',')
   		i = i + 1
   
   
   		for key, val in ipairs(l) do
   		    median[1][i][key] = val
   		end
   	end
   
   	-- if itorch then
   	    -- itorch.image(median)
   	-- else
   		-- print('skipping visualization because the script is not run from itorch')
   	-- end
   
   	statistics[4] = median[1]
   	csvFile:close()
   
   	local filePathstd = string.format("%s/%s_%06d_std.csv", path, dataType, j)
   	-- print(filePathstd)
   	-- Read data from CSV to tensor
   	local csvFile = io.open(filePathstd, 'r')
   
   	--- torch.Tensor(LIN, COL)
   	local std = torch.Tensor(1, 400, 200)
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(',')
   		i = i + 1
   
   
   		for key, val in ipairs(l) do
   		    std[1][i][key] = val
   		end
   	end
   
   	-- if itorch then
   	    -- itorch.image(std)
   	-- else
   		-- print('skipping visualization because the script is not run from itorch')
   	-- end
   
   	statistics[5] = std[1]
   	csvFile:close()
   
   	local filePathgt = string.format("%s/%s_%06d_gt.png", path, dataType, j)
   	-- print(filePathstd) 
       -- label image data are resized to be [1,nClasses] in [0 255] scale:
       local labelIn = image.load(filePathgt, 1, 'byte')
   
       local labelFile = image.scale(labelIn, 200, 400, 'simple'):float()
   
       labelFile:apply(function(x) return classMap[x][1] end)
       --print(labelIn)
   
   
   --[[
   	--- torch.Tensor(LIN, COL)
   	local gt = torch.Tensor(1, 400, 200)
   
   	local i = 0
   	for line in csvFile:lines('*l') do
   		local l = line:split(', ')
   		i = i + 1
   
   
   		for key, val in ipairs(l) do
               --print('_' .. val .. '_')
   		    gt[1][i][key] = val
               --print(gt[1][i][key])
   		end
   	end
   
   	---- if itorch then
   	--    -- itorch.image(gt)
   	---- else
   		-- print('skipping visualization because the script is not run from itorch')
   	---- end
       ]]
   	target[1] = labelFile
   
   	inputs[c] = statistics
   	targets[c] = target
   	c = c + 1
   end
   
   
   return inputs, targets
end

-- inputs, targets = GetBatchWithType('Dataset', 'um', 0, 2)

