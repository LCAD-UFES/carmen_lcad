require 'nn' 
require 'image'


function GetBatch(path, ini, fim)

torch.setdefaulttensortype('torch.DoubleTensor')

local inputs = {}
local targets = {}

for j = ini, fim do
	statistics = {}
	local filePathMax = string.format("%s/%06d_max.csv", path, j)
	-- print(filePathMax)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathMax, 'r')
	local header = csvFile:read()

	--- torch.Tensor(LIN, COL)
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

	table.insert(statistics, max)
	csvFile:close()

	local filePathmin = string.format("%s/%s_%06d_min.csv", path, dataType, j)
	-- print(filePathmin)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathmin, 'r')
	local header = csvFile:read()

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


	table.insert(statistics, min)
	csvFile:close()

	local filePathmean = string.format("%s/%s_%06d_mean.csv", path, dataType, j)
	-- print(filePathmean)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathmean, 'r')
	local header = csvFile:read()

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

	table.insert(statistics, mean)
	csvFile:close()

	local filePathmedian = string.format("%s/%s_%06d_median.csv", path, dataType, j)
	-- print(filePathmedian)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathmedian, 'r')
	local header = csvFile:read()

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

	table.insert(statistics, median)
	csvFile:close()

	local filePathstd = string.format("%s/%s_%06d_std.csv", path, dataType, j)
	-- print(filePathstd)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathstd, 'r')
	local header = csvFile:read()

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

	table.insert(statistics, std)
	csvFile:close()

	local filePathgt = string.format("%s/%s_%06d_gt.csv", path, dataType, j)
	-- print(filePathstd)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathgt, 'r')
	local header = csvFile:read()

	--- torch.Tensor(LIN, COL)
	local gt = torch.Tensor(1, 400, 200)

	local i = 0
	for line in csvFile:lines('*l') do
		local l = line:split(',')
		i = i + 1


		for key, val in ipairs(l) do
		    gt[1][i][key] = val
		end
	end

	-- if itorch then
	    -- itorch.image(gt)
	-- else
		-- print('skipping visualization because the script is not run from itorch')
	-- end

	table.insert(targets, gt)
	csvFile:close()

	table.insert(inputs, statistics)

end

-- print(inputs)
-- print(targets)
return inputs, targets
end

function GetBatchOneDimension(path,dataType, ini, fim)
torch.setdefaulttensortype('torch.DoubleTensor')

local inputs = {}
local targets = {}

for j = ini, fim do
	--- torch.Tensor(LIN, COL)
	local statistics = torch.Tensor(1, 400, 200)
	local target = torch.Tensor(1, 400, 200)

	local filePathMax = string.format("%s/%s_%06d_max.csv", path, dataType, j)
	-- print(filePathMax)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathMax, 'r')
	local header = csvFile:read()

	local max = torch.Tensor(1, 400, 200)


	local i = 0
	for line in csvFile:lines('*l') do
		local l = line:split(',')
		i = i + 1


		for key, val in ipairs(l) do
		    max[1][i][key] = val
		end
	end
	csvFile:close()

	statistics[1] = max[1]

	local filePathgt = string.format("%s/%s_%06d_gt.csv", path, dataType, j)
	-- print(filePathstd)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathgt, 'r')
	local header = csvFile:read()

	--- torch.Tensor(LIN, COL)
	local gt = torch.Tensor(1, 400, 200)

	local i = 0
	for line in csvFile:lines('*l') do
		local l = line:split(',')
		i = i + 1


		for key, val in ipairs(l) do
		    gt[1][i][key] = val
		end
	end

	---- if itorch then
	--    -- itorch.image(gt)
	---- else
		-- print('skipping visualization because the script is not run from itorch')
	---- end

	target[1] = gt[1]
	csvFile:close()

	table.insert(inputs, statistics)
	table.insert(targets, target)

end

-- print(inputs)
-- print(targets)

return inputs, targets
end

--- retorna um banco de dados pego (csv de ini, fin)
function GetBatchWithType(path,dataType, ini, fim, cuda)

if cuda == 1 then
  torch.setdefaulttensortype('torch.CudaTensor')
else
  torch.setdefaulttensortype('torch.DoubleTensor')
end

inputs = torch.Tensor((fim-ini)+1, 5, 400, 200)
targets = torch.Tensor((fim-ini)+1, 400, 200)

for j = ini, fim do

	--- torch.Tensor(LIN, COL)
	local statistics = torch.Tensor(5, 400, 200)
	local target = torch.Tensor(1, 400, 200)

	local filePathMax = string.format("%s/%s_%06d_max.csv", path, dataType, j)
	-- print(filePathMax)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathMax, 'r')
	local header = csvFile:read()

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
	local header = csvFile:read()

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
	local header = csvFile:read()

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
	local header = csvFile:read()

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
	local header = csvFile:read()

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

	local filePathgt = string.format("%s/%s_%06d_gt.csv", path, dataType, j)
	-- print(filePathstd)
	-- Read data from CSV to tensor
	local csvFile = io.open(filePathgt, 'r')
	local header = csvFile:read()

	--- torch.Tensor(LIN, COL)
	local gt = torch.Tensor(1, 400, 200)

	local i = 0
	for line in csvFile:lines('*l') do
		local l = line:split(', ')
		i = i + 1


		for key, val in ipairs(l) do
                    if(tonumber(val) < 0 or tonumber(val) >= 2) then
                        print(val)
                    end
		    gt[1][i][key] = (val+1)
		end
	end

	--if itorch then
	--    itorch.image(gt)
       -- else
	--   print('skipping visualization because the script is not run from itorch')
	--end

	csvFile:close()

	inputs[j+1] = statistics
	targets[j+1] = gt[1]

end


return inputs, targets
end

inputs, targets = GetBatchWithType('Dataset', 'um', 0, 2)


