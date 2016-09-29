require 'torch'
require 'nn'
require 'cutorch'
require 'cudnn'
require 'image'
require 'optim'
require 'pl'

--
-- Global vars
--

opt = lapp[[
   -p,--prepair_samples                        prepair samples using pre-trained cnn
   -n,--num_samples      (default 5)           number of samples to prepair (only useful with -p)
   -d,--display_images                         display image samples during preparation (only useful with -p)
   -r,--learning_rate    (default 0.05)        learning rate
   -m,--momentum         (default 0.5)         momentum
   -e,--num_epochs       (default 500)         number of epochs
   -s,--sanity                                 sanity test (train and test with training set)
   -t,--threads          (default 4)           number of threads
   -a,--allowed_distance (default 5)           maximum allowed distance from target image (in images)
]]

--
-- Global settings
--
local net_utils = {}

-- fix seed
torch.manualSeed(1)

-- threads
torch.setnumthreads(opt.threads)
print('Set number of threads to ' .. torch.getnumthreads())

-- use floats (good for SGD)
torch.setdefaulttensortype('torch.FloatTensor')


------------------------------------------------------------------------------
-- takes a batch of images and preprocesses them
-- VGG-16 network is hardcoded, as is 224 as size to forward
local function prepro(imgs, data_augment, on_gpu)
  assert(data_augment ~= nil, 'pass this in. careful here.')
  assert(on_gpu ~= nil, 'pass this in. careful here.')

  local h,w = imgs:size(3), imgs:size(4)
  local cnn_input_size = 224

  -- cropping data augmentation, if needed
  if h > cnn_input_size or w > cnn_input_size then 
    local xoff, yoff
    if data_augment then
      xoff, yoff = torch.random(w-cnn_input_size), torch.random(h-cnn_input_size)
    else
      -- sample the center
      xoff, yoff = math.ceil((w-cnn_input_size)/2), math.ceil((h-cnn_input_size)/2)
    end
    -- crop.
    imgs = imgs[{ {}, {}, {yoff,yoff+cnn_input_size-1}, {xoff,xoff+cnn_input_size-1} }]
  end

  -- ship to gpu or convert from byte to float
  if on_gpu then
    print("Conversion to float on gpu") 
    imgs = imgs:cuda() 
  else 
    print("Conversion to float on cpu") 
    imgs = imgs:float() 
  end

  -- lazily instantiate vgg_mean
  if not net_utils.vgg_mean then
    net_utils.vgg_mean = torch.FloatTensor{123.68, 116.779, 103.939}:view(1,3,1,1) -- in RGB order
  end
  net_utils.vgg_mean = net_utils.vgg_mean:typeAs(imgs) -- a noop if the types match

  -- subtract vgg mean
  imgs:add(-1, net_utils.vgg_mean:expandAs(imgs))

  return imgs
end


local function load_images_and_labels(csv_name, year, dataset_prefix)
  local csv = require("csv")
  local f = csv.open(csv_name)
  local i = -1
  local x, y, px, py, img_file_name
  local num_imgs = opt.num_samples
  local imgs = torch.Tensor(num_imgs, 3, 224, 224)
  local labels = torch.Tensor(num_imgs)
  for fields in f:lines() do
    if i ~= -1 then
      local label
      for j, v in ipairs(fields) do
        if j == 1 then img_file_name = year .. v .. ".bb08.l.png" end 
        if j == 2 then x = tonumber(v) end 
        if j == 3 then y = tonumber(v) end 
        if j == 4 then label = tonumber(v) end 
      end
      if i >= 1 and i <= num_imgs then
        print(img_file_name, ' dist = ', math.sqrt((x - px) * (x - px) + (y - py) * (y - py)))
        local img = image.load(img_file_name, 3, 'byte')
        img = image.scale(img, 224, 224, 'bilinear')
        imgs[i] = img
        labels[i] = label
        if opt.display_images then
          image.display{image = imgs[i], zoom = 1, legend = 'image ' .. tostring(i)}
        end
      end
      px = x
      py = y
    end
    i = i + 1
  end
  
  --imgs = prepro(imgs, false, true) -- preprocess in place, and don't augment
  return imgs, labels
end -- load_images_and_labels()


local function compute_cnn_output_for_images(images)
  local batch_size = 10
  local images_batch = images:split(batch_size, 1)
  local cnn_output
  local j = 1
  for i = 1, images:size(1), batch_size do
    xlua.progress(i, images:size(1)) -- Data preparation progress bar 
    local images_batch_i = images_batch[j]:clone()
    images_batch_i = images_batch_i:cuda()
    local cnn_output_i
    if j == 1 then
      cnn_output_i = cnn_model:forward(images_batch_i)
      cnn_output = cnn_output_i:clone()
    else
      cnn_output_i = cnn_model:forward(images_batch_i)
      local cnn_output_ic = cnn_output_i:clone()
      cnn_output = torch.cat(cnn_output, cnn_output_ic, 1)
    end
    j = j + 1
  end

  return cnn_output
end -- compute_cnn_output_for_images()


local function prepair_samples_using_cnn(csv_name, year, dataset_prefix)
  print("Generating samples for " .. dataset_prefix .. " " .. csv_name)
  local images, labels = load_images_and_labels(csv_name, year, dataset_prefix)
  local cnn_output = compute_cnn_output_for_images(images)
  torch.save(dataset_prefix .. "_images.tensor", images)
  torch.save(dataset_prefix .. "_labels.tensor", labels)
  torch.save(dataset_prefix .. "_cnn_out.tensor", cnn_output)
  
  if opt.display_images then
    for i = 1, images:size(1), 1 do
      local cnn_output_ret = torch.reshape(cnn_output[i], 24, 32)
      --local cnn_output_image = image.scale(cnn_output_ret:double(), 32*8, 24*8, 'simple')
      
      image.display{image = images[i], zoom = 1, legend = 'image ' .. tostring(i)}
      image.display{image = cnn_output_ret, zoom = 8, legend = 'model ' .. tostring(i)}
    end
  end
end -- prepair_samples_using_cnn()


local function prepair_samples_using_cnn_new(images_file_name, labels_file_name)
  print("Generating samples for " .. images_file_name .. " and " .. labels_file_name)
  local images = torch.load(images_file_name)
  local labels = torch.load(labels_file_name)
  local cnn_output = compute_cnn_output_for_images(images)
  torch.save(images_file_name .. "_images.tensor", images)
  torch.save(labels_file_name .. "_labels.tensor", labels)
  torch.save(images_file_name .. "_cnn_out.tensor", cnn_output)
  
  if opt.display_images then
    for i = 1, images:size(1), 1 do
      local cnn_output_ret = torch.reshape(cnn_output[i], 24, 32)
      --local cnn_output_image = image.scale(cnn_output_ret:double(), 32*8, 24*8, 'simple')
      
      image.display{image = images[i], zoom = 1, legend = 'image ' .. tostring(i)}
      image.display{image = cnn_output_ret, zoom = 8, legend = 'model ' .. tostring(i)}
    end
  end
end -- prepair_samples_using_cnn_new()


local function build_net(cnn_output, labels)
  -- define model to train
  model = nn.Sequential()
  model:add(nn.Linear(cnn_output:size(2), labels:max()))
  model:add(nn.LogSoftMax())
  
  -- a negative log-likelihood criterion for multi-class classification
  criterion = nn.ClassNLLCriterion()
end -- build_net()

function file_exists(name)
   local f=io.open(name,"r")
   if f~=nil then io.close(f) return true else return false end
end

local function train_net(batchInputs, batchLabels)
  if file_exists("trained_net.tensor") then
    model = torch.load("trained_net.tensor")
    print("ATENTION: loading trained_net.tensor instead of training!")
    print("Delete this file if you want to train.")
    return
  end
      
-- https://github.com/torch/nn/blob/master/doc/training.md
  local optimState = {learningRate = opt.learning_rate, momentum = opt.momentum, learningRateDecay = 5e-7}
  --model = model:cuda()
  --batchInputs = batchInputs:cuda();
  --batchLabels = batchLabels:cuda();
  --criterion = criterion:cuda();
  
  local params, gradParams = model:getParameters()
  
  for epoch=1,opt.num_epochs do
    xlua.progress(epoch, opt.num_epochs) -- Show traning progress bar 
    -- local function we give to optim
    -- it takes current weights as input, and outputs the loss
    -- and the gradient of the loss with respect to the weights
    -- gradParams is calculated implicitly by calling 'backward',
    -- because the model's weight and bias gradient tensors
    -- are simply views onto gradParams
    local function feval(params)
      gradParams:zero()
--      print("i ", batchInputs:size())
      local outputs = model:forward(batchInputs)
      --outputs = outputs:cuda();
--      print("o ", outputs:size())
--      print("l ", batchLabels:size())
      local loss = criterion:forward(outputs, batchLabels)
      local dloss_doutput = criterion:backward(outputs, batchLabels)
      --dloss_doutput = dloss_doutput:cuda()
      model:backward(batchInputs, dloss_doutput)
      return loss,gradParams
    end
    --feval(params)
    optim.sgd(feval, params, optimState)
    --print("epoch = ", epoch)
  end
  torch.save("trained_net.tensor", model)
end -- train_net()


local function evaluate_resuts(results, labels)  
  -- this matrix records the current confusion across classes
  local confusion = optim.ConfusionMatrix(labels:max())
  for i = 1,results:size(1) do
     confusion:add(results[i], labels[i])
  end
--  print(confusion)
--  print(results:exp())
--  print(results:size())
--  for i = 1, results:size(1) do
--    local max, indice = torch.max(results[i], 1)
--    print("max, indice, label", max:max(), indice:max(), labels[i])
--  end
end -- evaluate_resuts()


local function test_net(cnn_output, labels)
  local results = model:forward(cnn_output)
  evaluate_resuts(results, labels)  
  return results:exp()
end -- test_net()


local function main()
  if opt.prepair_samples then  
    print("Preparing Samples...\n")
    
    cnn_model = torch.load('/home/alberto/neuraltalk2/cnn.model')
    cnn_model = cnn_model:cuda()
    -- print(cnn_model)
    cnn_model:evaluate()
    -- local images = torch.load('/home/alberto/neuraltalk2/image.data')
    
    print("- Training Samples\n")
    -- prepair_samples_using_cnn("/dados/GPS_clean/UFES-2012-30-train.csv", "/dados/GPS_clean/2012/", "training") -- depois que preparar, nao precisa rodar de novo
    prepair_samples_using_cnn_new("training-20140418.csv.images.tensor", "training-20140418.csv.labels.tensor") -- depois que preparar, nao precisa rodar de novo
    print("- Test Samples\n")
    -- prepair_samples_using_cnn("/dados/GPS_clean/UFES-2014-30-train.csv", "/dados/GPS_clean/2014/", "test") -- depois que preparar, nao precisa rodar de novo
    prepair_samples_using_cnn_new("test-20160906.csv.images.tensor", "test-20160906.csv.labels.tensor") -- depois que preparar, nao precisa rodar de novo
  end
  
  local images = torch.load("training-20140418.csv.images.tensor_images.tensor")
  local cnn_output = torch.load("training-20140418.csv.images.tensor_cnn_out.tensor")
  local labels = torch.load("training-20140418.csv.labels.tensor_labels.tensor")
  
  print("++++ Build net")
  build_net(cnn_output, labels)
  -- print(model)

  -- treinar a rede com cada cnn_output (input) com o label correspondente (target)
  print("++++ Train net")
  train_net(cnn_output:float(), labels)
  print()
  
  -- testar com as images de 2012 (teste de sanidade) e depois com images de 2014 (tem que ler e gerar tensor em forma de arquivo)
  local test_images, test_cnn_output, test_labels
  if opt.sanity then
    print("==== Test net: sanity test! ====")
    test_images = torch.load("training-20140418.csv.images.tensor_images.tensor")
    test_cnn_output = torch.load("training-20140418.csv.images.tensor_cnn_out.tensor")
    test_labels = torch.load("training-20140418.csv.labels.tensor_labels.tensor")
  else
    print("==== Test net: real test! ====")
    test_images = torch.load("test-20160906.csv.images.tensor_images.tensor")
    test_cnn_output = torch.load("test-20160906.csv.images.tensor_cnn_out.tensor")
    test_labels = torch.load("test-20160906.csv.labels.tensor_labels.tensor")
  end
  local results = test_net(test_cnn_output:float(), test_labels) -- os mesmos <images, labels> (sanidade) ou os de 2014
  
  local training_images = torch.load("training-20140418.csv.images.tensor_images.tensor")
  local training_labels = torch.load("training-20140418.csv.labels.tensor_labels.tensor")
  local correct = 0
  local win1, win2
  for i = 1, test_images:size(1) do
    win1 = image.display{image = test_images[i], zoom = 1, win=win1, legend = 'test image ' .. tostring(i)}
    local max_val, index = torch.max(results, 2)
    win2 = image.display{image = training_images[index[i]:max()], zoom = 1, win=win2, legend = 'training image ' .. training_labels[index[i]:max()]}
    if math.abs(test_labels[i] - training_labels[index[i]:max()]) <= opt.allowed_distance then
      correct = correct + 1
    end
    print("test_label, training_label ", test_labels[i], training_labels[index[i]:max()])
    local line = io.read() 
  end
  print("correct = " .. 100 * correct / test_images:size(1) .. "%")
  
end -- main()

main()
-- qlua main.lua -e 5000 -m 0.0