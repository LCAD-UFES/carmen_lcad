require 'torch'
require 'image'

local function build_csv_file(csv_name)
  local csv = require("csv")
  local f = csv.open(csv_name)
  local j = -1
  local x, y, theta, v, timestamp, left_img, right_img, ref_x, ref_y
  local index = 1
  print("index, dist, x, y, theta, v, timestamp, left_img, right_img")
  for fields in f:lines() do
    if j ~= -1 then
      for i, value in ipairs(fields) do
        if i == 1 then x = tonumber(value) end 
        if i == 2 then y = tonumber(value) end 
        if i == 3 then theta = tonumber(value) end 
        if i == 4 then v = tonumber(value) end 
        if i == 5 then timestamp = tonumber(value) end 
        if i == 6 then left_img = value end 
        if i == 7 then right_img = value end 
      end
      if j == 0 then
        ref_x = x
        ref_y = y
      end
      local dist = math.sqrt((x - ref_x) * (x - ref_x) + (y - ref_y) * (y - ref_y))
      if (dist >= 30.0) then
        print(index .. ", " .. dist .. ", " .. x .. ", " .. y .. ", " .. theta .. ", " .. v .. ", " .. timestamp .. ", " .. left_img .. ", " .. right_img)
        ref_x = x
        ref_y = y
        index = index + 1
      end
    end
    j = j + 1
  end
end -- load_images_and_labels()


local function find_nearest(csv_name2, ref_x, ref_y)
  local csv = require("csv")
  local f = csv.open(csv_name2)
  local j = -1
  local index, dist, x, y, theta, v, timestamp, left_img, right_img
  local index, dist, nx, ny, ntheta, nv, ntimestamp, nleft_img, nright_img
  local best_distance = 10000000
  for fields in f:lines() do
    if j ~= -1 then
      for i, value in ipairs(fields) do
        if i == 1 then x = tonumber(value) end 
        if i == 2 then y = tonumber(value) end 
        if i == 3 then theta = tonumber(value) end 
        if i == 4 then v = tonumber(value) end 
        if i == 5 then timestamp = tonumber(value) end 
        if i == 6 then left_img = value end 
        if i == 7 then right_img = value end 
      end
      local dist = math.sqrt((x - ref_x) * (x - ref_x) + (y - ref_y) * (y - ref_y))
      if (dist < best_distance) then
        nx = x
        ny = y
        ntheta = theta
        nv = v
        ntimestamp = timestamp
        nleft_img = left_img
        nright_img = right_img
        best_distance = dist
      end
    end
    j = j + 1
  end
  --print(best_distance, nx, ref_x, ny, ref_y, ntheta, nv, ntimestamp, nleft_img, nright_img)  
  return best_distance, nx, ny, ntheta, nv, ntimestamp, nleft_img, nright_img
end -- find_nearest()


local function make_csv_pair(csv_name1, csv_name2)
  local csv = require("csv")
  local f = csv.open(csv_name1)
  local j = -1
  local index, dist, x, y, theta, v, timestamp, left_img, right_img, ref_x, ref_y
  print("index, dist, x, y, theta, v, timestamp, left_img, right_img")
  for fields in f:lines() do
    if j ~= -1 then
      for i, value in ipairs(fields) do
        if i == 1 then index = tonumber(value) end 
        if i == 3 then x = tonumber(value) end 
        if i == 4 then y = tonumber(value) end 
      end
      dist, x, y, theta, v, timestamp, left_img, right_img = find_nearest(csv_name2, x, y)
      print(index .. ", " .. dist .. ", " .. x .. ", " .. y .. ", " .. theta .. ", " .. v .. ", " .. timestamp .. ", " .. left_img .. ", " .. right_img)
    end
    j = j + 1
  end
end -- make_csv_pair()


local function view_image_pair(csv_name1, csv_name2, images_dir1, images_dir2)
  local csv = require("csv")
  local f1 = csv.open(csv_name1)
  local j = -1
  local index1, index2, left_img1, left_img2, left_img2t
  for fields1 in f1:lines() do
    if j ~= -1 then
      for i1, value in ipairs(fields1) do
        if i1 == 1 then index1 = tonumber(value) end 
        if i1 == 8 then left_img1 = value end 
      end
      local f2 = csv.open(csv_name2)
      for fields2 in f2:lines() do
        for i2, value in ipairs(fields2) do
          if i2 == 1 then index2 = tonumber(value) end 
          if i2 == 8 then left_img2t = value end 
        end
        if (index1 == index2) then
          left_img2 = left_img2t
        end
      end
      f2:close()
      local img1 = image.load(images_dir1 .. left_img1, 3, 'byte')
      img1 = image.scale(img1, 640, 480, 'bilinear')
      img1 = image.crop(img1, 0, 80, 640, 360)
      img1 = image.scale(img1, 224, 224, 'bilinear')
      image.display{image = img1, zoom = 1, legend = 'image1'}

      local img2 = image.load(images_dir2 .. left_img2, 3, 'byte')
      img2 = image.scale(img2, 640, 480, 'bilinear')
      img2 = image.crop(img2, 0, 80, 640, 360)
      img2 = image.scale(img2, 224, 224, 'bilinear')
      image.display{image = img2, zoom = 1, legend = 'image2'}
      local char = io.read()
    end
    j = j + 1
  end
end -- make_csv_pair()


local function save_images(csv_name1, csv_name2, images_dir1, images_dir2, num_images)
  local csv = require("csv")
  local f1 = csv.open(csv_name1)
  local j = -1
  local i = 1
  local imgs1 = torch.Tensor(tonumber(num_images), 3, 224, 224)
  local imgs2 = torch.Tensor(tonumber(num_images), 3, 224, 224)
  local labels1 = torch.Tensor(tonumber(num_images))
  local labels2 = torch.Tensor(tonumber(num_images))
  local index1, index2, left_img1, left_img2, left_img2t
  for fields1 in f1:lines() do
    if j ~= -1 then
      for i1, value in ipairs(fields1) do
        if i1 == 1 then index1 = tonumber(value) end 
        if i1 == 8 then left_img1 = value end 
      end
      local f2 = csv.open(csv_name2)
      for fields2 in f2:lines() do
        for i2, value in ipairs(fields2) do
          if i2 == 1 then index2 = tonumber(value) end 
          if i2 == 8 then left_img2t = value end 
        end
        if (index1 == index2) then
          left_img2 = left_img2t
        end
      end
      f2:close()
      if (i <= tonumber(num_images)) then
        local img1 = image.load(images_dir1 .. left_img1, 3, 'byte')
        img1 = image.scale(img1, 640, 480, 'bilinear')
        img1 = image.crop(img1, 0, 80, 640, 360)
        img1 = image.scale(img1, 224, 224, 'bilinear')
        imgs1[i] = img1
        labels1[i] = index1
  
        local img2 = image.load(images_dir2 .. left_img2, 3, 'byte')
        img2 = image.scale(img2, 640, 480, 'bilinear')
        img2 = image.crop(img2, 0, 80, 640, 360)
        img2 = image.scale(img2, 224, 224, 'bilinear')
        imgs2[i] = img2
        labels2[i] = index1
      end
      print(index1, i)
      i = i + 1
    end
    j = j + 1
  end
  torch.save(csv_name1 .. ".images.tensor", imgs1)
  torch.save(csv_name2 .. ".images.tensor", imgs2)
  torch.save(csv_name1 .. ".labels.tensor", labels1)
  torch.save(csv_name2 .. ".labels.tensor", labels2)
end -- save_images()


local function main(name1, name2, name3, name4, name5)
  --build_csv_file(name1)
  -- qlua make_dataset_from_raw_data.lua globalpos.txt > training-20140418-30m.csv
  --make_csv_pair(name1, name2)
  -- qlua make_dataset_from_raw_data.lua training-20140418.csv globalpos.txt > test-20160906.csv
  --view_image_pair(name1, name2, name3, name4)
  -- qlua make_dataset_from_raw_data.lua training-20140418.csv test-20160906.csv /media/alberto/Seagate\ Backup\ Plus\ Drive/images-20140418/ /media/alberto/Seagate\ Backup\ Plus\ Drive/images-20160906/  
  --save_images(name1, name2, name3, name4, name5)
  -- qlua make_dataset_from_raw_data.lua training-20140418.csv test-20160906.csv /media/alberto/Seagate\ Backup\ Plus\ Drive/images-20140418/ /media/alberto/Seagate\ Backup\ Plus\ Drive/images-20160906/ 100
  -- Depois de rodar tudo, rode o main.lua como abaixo
  --qlua main.lua -e 4000 -m 0.0 -p -n 100 -- gera a base de dados e roda a rede
  -- So rodar teste de sanidade
  --qlua main.lua -e 4000 -m 0.0 -s
  -- So rodar teste de verdade
  --qlua main.lua -e 4000 -m 0.0
end -- main()

main(arg[1], arg[2], arg[3], arg[4], arg[5])
