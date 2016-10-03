require 'torch'
require 'image'

local function build_csv_file(csv_name, out_name, offset)
  local csv = require("csv")
  local f = csv.open(csv_name)
  local o = io.open(out_name, "w+")
  local j = -1
  local x, y, theta, v, timestamp, left_img, right_img, ref_x, ref_y
  local index = 1
  --o:write("index, dist, x, y, theta, v, timestamp, left_img, right_img\n")
  o:write("timestamp, x, y, label\n")
  for fields in f:lines() do
    if j ~= -1 then
      for i, value in ipairs(fields) do
        if i == 1 then x = tonumber(value) end 
        if i == 2 then y = tonumber(value) end 
        if i == 3 then theta = tonumber(value) end 
        if i == 4 then v = tonumber(value) end 
        if i == 5 then timestamp = value end 
        if i == 6 then left_img = value end 
        if i == 7 then right_img = value end 
      end
      if j == 0 then
        ref_x = x
        ref_y = y
      end
      local dist = math.sqrt((x - ref_x) * (x - ref_x) + (y - ref_y) * (y - ref_y))
      if (dist >= offset) then
        --o:write(index .. ", " .. dist .. ", " .. x .. ", " .. y .. ", " .. theta .. ", " .. v .. ", " .. timestamp .. ", " .. left_img .. ", " .. right_img .. "\n")
        o:write(timestamp .. ", " .. x .. ", " .. y .. ", " .. index .. "\n")
        ref_x = x
        ref_y = y
        index = index + 1
      end
    end
    j = j + 1
  end
  o:close()
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
        if i == 5 then timestamp = value end 
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


local function make_csv_pair(csv_name1, csv_name2, out_name)
  local csv = require("csv")
  local f = csv.open(csv_name1)
  local o = io.open(out_name, "w+")
  local j = -1
  local index, dist, x, y, theta, v, timestamp, left_img, right_img, ref_x, ref_y
  --o:write("index, dist, x, y, theta, v, timestamp, left_img, right_img\n")
  o:write("timestamp, x, y, label\n")
  for fields in f:lines() do
    if j ~= -1 then
      for i, value in ipairs(fields) do
        if i == 1 then timestamp = value end 
        if i == 2 then x = tonumber(value) end 
        if i == 3 then y = tonumber(value) end 
        if i == 4 then index = tonumber(value) end 
      end
      dist, x, y, theta, v, timestamp, left_img, right_img = find_nearest(csv_name2, x, y)
      --o:write(index .. ", " .. dist .. ", " .. x .. ", " .. y .. ", " .. theta .. ", " .. v .. ", " .. timestamp .. ", " .. left_img .. ", " .. right_img .. "\n")
      o:write(timestamp .. ", " .. x .. ", " .. y .. ", " .. index .. "\n")
    end
    j = j + 1
  end
  o:close()
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


local function count_images(csv_name1)
  local csv = require("csv")
  local f1 = csv.open(csv_name1)
  local j = -1
  for fields1 in f1:lines() do
    j = j + 1
  end
  return j;
end


local function save_images(csv_name1, csv_name2, images_dir1, images_dir2, max_images)
  local csv = require("csv")
  local f1 = csv.open(csv_name1)
  local num_images = math.min(count_images(csv_name1), max_images)
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
        if i1 == 4 then index1 = tonumber(value) end 
        if i1 == 1 then left_img1 = value end 
      end
      local f2 = csv.open(csv_name2)
      for fields2 in f2:lines() do
        for i2, value in ipairs(fields2) do
          if i2 == 4 then index2 = tonumber(value) end 
          if i2 == 1 then left_img2t = value end 
        end
        if (index1 == index2) then
          left_img2 = left_img2t
        end
      end
      f2:close()
      if (i <= tonumber(num_images)) then
        local img1 = image.load(images_dir1 .. left_img1 .. '.bb08.l.png', 3, 'byte')
        img1 = image.scale(img1, 640, 480, 'bilinear')
        img1 = image.crop(img1, 0, 0, 640, 360)
        img1 = image.scale(img1, 224, 224, 'bilinear')
        imgs1[i] = img1
        labels1[i] = index1
        local img2 = image.load(images_dir2 .. left_img2 .. '.bb08.l.png', 3, 'byte')
        img2 = image.scale(img2, 640, 480, 'bilinear')
        img2 = image.crop(img2, 0, 0, 640, 360)
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


local function main(logfilename_fortraining, csvfilename_fortraining, 
  logfilename_fortest, csvfilename_fortest, imagedir_fortraining, imagedir_fortest, 
  offset, max_images)
  build_csv_file(logfilename_fortraining, csvfilename_fortraining, offset)
  make_csv_pair(csvfilename_fortraining, logfilename_fortest, csvfilename_fortest)
  --view_image_pair(csvfilename_fortraining, csvfilename_fortest, imagedir_fortraining, imagedir_fortest)
  save_images(csvfilename_fortraining, csvfilename_fortest, imagedir_fortraining, imagedir_fortest, max_images)
end -- main()

offsets = {30.0}--, 15.0, 10.0, 5.0, 1.0}
for i, offset in pairs(offsets) do
  print('offset:' .. offset)
  main('globalpos-20140418.txt', 'UFES-20140418-' .. offset .. '-train.csv', 
  'globalpos-20160906.txt', 'UFES-20160906-' .. offset .. '-test.csv',
  '/dados/UFES/GPS_clean/20140418/', '/dados/UFES/GPS_clean/20160906/',
  offset, 10000)
end
