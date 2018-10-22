# Training neural_mapper

Currently the network can be trained on two datasets:

| Datasets | Input Resolution | Output Resolution | # of classes |
|:--------:|:----------------:|:------------------:|:------------:|
| [kitti-road-dataset]() | 400x200 | 400x200 | 2 |
| [IARA-mapper-dataset]() | 1050x1050* | 3 | *Input resolution can be changed

## Example command for training:

```
CUDA_VISIBLE_DEVICES=0 qlua run.lua --dataset kitti --datapath /dados/kitti-road-dataset/ --model models/road_model.lua --save save/trained/model/ --imHeight 400 --imWidth 200 --labelHeight 400 --labelWidth 200 --cachepath KITTI_DATASET_REDUCED --batchSize 1 --channels 5 --learningRate 0.0006 --lrDecayEvery 0 --maxepoch 100
```

# Lua language editor

You can use the the ZeroBrane Studio: https://studio.zerobrane.com/download

For Gerenate the dataset see the generate_gt folder
