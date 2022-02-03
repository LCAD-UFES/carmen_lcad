# DeepMapper

DeepMapper is a Carmen subsystem to estimate depth maps from monocular camera images using Neural Networks. 

## Pretrained model

The pretrained models "AdaBins_nyu.pt" and "AdaBins_kitti.pt" are available at [here](https://1drv.ms/u/s!AuWRnPR26byUmfRxBQ327hc8eXse2Q?e=AQuYZw).
* Download these weights and save them at "$CARMEN_HOME/src/deep_mapper/Adabins/pretrained/", it is needed for running the Neural Network.

## Preparing the environment

```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```
If your system already has CUDA 10.0 e CUDNN compatibles installed, run the command:
```shell
    ./install_dependencies.sh
```

* For using this subsytem is required: CUDA_10.0 (and CUDNN compatible), Python 3.6, pip and virtualenv.

### In case your machine doesn't have CUDA and CUDNN
* Run the command and all dependencies are gonna be installed:
```shell
    ./install_dependencies.sh 1
```
The parameter 1 is required to install CUDA and CUDNN.


## For using the module in Carmen_LCAD

 Once the weights are saved at "pretrained" folder and the project CARMEN_LCAD compiled and all dependencies installed, <br/>
 run the commands bellow to use. 
 
### Configuring a process.ini (optional)
It is necessary ajust some process.ini to load inside a proccontrol process. Edit the choosen process and add these lines:
```
deep_map_ART    depth_map   0   0   export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_adabins 1  # using with ART and Intelbras
deep_map_IARA   depth_map   0   0   export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_adabins 3  # using with IARA and Bumblebee 3
GLP_IARA   depth_map   0   0   export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_glpdepth 3  # using with IARA and Bumblebee 3
```

### Execution
Execute these steps:
```shell
cd $CARMEN_HOME/bin/
./central
```
```shell
cd $CARMEN_HOME/bin/
./proccontrol process-playback-fovea.ini
```
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_adabins 1
```
After loading the pretrained weights, just play and run. The results are showed.

In order to use GLPDepth, run:
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/GLPDepth::$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/GLPDepth/venv/bin/activate; ./stereo_velodyne_glpdepth 1
```

In order to use DPT-Intel, run:
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DPT::$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/DPT/venv/bin/activate; ./stereo_velodyne_dpt 3
```


# Original Article
[AdaBins](https://arxiv.org/abs/2011.14141)
Global-Local Path Networks for Monocular Depth Estimation with Vertical CutDepth [GLPDepth](https://arxiv.org/abs/2201.07436)
