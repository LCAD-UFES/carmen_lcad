# DeepMapper

DeepMapper is a Carmen subsystem to estimate depth maps from monocular camera images using Neural Networks.

Requirements: Python3.8, pip, virtualenv, Cuda 11.0 and CUDNN compatible.

For install libraries, run:
```shell
cd $CARMEN_HOME/src/deep_mapper/DPT/ && make clean && make && cd ../GLPDepth && make clean && make && cd ../Adabins && make clean && make && cd .. && make clean && make
```
pip install torch==1.10.1+cu111 torchvision==0.11.2+cu111 torchaudio==0.10.1 -f https://download.pytorch.org/whl/torch_stable.html

## Preparing the environment for GLPDepth

Download the pretrained weights:

```shell
    cd $CARMEN_HOME/src/deep_mapper/GLPDepth
    make download
```
Create VirtualEnv in order to install torch and its dependencies:
```shell
    cd $CARMEN_HOME/src/deep_mapper/GLPDepth
```
```shell
    ./create_env.sh
```

Compile GLPDepth libraries:
```shell
    cd $CARMEN_HOME/src/deep_mapper/GLPDepth
```
```shell
    make
```

Compile DeepMapper:
```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```

## Preparing the environment for DPT-Intel

Download the pretrained weights:

```shell
    cd $CARMEN_HOME/src/deep_mapper/DPT
    make download
```

Create VirtualEnv in order to install torch and its dependencies:
```shell
    cd $CARMEN_HOME/src/deep_mapper/DPT
```
```shell
    ./create_env.sh
```
Compile DPT libraries:
```shell
    make
```
Compile DeepMapper:
```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```

## Preparing the environment for AdaBins

The pretrained model "AdaBins_kitti.pt" is available at [here](https://1drv.ms/u/s!AuWRnPR26byUmfRxBQ327hc8eXse2Q?e=AQuYZw).
* Download these weights and save them at "$CARMEN_HOME/src/deep_mapper/Adabins/pretrained/", it is needed for running the Neural Network.


```shell
    cd $CARMEN_HOME/src/deep_mapper/Adabins
```
```shell
    make download
```

If your system already has CUDA 11.0 e CUDNN compatibles installed, run the command:
```shell
    cd $CARMEN_HOME/src/deep_mapper/Adabins
```
```shell
    ./install_dependencies.sh
```
```shell
    cd $CARMEN_HOME/src/deep_mapper/Adabins
```
```shell
    make 
```
* For using this subsytem is required: CUDA_11.0, Python 3.8, pip and virtualenv.

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
cam1_adabins		detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/Adabins/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network adabins
cam1_glpdepth	    detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/GLPDepth:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/GLPDepth/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network glpdepth
 cam1_dpt           detection   0       0       export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DPT:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/DPT/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network dpt
```

### Execution
Execute these steps:
```shell
cd $CARMEN_HOME/bin/
./central
```
Making maps
```shell
cd $CARMEN_HOME/bin/
./proccontrol process-volta_da_ufes_playback_viewer_3D_map_generation_sensorbox_deepmapper_1.ini
```
Localize
```shell
./proccontrol process-volta_da_ufes_playback_viewer_3D_sensorbox_deepmapper_1.ini
```

### Steps to a Grip-Map from GLPDepth

##### Map Generation
Verify if velodyne are "off" and lidar8 are "on" at /src/carmen-ford-escape-sensorbox-deepmapper.ini
```shell
./proccontrol process-volta_da_ufes_playback_viewer_3D_map_generation_sensorbox_deepmapper_1.ini
```
Clean maps and turn on PubPoses
##### Localize
```shell
./proccontrol process-volta_da_ufes_playback_viewer_3D_sensorbox_deepmapper_1.ini
```

# Original Article
[AdaBins](https://arxiv.org/abs/2011.14141)
Global-Local Path Networks for Monocular Depth Estimation with Vertical CutDepth [GLPDepth](https://arxiv.org/abs/2201.07436)
