# DeepMapper

DeepMapper is a Carmen subsystem to estimate depth maps from monocular camera images using Neural Networks. 

Requirements: Python3.7, pip, virtualenv, Cuda 10.0 and CUDNN compatible.

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
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```
If your system already has CUDA 10.0 e CUDNN compatibles installed, run the command:
```shell
    ./install_dependencies.sh
```

* For using this subsytem is required: CUDA_10.0 Python 3.6, pip and virtualenv.

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
cam1_adabins		detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network adabins
cam1_glpdepth	    detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/GLPDepth:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/GLPDepth/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network glpdepth
 cam1_dpt           detection   0       0       export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DPT:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/DPT/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network dpt
 
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
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network adabins
```
After loading the pretrained weights, just play and run. The results are showed.

In order to use GLPDepth, run:
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/GLPDepth:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/GLPDepth/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network glpdepth
```

In order to use DPT-Intel, run:
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DPT:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/DPT/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network dpt
```


# Original Article
[AdaBins](https://arxiv.org/abs/2011.14141)
Global-Local Path Networks for Monocular Depth Estimation with Vertical CutDepth [GLPDepth](https://arxiv.org/abs/2201.07436)
