# DeepMapper

DeepMapper is a Carmen subsystem to estimate depth maps from monocular camera images using Neural Networks. 

Requirements: Python3.7, pip, virtualenv, Cuda 10.0 and CUDNN compatible.

## Preparing the environment for GLPDepth

Download the pretrained weights:

```shell
    cd $CARMEN_HOME/src/deep_mapper/GLPDepth
    make download
```

Install Python3.7.7 (Will install an alternative installation - not bad for your PC)
```shell
    cd $CARMEN_HOME/src/deep_mapper/GLPDepth
```
```shell
    ./install_python3.7.sh
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

If your system already has CUDA 10.0 e CUDNN compatibles installed, run the command:
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
cam1_adabins		detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/Adabins/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network adabins
cam1_glpdepth	    detection	0		0		export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/GLPDepth:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/GLPDepth/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network glpdepth
 cam1_dpt           detection   0       0       export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DPT:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/DPT/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network dpt
 cam1_newcrfs           detection   0       0       export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/newcrfs:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/newcrfs/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network newcrfs
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
Making maps
```shell
cd $CARMEN_HOME/bin/
./proccontrol process-volta_da_ufes_playback_viewer_3D_map_generation_deep_mapper.ini
```
```shell
cd $CARMEN_HOME/bin/
export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/Adabins:$CARMEN_HOME/src/deep_mapper/Adabins/models/:$PYTHONPATH; source $CARMEN_HOME/src/deep_mapper/Adabins/venv/bin/activate; ./stereo_velodyne_depth 1 -neural_network adabins
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

### Steps to a Grip-Map from GLPDepth

#### Step 1 - Logger with lidar

Ajust your carmen.ini file:
```shell
# logger parameters

#logger_ascii		off
logger_localize		off
logger_gps		on
logger_simulator	off
logger_imu          	on
logger_xsens		on
logger_velodyne		on
logger_velodyne_save_to_file on 
logger_xsens_mtig	off
logger_imu_pi       	on
logger_odometry		on
logger_visual_odometry 	off
logger_laser		on
logger_robot_laser	off
logger_params		on
logger_motioncmds   	off  # includes base_velocity and motion commands given to robot
logger_kinect		off
logger_bumblebee	off
logger_bumblebee_frames_to_save 1 # How many frames you save (1 for all)
logger_bumblebee_save_to_file on 
logger_web_cam		off
logger_camera		on
logger_sonar		off
logger_ford_escape_status on
logger_can_dump		off
```

```shell
cd $CARMEN_HOME/bin/
./logger /<path_to_save_log>/log_<name_of_original_log>-lidar8.txt
```

Example:
```shell
cd $CARMEN_HOME/bin/
rm -rf /dados/log_volta_da_ufes_20220120-lidar8.txt*
:~/carmen_lcad/bin$ ./logger /dados/log_volta_da_ufes_20220120-lidar8.txt
```

#### Step 2 - Generate a map

Refer to Graphslam and use /<path_to_save_log>/log_<name_of_original_log>-lidar8.txt LOG to generate a grid map.


# Original Article
[AdaBins](https://arxiv.org/abs/2011.14141)
Global-Local Path Networks for Monocular Depth Estimation with Vertical CutDepth [GLPDepth](https://arxiv.org/abs/2201.07436)
