<div align="center">
<h1> Lane Position Localizer: Precise Localization on the Runway Using Lane Detection 🚗 </h1>

</div>

# Introduction

Autonomous driving is a groundbreaking technology that is reshaping the transportation industry, offering the potential to greatly enhance road safety and increase the efficiency of mobility. Autonomous vehicles offer a multitude of advantages, including accident prevention, optimized fuel consumption, and improved passenger comfort. In this swiftly evolving field, probabilistic mathematical models play a pivotal role in addressing the challenges posed by uncertainty, particularly in achieving precise vehicle localization and accurate detection of traffic lanes.

## Localization Approaches
In the realm of autonomous driving, several approaches have been employed to address the crucial task of precise localization. Methods such as the Extended Kalman Filter, Particle Filter, and Probabilistic Occupancy Map are widely utilized to estimate the vehicle's position relative to its surrounding environment.

## Proposed Model
Our proposed model leverages the YOLOPv2 neural network for real-time detection of traffic lanes and vehicles. This system integrates color information for lane detection and utilizes a pretrained neural network for vehicle identification. The data captured by this model is subsequently employed for the navigation of the autonomous vehicle, while accounting for the inherent uncertainty present in the environment.

In this approach, we replace the conventional use of LiDAR with GPS, employing a measurement model that considers both camera and GPS data. This ensures an accurate estimation of the vehicle's pose in Frenét coordinates.

<div align="center">
  <a href="https://ibb.co/1bjc38J"><img src="https://i.ibb.co/jHstdgZ/Screenshot-from-2023-12-16-21-54-57.png" alt="Screenshot-from-2023-12-16-21-54-57" border="0"></a>
</div>


### Visualization
<div align="center">
  <td><img src=https://i.ibb.co/44x03D9/Design-sem-nome.png></td>
  <p> Fig. 1 - Detection of representative lanes and navigable areas - UFES Loop </p>
</div>



# Installation
## 1. Prerequisites

- Carmen_lcad installed
- Graphics card (GPU)
- OpenCV 4.x
- CUDA + cuDNN
- TensorRT 8.x (<a href="https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html">link</a>) 

## 2. Environment Setup

### 1. Install Carmen Lcad
- Install **CARMEN LCAD** using the <a href="https://github.com/LCAD-UFES/carmen_lcad">link</a>.

### 2. Install TensorRT:
- Siga as instruções desse (<a href="https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html">link</a>)

### 3. [YolopV2]Download source code and pre-built libraries:
- Download (<a href="https://drive.google.com/drive/folders/1yUQw13WZac7YGkQruhuEm902465WwSE4?usp=sharing">link</a>)
  
#### 3.1 Extraia os arquivos dentro da pasta lpl:
 ``` shell
# Folder
cd /Downloads
unzip play_with_tensorrt.zip
cp -r /downloads/play_with_tensorrt $CARMEN_HOME/src/lpl/
```
#### 3.2 Build
 ``` shell
# Folder
cd $CARMEN_HOME/src/lpl/play_with_tensorrt/pj_tensorrt_perception_yolopv2
cd build
cmake ..
make
```


### 5. Compile the "lpl" file:
- Open the folder lpl
``` shell
# Open the folder
cd carmen_lcad/src/lpl/
```
- Compile
``` shell
# Open the folder
make clean & make
```

### 6. Install Log Volta da UFES:
- Download the files from this <a href="https://drive.google.com/drive/u/1/folders/1b1_sD6NnUX-oZER_0LGa0jo9QpZEcsq7">link</a>
- Unzip the files
``` shell
# Unzip Logs
unzip logs.zip
```
``` shell
# Unzip Logs
unzip atego2430_vale.zip
```
-  Copy the files to the following folders
``` shell
# logs
cp -r /Downloads/logs/log_volta_da_ufes-art-20221108-2.txt* /dados/
```

``` shell
# Atego2430_vale
cp -r /Downloads/atego2430_vale/ $CARMEN_HOME/bin/
```

## 4. Execution
- Open the Terminal **Ctrl+Alt+T**:
- Open the folder carmen_lcad/bin
 ```shell
# Folder Bin
cd
cd carmen_lcad/bin
```
- Open ./central
```shell
# central:
./central
```

- Open the Logs
```shell
# Logs:
./proccontrol atego2430_vale/process-atego2430_vale_playback_viewer_3D.ini 
```

- Open the module
```shell
# Lane positions localizer:
./neural_object_detector_tracker intelbras1 1
```
<div align="center">
  <a href="https://ibb.co/thK4GSN"><img src="https://i.ibb.co/PYjNLRb/playback.png" alt="playback" border="0"></a>
  <p>Module - Carmen LCAD</p>
</div>


