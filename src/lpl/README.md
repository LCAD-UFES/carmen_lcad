<div align="center">
<h1> Lane Position Localizer: Precise Localization on the Runway Using Lane Detection 🚗 </h1>

<!-- <--!span><font size="5", > Efficient and Robust 2D-to-BEV Representation Learning via Geometry-guided Kernel Transformer
</font></span> -->

  Alefe Gadioli e Israel Barbosa
<!-- <a href="https://scholar.google.com/citations?user=pCY-bikAAAAJ&hl=zh-CN">Jinwei Yuan</a> -->
<div><a href="https://drive.google.com/file/d/112jPwJ2mCc6egTONY9GHNCOqfAn8-g5D/view?usp=drive_link">Lane Position Localizer Report</a></div> 

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

## Implementation Methodology

The practical implementation of our method has been executed on a physical vehicle equipped with a camera and a GPS system. Although practical trials have not yet been conducted, our approach enables real-time detection using the YOLOPv2 network, enabling a continuous and precise update of the vehicle's position based on sensor data. We have chosen to integrate our method with the CARMEN system, facilitating seamless integration with other modules of the autonomous vehicle, such as trajectory planning and motion control. This integration enables safe operation in complex environments and efficient code reuse in future projects, contributing to the ongoing advancement of autonomous driving and vehicle localization.

## Results
The results of using the detection module have shown promise with the utilization of the YOLOPv2 convolutional neural network, as can be seen in the following figures.

### Visualization
<div align="center">
  <td><img src=https://i.ibb.co/44x03D9/Design-sem-nome.png></td>
  <p> Fig. 1 - Detection of representative lanes and navigable areas - UFES Loop </p>
</div>

<div align="center">
  <td><img src=YOLOPv2/data/video2.gif></td>
  <p> Fig. 2 - Detection of representative lanes and navigable areas - LUME</p>
</div>

However, it is evident that the model faces challenges in detecting areas where lane markings are not clear, as illustrated in Figure 2. Conversely, the detection improves significantly in sharper areas, as demonstrated in Figure 1.



# Installation
## 1. Prerequisites

- Carmen_lcad installed
- Graphics card (GPU)
- Python >=3.6

## 2. Environment Setup

### 1. Install Carmen Lcad
- Install **CARMEN LCAD** using the <a href="https://github.com/LCAD-UFES/carmen_lcad">link</a>.


### 2. Install Python 3.6 or higher:
- **Linux:** Use your distribution's package manager, for example:
``` shell
# Python3 - Linux
sudo apt-get install python3
```
### 3. Install Dependencies:
- Open a terminal and navigate to the cloned project directory.
 ``` shell
# Open the Folder
cd carmen_lcad/src/lpl/YOLOPv2/
```
-  Install all required dependencies
``` shell
# YOLOPv2
pip install -r requirements.txt
```

### 4. Edit the system variables:
- Edit the .bashrc (gedit ~/.bashrc):
``` shell
# Terminal
gedit ~/.bashrc
```
- Add the content below after the #Darknet commands:
``` shell
#Darknet3
export DARKNET3_HOME=$CARMEN_HOME/sharedlib/darknet3
export LD_LIBRARY_PATH=$DARKNET3_HOME/lib:$LD_LIBRARY_PATH
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
cp -r /home/alefe/Downloads/logs/log_volta_da_ufes-art-20221108-2.txt* /dados/
```

``` shell
# Atego2430_vale
cp -r /home/alefe/Downloads/atego2430_vale/ /home/alefe/carmen_lcad/bin/
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



# Future Works
As subsequent steps, we will further our efforts to enhance detection, collect lane width data for integration with GPS to accurately calculate the vehicle's pose through the particle model.
