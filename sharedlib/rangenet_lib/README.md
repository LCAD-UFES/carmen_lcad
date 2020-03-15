# LibRangeNet

## Semantic segmentation inference lib using RangeNet

#### Dependencies

##### System dependencies
First you need to install the nvidia driver and CUDA. 

- For Carmen you need to install CUDA with .deb files.

- Install open_cv WITH VTK = ON

- Then you can do the other dependencies:

  ```sh
  $ sudo apt-get update 
  $ sudo apt-get install -yqq  build-essential python3-dev python3-pip apt-utils git cmake libboost-all-dev libyaml-cpp-dev libopencv-dev
  ```
  
##### Python dependencies

- Then install the Python packages needed:

  ```sh
  $ sudo apt install python-empy
  $ sudo pip install trollius numpy
  ```
  
##### TensorRT

In order to infer with TensorRT during inference with the C++ libraries:

- Install TensorRT: [Link](https://developer.nvidia.com/tensorrt).
- RangeNet++ code only works with **TensorRT version 5** (Note that you need at least version 5.1.0).

#### Build the library
We changed the library for using only cmake and makefile.

```sh
  :~/carmen_lcad/sharedlib/rangenet_lib$ cmake .
  :~/carmen_lcad/sharedlib/rangenet_lib/src$ make
```
#### Generate BIN files for testing

Set variable mapper_use_remission = on in your carmen.ini file.

Run Central
```
:~/carmen_lcad/bin$ ./central
```

Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

Run save_velodyne_bin:
```
:~/carmen_lcad/sharedlib/rangenet_lib/src$ ./save_velodyne_bin
```


#### Run the demo

To run the demo, you need a pre-trained model, which can be downloaded in [model](http://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz). Unzip it in rangenet_lib folder:

```sh
  # go to the root path of the rangenet_lib folder
  $ cd ~/carmen_lcad/sharedlib/rangenet_lib/
  :~/carmen_lcad/sharedlib/rangenet_lib$ mkdir model && cd model
  :~/carmen_lcad/sharedlib/rangenet_lib/model$ mv ~/Downloads/darknet53.tar.gz .
  :~/carmen_lcad/sharedlib/rangenet_lib/model$ tar -zxvf darknet53.tar.gz
```

The LiDAR scan was generated in example folder.

To infer a single LiDAR scan and visualize the semantic point cloud:

  ```sh
  $ cd ~/carmen_lcad/sharedlib/rangenet_lib/
  # use --verbose or -v to get verbose mode
  $ ./infer -h # help
  $ ./infer -p /path/to/the/pretrained/model -s /path/to/the/scan.bin --verbose

  :~/carmen_lcad/sharedlib/rangenet_lib$ ./infer -p model/darknet53/ -s example/1542360697.155346.bin --verbose #example

  ```

**Notice**: for the first time running, it will take several minutes to generate a `.trt` model for C++ interface.


### Testing

Set variable mapper_use_remission = on in your carmen.ini file.

Run Central
```
:~/carmen_lcad/bin$ ./central
```

Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

Run velodyne_to_rangenet:
```
:~/carmen_lcad/bin$ ./velodyne_to_rangenet
```

Push Playback button. The results from segmentation will be at screen.
