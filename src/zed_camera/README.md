# ZED Camera Sensor

### Dependencies
Ubuntu 14.04 LTS
* [OpenCV 3.1](https://github.com/opencv/opencv)
* [CUDA 7.5](https://developer.nvidia.com/cuda-downloads)
* [ZED SDK 1.0](https://www.stereolabs.com/download_327af3/ZED_SDK_Linux_x86_64_v1.0.0c.run)

### Build
* Makefile
```
$ cd $CARMEN_HOME/src/zed_camera
$ make
```
### Run

```
./zed_camera_sensor <stereo|depth>
```
This drive publish bumblebee messages with camera id 4

### Development

Messages
* carmen_bumblebee_basic_stereoimage_message - if stereo mode 
* carmen_zed_camera_sensor_depthmap_message - if depth mode

Params in the carmen-ford-escape.ini:

  ###############################

  '# Bumblebee parameters

  '#--------- Bumblebee Basic 4 - ZED ------------

  bumblebee_basic4_width    1920

  bumblebee_basic4_height   1080

  bumblebee_basic4_zed_fps   0.0     #0.0 is the default value
```
Note:
width, height and FPS possible configurations:
RESOLUTION_HD2K 2208*1242, available framerates: 15 fps.
RESOLUTION_HD1080 1920*1080, available framerates: 15, 30 fps.
RESOLUTION_HD720 1280*720, available framerates: 15, 30, 60 fps.
RESOLUTION_VGA 672*376, available framerates: 15, 30, 60, 100 fps.

All other parameters follow the bumblebee params pattern
```
### Todos
 - Solve latency problem
