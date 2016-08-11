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

* Cmake
```
$ cd $CARMEN_HOME/src/zed_camera
$ mkdir build
$ cd build
$ cmake ..
$ make
```


### Development

Messages
* carmen_zed_camera_sensor_stereoimage_message:
```
typedef struct {
    int width;     
    int height;    
    int image_size;
    int isRectified = 1;
    unsigned char *raw_left;
    unsigned char *raw_right;
    double timestamp;
    char *host;
} carmen_zed_camera_sensor_stereoimage_message;
```
* carmen_zed_camera_sensor_depthmap_message:
```
typedef struct {
    int width;
    int height;
    int image_size;
    unsigned char *raw_image;
    double timestamp;
    char *host;
} carmen_zed_camera_sensor_depthmap_message;
```
* Params in the param daemon _ini_ file:
```
####################################
# ZED Camera Sensor

zed_camera_sensor_quality		2 	    #HD2K: 0, HD1080: 1, HD720: 2, VGA: 3
zed_camera_sensor_fps			0.0	    #0.0 is the default value
```

Note:
The images have 4 channels as the sensor produces.

### Todos
 - Develop disparity map mode(if needed)
