# ZED Camera Sensor with OpenCV

### Dependencies
Ubuntu 16.04 LTS
* [OpenCV 3.1](https://github.com/opencv/opencv)

### Build

    ```
    $ cd $CARMEN_HOME/src/zed_opencv
    $ make
    ```

### Run

    ```
    ./zed_opencv <device_id> <calibration_file>
    ```

The device id is the current /dev/video<ID> and the zed calibration file is available in the settings/ folder inside the project.

This drive publish bumblebee messages with camera id equals 4

### Development

Messages
* carmen_bumblebee_basic_stereoimage_message

Params in the carmen-ford-escape.ini:

  ###############################

  '# Bumblebee parameters

  '#--------- Bumblebee Basic 4 - ZED ------------

  bumblebee_basic4_width    1920

  bumblebee_basic4_height   1080

  bumblebee_basic4_zed_fps   0.0     # 0.0 is the default value

  bumblebee_basic4_zed_serial_number  19436 # printed in the box

```
Note:
width, height and FPS possible configurations:
RESOLUTION_HD2K 2208*1242, available framerates: 15 fps.
RESOLUTION_HD1080 1920*1080, available framerates: 15, 30 fps.
RESOLUTION_HD720 1280*720, available framerates: 15, 30, 60 fps.
RESOLUTION_VGA 672*376, available framerates: 15, 30, 60, 100 fps.

All other parameters follow the bumblebee params pattern
``