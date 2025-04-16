# The Camera Module Drivers
This module is intented to combine all camera drivers
It is designed to work with cameras with multiple image sensors that may have different dimensios for each image sensor.

Check the IP camera with `check_camera`! Enter and enjoy!

```
./check_camera rtsp://admin:1q2w3e4r@192.168.1.108:554/cam/realmonitor?channel=1&subtype=1
./check_camera /dev/video0
```


## CAMERA DRIVERS Usage
It must receive the camera name in order to read the correspondig parameter from the .ini file and the message number (from 1 to 20)

```
./camera_drivers intelbras1 1
```

Note that the module does not publish the undistorted message, but the original image, leaving it up to the customer to rectify it online, if necessary.

OPTIONAL PARAMETERS:

- undistort <on/off>
    Force undistortion in the image published by camera_drivers


## CAMERA VIEWER Usage
It must receive the number of the message to subscribe

```
./camera_viewer 1
./camera_viewer intelbras1 1
```

In the second case, is enabled the undistortion process, with the parameters obtained by intelbras1.

OPTIONAL PARAMETERS:

-image <>
    Allows to choose one of the multiple images (if an index to an image that exceeds the maximun number of images in the message is passed, the image 0 is displayed)

-resize <>
    Allows to reduce or increase the image window size

-crop_x, crop_y, crop_w, crop_h <>
    Allows to crop the image


## PROCESS CAMERA MESSAGE

Include `camera_drivers_process_image.hpp`, and build the image, undistorted, if necessary, with the function
```
int
process_image(camera_message *message, int image_index, char*camera_name, int undistort, cv::Mat &src_image);
```

Example: 

```
cv::Mat src_image;
process_image(message, 0, "intelbras1", 1, src_image);

imshow("retified", src_image);
waitKey(1);
```

## CHECK CAMERA Usage
```
./check_camera "rtsp://admin:1q2w3e4r@192.168.1.108:554/cam/realmonitor?channel=1&subtype=1"
./check_camera "/dev/video0"
```


## IMPORTANT

Common mistakes

* host: `rtsp://admin:1q2w3e4r@192.168.1.108:554/cam/realmonitor?channel=1&subtype=1`. Check the user:password (probably `admin:1q2w3e4r`)
* In web navigator, try to access `192.168.1.108` and sigin up with `admin:1q2w3e4r`. If doesnt work, there is probably something wrong.
* In case of VGA camera and delay, access `192.168.1.108` and Settings > Video > select the in the right camera *compress MJPEG*.


## To include a new camera in the .ini file:

```      
#--------- <Camera Name> ------------
# pre processing
<camera_name>_model				  *ip_opencv or ip_ffmpeg or <stream>
<camera_name>_fps                 *9999  # set to 9999 to get maximun FPS due to camera asynchronous image transmission
<camera_name>_number_of_images    *1
<camera_name>_ip_adress            rtsp://admin:1q2w3e4r@192.168.1.117:554/cam/realmonitor?channel=1\&subtype=0  # subtype=1: 640x480, subtype=0: 1280x720
                                   # MANDATORY IF MODEL IS ip_
<camera_name>_resize_factor		   1.0  # Used to reduce the image, to save in a smaller size

<camera_name>_x                    0.23
<camera_name>_y                    0.0
<camera_name>_z	                   0.127
<camera_name>_roll	               0.0
<camera_name>_pitch	        	   0.0
<camera_name>_yaw	               0.18

# Only used in post processing, not in published image
# post processing
<camera_name>_crop                 off
<camera_name>_crop_x_1             100
<camera_name>_crop_y_1             100
<camera_name>_crop_width_1         200
<camera_name>_crop_height_1        200

# Intrinsic parameters of the camera, obtained from calibration
# if you change the resolution, you have to recalibrate
<camera_name>_fx		           522.81945837
<camera_name>_fy		           700.16902439
<camera_name>_cu		           321.80096339
<camera_name>_cv		           244.4899013
<camera_name>_k1		          -0.7826785
<camera_name>_k2		           0.83042721
<camera_name>_p1		           0.00100615
<camera_name>_p2		          -0.00125759
<camera_name>_k3		          -0.44771437
```

*Parameters such width, height and raw fps will became from readed image.*


### Models

* /dev/video3, /dev/video2: old OpenCV
* ip_opencv: using OpenCV
* ip_ffmpeg: using the FFMPEG method


### HD Camera via RSTP

As we noticed using the OpenCV to get the RTSP camera image shows some buffer lag: [stackoverflow](https://stackoverflow.com/questions/48348173/is-there-a-way-to-avoid-buffering-images-when-using-rtsp-camera-in-opencv), [stackoverflow](https://stackoverflow.com/questions/60816436/open-cv-rtsp-camera-buffer-lag). To work around this issue, we implemented a pure FFMPEG image processor, acording to these [examples](https://github.com/FFmpeg/FFmpeg/tree/master/doc/examples), using the **libavcodec** libraries, installed by `sudo apt-get install libavcodec-dev`.

The equivalent command to produce the image is `ffplay -flags low_delay -vf setpts=0 rtsp://user:pass@host:post/cam/realmonitor?channel=1&subtype=0`.

subtype=0 = HD
subtype=1 = VGA

## For first configuration:

connect to the network

create a wired: 192.168.1.0

first login: http://192.168.1.108/

password 1q2w3e4r
phone (Pedro) +55 27 998107436
email lumerobotics@gmail.com

in the case of the VIP 1220, downgrade in the firmware (settings > update > via file), in the VIP 1230 G3 version - version 2021-09-27

* disable motion detection
* video: Resolution 1280x720(1MP) | 640x480(VGA)
* video: Frame Rate (FPS) 15
* disable Watermark
* interface: disable all
* networks: static IP, select IP


check

```
./check_camera rtsp://admin:1q2w3e4r@192.168.1.114:554/cam/realmonitor?channel=1&subtype=0
```

subtype=0 = HD
subtype=1 = VGA
