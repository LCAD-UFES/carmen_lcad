Traffic Light Recognition Using Deep Learning and Prior Maps for Autonomous Cars
================================================================================

Autonomous terrestrial vehicles must be capable
of perceiving traffic lights and recognizing their current states
to share the streets with human drivers. Most of the time,
human drivers can easily identify the relevant traffic lights. To
deal with this issue, a common solution for autonomous cars
is to integrate recognition with prior maps. However, additional
solution is required for the detection and recognition of the traffic
light. Deep learning techniques have showed great performance
and power of generalization including traffic related problems.
Motivated by the advances in deep learning, some recent works
leveraged some state-of-the-art deep detectors to locate (and
further recognize) traffic lights from 2D camera images. However,
none of them combine the power of the deep learning-based
detectors with prior maps to recognize the state of the relevant
traffic lights. Based on that, this work proposes to integrate the
power of deep learning-based detection with the prior maps used
by our car platform IARA (acronym for Intelligent Autonomous
Robotic Automobile) to recognize the relevant traffic lights of
predefined routes. The process is divided in two phases: an offline
phase for map construction and traffic lights annotation; and an
online phase for traffic light recognition and identification of the
relevant ones. The proposed system was evaluated on five test
cases (routes) in the city of Vitória, each case being composed
of a video sequence and a prior map with the relevant traffic
lights for the route. Results showed that the proposed technique
is able to correctly identify the relevant traffic light along the
trajectory.

<!-- [Link to PDF.](example.com) -->

<!-- ![system-running](./misc/system_running.jpg) -->

## Videos

The following video demonstrates the proposed system working offline (not in the car).

<!-- https://stackoverflow.com/a/16079387/4630320 -->
[![youtube-video](https://img.youtube.com/vi/VhdLpuErJ8E/0.jpg)](https://youtu.be/VhdLpuErJ8E)

## Trained Models

Datasets used for training:
 - DriveU Traffic Light Dataset (DTLD): https://www.uni-ulm.de/en/in/driveu/projects/driveu-traffic-light-dataset/
 - LISA Traffic Light Dataset (LISA-TLD): https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset

We trained a [YOLOv3][yolo] model that can detect traffic lights and classify their state: red or yellow (in a single class); and green.
To run the inference using our trained model, you will need the following files:
 - Data (dummy): https://drive.google.com/open?id=1n14nQDpN_NAIn61JK5_EQYFJXPJM7Kai
 - Names: https://drive.google.com/open?id=1ZgUceNvhHm9XST0H_M_hi8hqk4wx9mI6
 - Cfg: https://drive.google.com/open?id=1dx6rKLPxnxu_YXzmsqr3D8IT5UTL_3Gu
 - Weights: https://drive.google.com/open?id=1CCbc-6FgCtXD2g_1jQfGdm0cWqfIwwbC

[yolo]: https://pjreddie.com/darknet/yolo/

After compiling [darknet] and downloading the previous files to the same place where the darknet binary is, run the folowing command to see the results on your own images.

[darknet]: https://github.com/pjreddie/darknet

```bash
./darknet detector test dummy_nrg.data yolov3-nrgr-10000-const-lr-1e-4.cfg yolov3-nrgr-10000-const-lr-1e-4_15000.weights YOUR_IMAGE.png
# Or, for multiple images (file paths will be read from STDIN):
./darknet detector test dummy_nrg.data yolov3-nrgr-10000-const-lr-1e-4.cfg yolov3-nrgr-10000-const-lr-1e-4_15000.weights
```

While doing inference you can also pass the option `-thresh 0.2` to the binary in order to decrease its confidence threshold to 0.2, like we do in our paper.
