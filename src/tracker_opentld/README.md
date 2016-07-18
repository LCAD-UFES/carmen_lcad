# Introduction

This is a C++ implementation of a tracker on CARMEN (Carnegie Mellon Robot Navigation Toolkit) with OpenTLD that was originally published in MATLAB by [Zdenek Kalal](http://personal.ee.surrey.ac.uk/Personal/Z.Kalal/tld.html). 

The OpenTLD implementation used in this module was develop by https://github.com/gnebehay/OpenTLD. What makes this algorithm outstanding is that it does not make use of any training data. This implementation is based solely on open source libraries, meaning that you do not need any commercial products to compile or run it.

More about the OpenTLD by gnebehay, the documentation of the internals as well as other possibly helpful information is contained in this [master thesis](https://github.com/downloads/gnebehay/OpenTLD/gnebehay_thesis_msc.pdf).

[Video Experimento - Volta da UFES](https://youtu.be/fThYL5N-aAw)

# Building
## Dependencies
* OpenCV
* libconfig++ (optional)
* Visual_Tracker module
* bumblebee_basic module (this module subscribes to `carmen_bumblebee_basic_stereoimage_message` with retified image)

## Compiling

After compile carmen

This module publish the messages defined by the visual_tracker module (`carmen_visual_tracker_output_message`) because of that, it needs to be compiled as well.

Navigate with the terminal to the directory of Visual_Tracker Module.
```bash
cd $CARMEN_HOME/src/visual_tracker/
make
```
Navigate with the terminal to the directory of this module Tracker_OpenTLD
```bash
cd $CARMEN_HOME/src/tracker_opentld/
make
```

# Usage
## Keyboard shortcuts

* `r` clear model, let user reinit tracking
* `c` clear model and stop tracking
* `t` show trajectory
* `l` toggle learning
* `a` toggle alternating mode (if true, detector is switched off when tracker is available)
* `b` remember current frame as background model / clear background

## Run
Check the carmen-ford-escape.ini if you want to modify some of the parameters

- Visual Tracker OpenTLD parameters
```bash
  tracker_opentld_view_width				640 --window and image width (can be equal to bumbeblee)
  tracker_opentld_view_height				480 --window and image height (can be equal to bumbeblee)
  tracker_opentld_confidence_threshold	0.5 --Confidence detection threshold
  tracker_opentld_detector_minScale		-10 --number of scales smaller than initial object size
  tracker_opentld_detector_maxScale		10 --number of scales larger than initial object size
  tracker_opentld_detector_numFeatures	15 --number of features
  tracker_opentld_detector_minSize		25 --minimum size of scanWindows
  tracker_opentld_detector_thetaP			0.65 --detector parameters
  tracker_opentld_detector_thetaN			0.5 --detector parameters
```

Run the Central:
`./central`

##### Proccontrol options
This process already have the option to run tracker_opentld, check the camera_number before start.

play a log with camera using

`./proccontrol process-volta_da_ufes_playback_viewer_3D.ini`

  
#### Command line options

Connect and turn on a Bumbeblee camera

Navigate with the terminal to the directory and run
```bash
cd $CARMEN_HOME/src/tracker_opentld/
./tracker_opentld <camera_number> <camera_side (0-left; 1-right)>
```
To test and see the published messages on terminal
```bash
./tracker_test
```
