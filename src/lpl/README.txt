Download weights from Darknet3:

yolov4.weights: https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights (save in $CARMEN_HOME/sharedlib/darknet3)


## NOD3_tracker
Create a virtualenv with python2.7
 sudo apt install virtualenv
 virtualenv --python=/usr/bin/python2.7 $CARMEN_HOME/src/neural_object_detector3/nod_venv2

Activate the virtualenv
source $CARMEN_HOME/src/neural_object_detector3/nod_venv2/bin/activate

Install the requirements inside the virtualenv
 pip install -r $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/requirements.txt

Modify the make file to compile cuda to your device capability archtecture
open the file make.sh at:
 gedit $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/models/psroi_pooling/make.sh
Change the flag -arch=sm_35 for -arch=sm_XX where XX is your CUDA Capability.
to check your CUDA Capability, compile and run the deviceQuery
  sudo make -C /usr/local/cuda/samples/1_Utilities/deviceQuery
run:
  /usr/local/cuda/samples/1_Utilities/deviceQuery/deviceQuery

Example:
""""
Device 0: "Tesla K40c"
  CUDA Driver Version / Runtime Version          10.1 / 9.0
  CUDA Capability Major/Minor version number:    3.5
""""""
OBS.: It seems that the tracking network wasn't work with Capability < 6.0.
If you try and succed with lower capability, please disclose to humanity ;).

Inside the pedestrian_tracker folder and with the virtualenv activated run:
 cd $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/
 sh make.sh

Add NOD in PYTHONPATH at bashrc
 gedit ~/.bashrc
add at the end of file
```
 #NOD_Tracker - sempre por ultimo
 export PYTHONPATH=$CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker:$PYTHONPATH
```

Download necessary data:
 cd $CARMEN_HOME/src/neural_object_detector3/
 make download

Compile the neural_object_detector3
 make

Run it:
./neural_object_detector_tracker intelbras1 1
