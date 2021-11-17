## NOD3_tracker
tracker used: https://github.com/xingyizhou/CenterTrack
Create a virtualenv with python3.6
 sudo apt install virtualenv
 virtualenv --python=/usr/bin/python3.6 $CARMEN_HOME/src/neural_object_tracker/center_track_venv

Activate the virtualenv
 source $CARMEN_HOME/src/neural_object_tracker/center_track_venv/bin/activate

Install the requirements inside the virtualenv
 pip install torch==1.4.0+cu100 torchvision==0.5.0+cu100 -f https://download.pytorch.org/whl/torch_stable.html
 pip install cython; pip install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
 pip install -r $CARMEN_HOME/src/neural_object_tracker/center_track/requirements.txt

Make DCNv2
 cd $CARMEN_HOME/src/neural_object_tracker/center_track/src/lib/model/networks/DCNv2/
 ./make.sh

Download necessary data:
 cd $CARMEN_HOME/src/neural_object_tracker/
 make download

Test center_track
 cd $CARMEN_HOME/src/neural_object_tracker/center_track/src
 python demo.py tracking --dataset coco --load_model ../models/coco_tracking.pth --demo ../videos/out.mp4


Add NOD in PYTHONPATH at bashrc
 gedit ~/.bashrc
add at the end of file
```
 #NOD_Tracker - sempre por ultimo
 export PYTHONPATH=$CARMEN_HOME/src/neural_object_tracker/center_track/src:$PYTHONPATH
```



Compile the neural_object_tracker
 make

Run it:
./neural_object_detector_tracker 3 1
