## NOD3_tracker
Create a virtualenv with python2.7
 sudo apt install virtualenv
 virtualenv --python=/usr/bin/python2.7 $CARMEN_HOME/src/neural_object_detector3/nod_venv2

Activate the virtualenv
source $CARMEN_HOME/src/neural_object_detector3/nod_venv2/bin/activate

Install the requirements inside the virtualenv
 pip install -r $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/requirements.txt

Inside the pedestrian_tracker folder and with the virtualenv activated run:
 cd $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/
 sh make.sh

Add NOD on PYTHONPATH at bashrc
 gedit ~/.bashrc
add at the end of file
```
 #NOD_Tracker - sempre por ultimo
 export PYTHONPATH=$CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker:$PYTHONPATH
```


Create the folder data
 mkdir $CARMEN_HOME/src/neural_object_detector3/pedestrian_tracker/data
Download the models to the folder data
https://drive.google.com/drive/folders/1yeee_RDNnc7Fa1bn_SnIi0snwhe090BB?usp=sharing

Compile the neural_object_detector3

Run it:
./neural_object_detection_tracker 3 1
