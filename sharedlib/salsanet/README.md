# LibSalsaNet

## Semantic segmentation inference lib using SalsaNet

#Make
:~/carmen_lcad/src/mapper_datmo$ make


## Installing dependencies

Install python 2.7 -> ```make install_python2```

Before installing the dependencies, if use without GPU, go to `requirements.txt` and change
`tensorflow-gpu==1.11.0` to `tensorflow==1.11.0`

For working from any place, go to your .bashrc and insert the line at the end of file:

```
# SalsaNet
export PYTHONPATH=$CARMEN_HOME/sharedlib/salsanet/scripts:$PYTHONPATH
```
Save and then refresh your bash with:
```
bash
```

###Installing virtualenv
:~/carmen_lcad/bin$ /$CARMEN_HOME/sharedlib/salsanet/create_env.sh

#Activating virtualenv
:~/carmen_lcad/bin$ source $CARMEN_HOME/sharedlib/salsanet/squeezeseg_env/bin/activate

### Testing
For testing porpouses, start with SalsaNet IARA example:
~/carmen_lcad/sharedlib/salsanet/scripts$ python test_IARA_sample.py

#Using SalsaNet with carmen:

Set variable mapper_use_remission = on in your carmen.ini file.

Run Central
```
:~/carmen_lcad/bin$ ./central
```

Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

Run mapper_datmo:
```
:~/carmen_lcad/bin$ ./mapper_datmo -map_path ../data/mapper_teste2 -camera5 right -verbose 2 -file_warnings off
```

Push Playback button. The results from segmentation will be at screen.

### If you have some problem with TensorFlow, do these instructions: 

Your CPU supports instructions that this TensorFlow binary was not compiled to use: AVX2 FMA

Download TensorFlow 1.11 with Python 2.7 at:

https://github.com/lakshayg/tensorflow-build

Execute the follow command, changing the path with dowloaded file with virtualenv activated:
pip install --ignore-installed --upgrade /path/to/binary.whl

Example:
pip install --ignore-installed --upgrade ~/Downloads/tensorflow-1.11.0-cp27-cp27mu-linux_x86_64.whl 

To install OpenCV in Python with virtualenv activated, do:
python -m pip install opencv-python

For using MatPlotLib, it is necessary to install python-tk with the follow command:
sudo apt-get install python-tk


### Useful links

https://stackoverflow.com/questions/8998499/virtual-environments-and-embedding-python
https://virtualenv.pypa.io/en/latest/userguide/#using-virtualenv-without-bin-python
