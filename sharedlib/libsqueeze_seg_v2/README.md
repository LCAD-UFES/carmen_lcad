# LibSqueezeSegV2

## Semantic segmentation inference lib using SqueezeSegV2

* make mapper_datmo - it will make libsqueezeseg.a
:~/carmen_lcad/src/mapper_datmo$ make

## Installing dependencies

* Install python 2.7 -> ```make install_python2```

* Before installing the dependencies, if use without GPU, go to `requirements.txt` and change
`tensorflow-gpu==1.11.0` to `tensorflow==1.11.0`

* Create virtualenv
:~/carmen_lcad/bin$ $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/create_env.sh

### Testing
* For testing porpouses, start with SqueezeSegV2 example:
~/carmen_lcad/sharedlib/libsqueeze_seg_v2$ python ./src/demo.py

### Using SqueezeSegV2 with carmen:

Set variable mapper_use_remission = on in your carmen.ini file.

##### Run Central
```
:~/carmen_lcad/bin$ ./central
```

##### Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

##### Run mapper_datmo with squeezeseg:
```
:~/carmen_lcad/bin$ ./mapper_datmo -map_path ../data/mapper_teste2 -camera5 right -verbose 2 -file_warnings off -neural_network squeezeseg
```

Push Playback button. The results from segmentation will be at screen.

### If you have some problem with TensorFlow, do these instructions for using only CPU: 

Your CPU supports instructions that this TensorFlow binary was not compiled to use: AVX2 FMA

Download TensorFlow 1.11 with Python 2.7 at:

https://github.com/lakshayg/tensorflow-build

Execute the follow command, changing the path with dowloaded file with virtualenv activated:
pip install --ignore-installed --upgrade /path/to/binary.whl

Example:
pip install --ignore-installed --upgrade ~/Downloads/tensorflow-1.11.0-cp27-cp27mu-linux_x86_64.whl 

To install OpenCV in Python with virtualenv activated, do:
python -m pip install opencv-python

### If you would like to use tensorflow with GPU and tensorflow-gpu it is not working properly:

Build tensorflow from source:
https://www.tensorflow.org/install/source

Use the steps for build tensorflow-1.11.0 with GPU support, with python	2.7, GCC 4.8 and Bazel 0.15.0.
