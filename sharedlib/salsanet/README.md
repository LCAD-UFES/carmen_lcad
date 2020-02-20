# LibSalsaNet

## Semantic segmentation inference lib using SalsaNet

* make mapper_datmo - it will make libsalsanet.a
```
:~/carmen_lcad/src/mapper_datmo$ make
```

## Installing dependencies

* It will be used the same virtualenv from SqueezeSegV2, so install SqueezeSegV2 first:
```
:~/carmen_lcad/sharedlib/salsanet$ $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/create_env.sh
```

* Go to salsanet folder, and activate virtualenv
```
:~/carmen_lcad/sharedlib/salsanet$ source $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/squeezeseg_env/bin/activate
```

* For using MatPlotLib, it is necessary to install python-tk with the follow command:
```
sudo apt-get install python-tk
```

* Install missing dependencies for SalsaNet:
```
:~/carmen_lcad/sharedlib/salsanet$ pip install -r requirements.txt
```

### Testing

For testing porpouses, start with SalsaNet IARA example:
```
~/carmen_lcad/sharedlib/salsanet/scripts$ python test_IARA_sample.py
```

##### Using SalsaNet with carmen:

Set variable `mapper_use_remission = on` in your carmen.ini file.

##### Run Central
```
:~/carmen_lcad/bin$ ./central
```

#### Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

#### Run mapper_datmo with salsanet:
```
:~/carmen_lcad/bin$ ./mapper_datmo -map_path ../data/mapper_teste2 -camera5 right -verbose 2 -file_warnings off -neural_network salsanet
```

Push Playback button. The results from segmentation will be at screen.




### If you would like to use TensorFlow CPU and is not working properly, do these instructions: 

Your CPU supports instructions that this TensorFlow binary was not compiled to use: AVX2 FMA

Download TensorFlow 1.11 with Python 2.7 at:

https://github.com/lakshayg/tensorflow-build

Execute the follow command, changing the path with dowloaded file with virtualenv activated:
pip install --ignore-installed --upgrade /path/to/binary.whl

Example:
pip install --ignore-installed --upgrade ~/Downloads/tensorflow-1.11.0-cp27-cp27mu-linux_x86_64.whl 

### If you would like to use tensorflow with GPU and tensorflow-gpu is not working properly:

Build tensorflow from source:
https://www.tensorflow.org/install/source

Use the steps for build tensorflow-1.11.0 with GPU support, python	2.7, GCC 4.8 and Bazel 0.15.0.
