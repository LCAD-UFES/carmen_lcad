# Neural Car Detector

Detect vehicles using convolutional neural networks.

## Building
### Dependencies
To run this module the Nvidia Caffe is needed.
Install nvcaffe dependencies:

``` bash
$ sudo apt-get update -y
$ sudo apt-get install cmake -y

# General dependencies
$ sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev \
libhdf5-serial-dev protobuf-compiler -y
$ sudo apt-get install --no-install-recommends libboost-all-dev -y

# BLAS
$ sudo apt-get install libatlas-base-dev -y

# Remaining Dependencies
$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev -y

# Python Dependencies
$ sudo apt-get install python-dev python-numpy python-skimage python-protobuf -y
```
Install nvcaffe

``` bash
$ cd ~
$ git clone https://github.com/NVIDIA/caffe.git nvcaffe
$ cd nvcaffe
$ cp Makefile.config.example Makefile.config
```

If using gpu with CUDA capability lower than 3.0 change CUDA_ARCH lines, in Makefile.config to this

	CUDA_ARCH := -gencode arch=compute_20,code=sm_20 \
			-gencode arch=compute_20,code=sm_21 \
			-gencode arch=compute_50,code=sm_50 \
			-gencode arch=compute_52,code=sm_52 \
			-gencode arch=compute_60,code=sm_60 \
			-gencode arch=compute_61,code=sm_61 \
			-gencode arch=compute_61,code=compute_61

Uncomment the line, in Makefile.config

	WITH_PYTHON_LAYER := 1

If during compilation appear nvml.h error add the following line to Makefile.config:

	NO_NVML := 1
	
Then:

``` bash
$ make -j4
$ make pycaffe
```

Add the following lines to the end of .bashrc file

	# NVIDIA Caffe
	export NVCAFFE_HOME=~/nvcaffe 
	export PYTHONPATH=$NVCAFFE_HOME/python:/usr/lib/python2.7/dist-packages:$PYTHONPATH python
	export LD_LIBRARY_PATH=$NVCAFFE_HOME/build/lib:$LD_LIBRARY_PATH
	
Make and run neural car detector module. (Download caffemodel first: https://www.dropbox.com/s/075njerr4xvyz9u/snapshot_iter_21600.caffemodel?dl=0)

``` bash
$ cd $CARMEN_HOME/src/neural_car_detector
$ make
$ ./neural_car_detector 3 1
```