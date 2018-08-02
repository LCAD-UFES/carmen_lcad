# Installing Carmen LCAD on Ubuntu 14.04

These are the instructions for a fresh install of CARMEN LCAD. It is expected that you already have minimal experience with Linux and know how to use the Terminal.  

## What do you need before starting?
+ A machine running Ubuntu 14.04.5 LTS. Other versions may work, but are not guaranteed. You can find it [here](http://releases.ubuntu.com/14.04/). We advise against using a virtual machine because of the performance.  
+ Patience. This is a long process and requires attention while following the steps.

## Installation process

### Initial Downloads
Update apt:
```
sudo apt-get update
```
Install the packages of subversion, gimp, meld, vim and git:
```
sudo apt-get install gimp meld subversion vim git
```
>**Download the next two items while doing the next steps.**

Download Carmen through git :
```
git clone https://github.com/LCAD-UFES/carmen_lcad.git
```
Download MAE through git:
```
git clone https://github.com/LCAD-UFES/MAE.git
```
### Install the dependencies
Install this packages required by Carmen:
```
sudo apt-get install swig \
libgtk2.0-dev \
qt-sdk \
libimlib2 libimlib2-dev \
imagemagick libmagick++-dev \
libwrap0 libwrap0-dev tcpd \
libncurses5 libncurses5-dev \
libgsl0-dev libgsl0ldbl \
libdc1394-22 libdc1394-22-dev libdc1394-utils \
cmake \
libgtkglext1 libgtkglext1-dev \
libgtkglextmm-x11-1.2-0 libgtkglextmm-x11-1.2-dev \
libglade2-0 libglade2-dev \
freeglut3 freeglut3-dev \
libcurl3 libcurl3-nss libcurl4-nss-dev \
libglew1.5 libglew1.5-dev libglewmx1.5 libglewmx1.5-dev libglew-dev \
libkml0 libkml-dev \
liburiparser1 liburiparser-dev \
libusb-1.0-0 libusb-1.0-0-dev libusb-dev \
libxi-dev libxi6 \
libxmu-dev libxmu6 \
build-essential libforms-dev \
byacc \
flex \
doxygen \
libespeak-dev libfftw3-dev \
libpng++
```
In the case an error occur because of  libcheese-gtk23, install this packages and try again: 
```
sudo apt-get install libglew-dev libcheese7 libcheese-gtk23 libclutter-gst-2.0-0 libcogl15 libclutter-gtk-1.0-0 libclutter-1.0-0
```
### Install Java
```
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update && sudo apt-get install oracle-java8-installer
update-alternatives --display java
```
Edit the file /etc/environment:
```
sudo gedit /etc/environment
```
Add to the end of the file:
```
JAVA_HOME=/usr/lib/jvm/java-8-oracle
```
### Install Eclipse
Download Eclipse from: 
```
https://www.eclipse.org/downloads/eclipse-packages/
```
It's recommended to utilize the Eclipse IDE for C/C++ Developers. The current version of Eclipse is eclipse-cpp-mars-1-linux-gtk-x86_64.tar.gz

Unzip eclipse:
```
cd Downloads/
sudo mv eclipse-cpp-mars-1-linux-gtk-x86_64.tar.gz  /opt
cd /opt/
sudo tar -xvf eclipse-cpp-mars-1-linux-gtk-x86_64.tar.gz 
```
Create a desktop link e edit it in /usr/share/applications:
```
sudo gedit /usr/share/applications/eclipse.desktop
```
Put the content below in file :
```
[Desktop Entry]
Name=Eclipse
Type=Application
Exec=/opt/eclipse/eclipse
Terminal=false
Icon=/opt/eclipse/icon.xpm
Comment=Integrated Development Environment
NoDisplay=false
Categories=Development;IDE
Name[en]=Eclipse
```
### Install the dependencies of PCL:
```
sudo apt-get install libeigen3-dev libboost-all-dev libflann-dev libvtk5-dev cmake-gui
```
### Install OpenCV 3.1:
Start installing the dependencies:
```
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
Download and select the correct version:
```
sudo mkdir /opt/opencv3.1.0/
cd /opt/opencv3.1.0/
sudo git clone https://github.com/Itseez/opencv.git
sudo git clone https://github.com/Itseez/opencv_contrib.git
cd opencv
sudo git checkout 3.1.0
cd /opt/opencv3.1.0/opencv_contrib
sudo git checkout 3.1.0
cd /opt/opencv3.1.0/opencv
```
Build it:
```
sudo mkdir build
cd build
sudo cmake -D -DWITH_IPP=ON -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_EXTRA_MODULES_PATH=/opt/opencv3.1.0/opencv_contrib/modules /opt/opencv3.1.0/opencv/ ..
sudo make -j8
sudo make install
```
It is common to have problems with ippicv using this version of OpenCV. To fix it:
+ Edit `/usr/local/lib/pkgconfig/opencv.pc`
+ Add `libdir3rd=${exec_prefix}/share/OpenCV/3rdparty/lib` after libdir
+ Add `-L${libdir3rd}` at Libs  

### Install bullet, FANN and linux-CAN:
Download and extract:
```
sudo su
cd /usr/local/
wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/bullet/bullet-2.78-r2387.tgz
wget http://downloads.sourceforge.net/project/fann/fann/2.2.0/FANN-2.2.0-Source.tar.gz
wget http://www.kvaser.com/software/7330130980754/V5_3_0/linuxcan.tar.gz
tar -xvf bullet-2.78-r2387.tgz 
tar -xvf linuxcan.tar.gz
tar -xvf FANN-2.2.0-Source.tar.gz
mv bullet-2.78 bullet
```
Build:
```
cd bullet
./configure
make
make install
cd ..
cd linuxcan
make
make install
cd ..
cd FANN-2.2.0-Source
mkdir build
cd build
cmake ..
make
make install
```
### Configure the paths of installed packages 
Edit the file /etc/ld.so.conf.d/opencv.conf:
```
gedit /etc/ld.so.conf.d/opencv.conf
```
Add to it's end:
```
/usr/local/lib
```
Execute:
```
ldconfig
```
edit /etc/bash.bashrc:
```
gedit /etc/bash.bashrc
```
Add to it's end:
```
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH
```
Exit root:
```
exit
```
Open the file ~/.bashrc:
```
gedit ~/.bashrc
```
Place in yours the content below:
```
#CARMEN
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu/:/usr/lib/libkml
export CARMEN_HOME=~/carmen_lcad

#OpenJaus
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARMEN_HOME/sharedlib/OpenJAUS/libopenJaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/libjaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojTorc/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojIARASim/lib

#Darknet
export DARKNET_HOME=$CARMEN_HOME/sharedlib/darknet
export LD_LIBRARY_PATH=$DARKNET_HOME/lib:$LD_LIBRARY_PATH

#MAE
export MAEHOME=~/MAE
export PATH=$PATH:$MAEHOME/bin
```
### Install imlib and flycapture: 
```
cd $CARMEN_HOME/ubuntu_packages/
sudo dpkg -i imlib_1.9.15-20_amd64.deb 
sudo dpkg -i imlib-devel_1.9.15-20_amd64.deb
tar -xvf flycapture2-2.5.3.4-amd64-pkg.tgz
cd flycapture2-2.5.3.4-amd64/
sudo apt-get install libglademm-2.4-1c2a libglademm-2.4-dev libgtkmm-2.4-dev libudev-dev
sudo sh install_flycapture.sh
```
### Make some links:
```
sudo ln -s /usr/lib64/libgdk_imlib.so.1.9.15 /usr/lib64/libgdk_imlib.a
sudo ln -s /usr/src/linux-headers-3.8.0-30/ /usr/src/linux
```
### Install PCL:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```
### Install Kinect Camera:
```
sudo su
cd /usr/local
wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.19/libusb-1.0.19.tar.bz2
tar xvf libusb-1.0.19.tar.bz2
cd libusb-1.0.19
./configure
make
make install
```
if a error occur during installation, execute:
```
sudo apt-get install libudev-dev
```
### Install TPLIB:
```
mkdir /usr/local/tplib
cd /usr/local/tplib
git clone git://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake ..
cp src/libfreenect.pc /usr/local/tplib/
make
cp ../src/libfreenect.pc.in src/libfreenect.pc 
cp ../fakenect/fakenect.sh.in fakenect/fakenect.sh
make install
ldconfig /usr/local/lib64/
exit
```
Execute:
```
glview
```
If give error, try:
```
freenect-glview
```
If give error, execute:
```
sudo ldconfig /usr/local/lib
```
### Install library G2O:
```
sudo apt-get install cmake libsuitesparse-dev libqt4-dev qt4-qmake
cd /usr/local/
sudo svn co https://svn.openslam.org/data/svn/g2o
cd /usr/local/g2o/trunk/build/
sudo cmake ../ -DBUILD_CSPARSE=ON -DG2O_BUILD_DEPRECATED_TYPES=ON -DG2O_BUILD_LINKED_APPS=ON
sudo make
sudo make install
```
### Create the link of boost:
```
sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so /usr/lib/x86_64-linux-gnu/libboost_thread-mt.so
```
### Install MAE:
First, it's dependencies:
```
sudo apt-get install make g++ freeglut3-dev byacc libforms-dev libtiff4-dev libXi-dev libXmu-dev doxygen tcsh flex libdc1394-22-dev
```
Now, MAE itself:
+ Enter MAE directory:
```
cd $MAEHOME
```
+ Compile MAE:
```
make
```
+ Check if $MAEHOME/lib/libnet_conn.a was created  
+ Check if $MAEHOME/bin/netcomp was created 

### Install Dlib:
```
cd /usr/local
sudo su
git clone https://github.com/davisking/dlib.git
cd dlib/
git checkout v19.4 -b dlib_v19.4
mkdir build
cd build/
In the IARA's computers, compile without CUDA:
cmake -D DLIB_USE_CUDA=OFF ..
If you need use CUDA, just to:
cmake .. 
make
make install
```
### Install libwnn
```
cd /usr/local
git clone https://github.com/filipemtz/libwnn.git
cd libwnn
mkdir build
cd build
cmake ..
make -j 4
make install
exit
```

If cmake failed to find OpenCV, remove the generated files from the build directory, and run cmake with the option -DOpenCV_DIR=[path to directory build of your opencv installation]. Ex.:

```
cd /usr/local/libwnn/build
rm -rf *
cmake -DOpenCV_DIR=/opt/opencv3.1.0/opencv/build/ ..
make -j 8
make install
exit
```

### Install cuDNN

Note: Install cuDNN library only if your computer has a NVIDIA GPU. First install the proper NVIDIA driver, then the NVIDIA CUDA Toolkit 7.5 or 8.0. Version 7.5 does not support NVIDIA Pascal architecture (GTX 1060, GTX 1070, GTX 1080, Titan X). For these GPU models use the CUDA Toolkit 8.0.

If using CUDA Toolkit 7.5 download cuDNN files by clicking here: [cudnn-7.5-linux-x64-v5.1.tgz](https://developer.nvidia.com/cudnn)
```
cd ~/Downloads
tar -xzvf cudnn-7.5-linux-x64-v5.1.tgz
sudo cp -rH cuda/* /usr/local/cuda-7.5/
rm -rf cuda
```

If using CUDA Toolkit 8.0 download cuDNN files by clicking here: [cudnn-8.0-linux-x64-v5.1.tgz](https://developer.nvidia.com/cudnn)
```
cd ~/Downloads
tar -xzvf cudnn-8.0-linux-x64-v5.1.tgz
sudo cp -rH cuda/* /usr/local/cuda-8.0/
rm -rf cuda
```

Make sure ~/.bashrc contains the following:
```
#CUDA and cuDNN
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDA_LIBS=/usr/local/cuda/lib64
```

### Install Caffe

Note: Caffe may be installed even if your computer does not have NVIDIA GPU, CUDA Toolkit, cuDNN library. In this case Caffe will be installed with the CPU_ONLY flag activated.

```
sudo apt-get install libgoogle-glog-dev protobuf-compiler libhdf5-dev liblmdb-dev libleveldb-dev libsnappy-dev libatlas-base-dev
cd $CARMEN_HOME/sharedlib/ENet/caffe-enet
mkdir build && cd build
cmake ..
make all -j 8
make pycaffe
```
If cmake fails to find OpenCV, remove the generated files from the build directory, and run cmake with the option -DOpenCV_DIR=[path to directory build of your opencv installation]. For example:

```
cd $CARMEN_HOME/sharedlib/ENet/caffe-enet/build
rm -rf *
cmake -DOpenCV_DIR=/opt/opencv3.1.0/opencv/build/ ..
make -j 8
make pycaffe
sudo apt-get install python-matplotlib python-numpy python-pil python-scipy build-essential cython python-skimage python-protobuf
```

If cmake fails, you may compile with **make**, uncommenting the following line in the Makefile.config. Please refer to the generic [Caffe installation guide](http://caffe.berkeleyvision.org/installation.html) for further help. 
```
WITH_PYTHON_LAYER := 1 
```

Please add the following to your ~/.bashrc file:
```
#Caffe ENet
export CAFFE_ENET_HOME=$CARMEN_HOME/sharedlib/ENet/caffe-enet
export CAFFE_HOME=$CAFFE_ENET_HOME
export PYTHONPATH=$CAFFE_ENET_HOME/python
export LD_LIBRARY_PATH=$CAFFE_ENET_HOME/build/lib:$LD_LIBRARY_PATH
```

In order to suppress the verbose messages of libcaffe, please add the following to your ~/.bashrc file:
```
export GLOG_minloglevel=1
```

Check the existence of file $CAFFE_ENET_HOME/include/caffe/proto/caffe.pb.h. If not present, please do the following: 
```
cd $CAFFE_ENET_HOME
protoc src/caffe/proto/caffe.proto --cpp_out=.
mkdir include/caffe/proto
mv src/caffe/proto/caffe.pb.h include/caffe/proto/
```

### Install CARMEN
Close all terminals and execute:
```
cd $CARMEN_HOME/src
./configure --nojava --nocuda --nozlib
  Should the C++ tools be installed for CARMEN: [Y/n] Y
  Should Python Bindings be installed: [y/N] N
  Searching for Python2.4... Should the old laser server be used instead of the new one: [y/N] N
  Install path [/usr/local/]: 
  Robot numbers [*]: 1,2
```
Before compiling CARMEN, we need to compile the module tracker beforehand for navigator_spline to work:
```
cd $CARMEN_HOME/src/tracker
make
```
It will give an error compiling, but it's okay.

To compile CARMEN, run:
```
cd $CARMEN_HOME/src
make
```
If give error of libusb.h, edit the file <code>/usr/local/include/libfreenect.hpp</code>:
```
sudo vim /usr/local/include/libfreenect.hpp
```
And alter the line `#include <libusb.h>` to:
```
#include <libusb-1.0/libusb.h>
```
If an error occur because of z_stream, try:
```
cd $CARMEN_HOME/src
make clean
./configure --nojava --nocuda
  Should the C++ tools be installed for CARMEN: [Y/n] Y
  Should Python Bindings be installed: [y/N] N
  Searching for Python2.4... Should the old laser server be used instead of the new one: [y/N] N
  Install path [/usr/local/]: 
  Robot numbers [*]: 1,2
make
```
