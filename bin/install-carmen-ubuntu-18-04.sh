#!/bin/bash

echo "Atualizar o apt:"

sudo apt-get update

echo "Instalar o git:"

sudo apt-get -y install git

echo "Alternativamente pode-se usar o gh:"

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key C99B11DEB97541F0
sudo apt-add-repository https://cli.github.com/packages
sudo apt update
sudo apt install gh

echo "Instalar as bibliotecas/pacotes abaixo:"

sudo apt-get -y install swig \
libgtk2.0-dev \
libimlib2 libimlib2-dev \
imagemagick libmagick++-dev \
libwrap0 libwrap0-dev tcpd \
libncurses5 libncurses5-dev libgsl23 \
libdc1394-22 libdc1394-22-dev libdc1394-utils \
libgtkglext1 libgtkglext1-dev \
libgtkglextmm-x11-1.2-0v5 \
libglade2-0 libglade2-dev \
freeglut3 freeglut3-dev \
libcurl4 libcurl4-nss-dev \
libkml-dev \
liburiparser1 liburiparser-dev \
libusb-1.0-0 libusb-1.0-0-dev libusb-dev \
libxi-dev libxi6 \
libxmu-dev libxmu6 \
build-essential libforms-dev \
byacc flex doxygen

sudo apt-get -y install libgflags-dev 

sudo apt-get -y install libespeak-dev libfftw3-dev pkg-config \
libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev \
libjpeg-dev libpng-dev libpng++-dev libtiff-dev \
libgstreamer-plugins-base1.0-dev gimp meld vim \
python-numpy python-dev python-pip python-wheel python3-numpy python3-dev \
python3-pip python3-wheel \
g++ mpi-default-dev openmpi-bin openmpi-common \
libqhull* libgtest-dev git-core \
libflann1.9 libboost1.65-all-dev \
libeigen3-dev \
libboost-all-dev libflann-dev libproj-dev libsuitesparse-dev libqt4-dev qt4-qmake \
make libtiff5-dev tcsh wget \
linux-headers-`uname -r` kmod libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev \
libgtkglextmm-x11-1.2-dev libudev-dev \
libvtk6* \
qttools5-dev libasound2-dev \
mpg123 portaudio19-dev libjsoncpp-dev \
libglew2.0 libglew-dev \
libgtk-3-dev \
cmake cmake-curses-gui cmake-qt-gui

sudo apt-get -y install libgsl0-dev

sudo apt-get -y install libimlib2-dev

sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt -y install libjasper1 libjasper-dev
sudo updatedb

echo "Instalar CUDA 11"

sudo apt-get install g++ freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin

sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
sudo apt-get update
sudo apt-get -y install nvidia-driver-515
sudo apt-get -y install cuda-11-2 cuda-11-3
sudo apt-get -y install libcupti-dev
sudo apt-get -y install libcudnn8-dev

sudo apt-key del 7fa2af80
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb

echo "Instalar Eclipse"

sudo add-apt-repository ppa:linuxuprising/java
sudo apt update && sudo apt install oracle-java17-set-default
sudo update-alternatives --config java
sudo echo 'JAVA_HOME=/usr/lib/jvm/java-17-oracle/bin/java' >> /etc/environment

wget https://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/2022-06/R/eclipse-cpp-2022-06-R-linux-gtk-x86_64.tar.gz
tar -xvf eclipse-cpp-2022-06-R-linux-gtk-x86_64.tar.gz
sudo mv eclipse /opt

sudo echo "[Desktop Entry] 
Name=Eclipse 
Type=Application 
Exec=/opt/eclipse/eclipse 
Terminal=false 
Icon=/opt/eclipse/icon.xpm
Comment=Integrated Development Environment
NoDisplay=false
Categories=Development;IDE
Name[en]=Eclipse" > /usr/share/applications/eclipse.desktop


mkdir ~/packages_carmen

cd ~/packages_carmen

echo "Instalar OpenCV"

wget https://github.com/opencv/opencv/archive/3.2.0.tar.gz -O opencv_3.2.0.tar.gz && wget https://github.com/opencv/opencv_contrib/archive/3.2.0.tar.gz -O opencv_contrib-3.2.0_.tar.gz && tar xvzf opencv_3.2.0.tar.gz && tar xzvf opencv_contrib-3.2.0_.tar.gz && cd opencv-3.2.0/ && mkdir build && cd build/ && cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D  WITH_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D  BUILD_PERF_TESTS=OFF -D INSTALL_C_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.2.0/modules -D CMAKE_BUILD_TYPE=RELEASE .. && make -j12 && sudo make install && sudo ldconfig && sudo updatedb

echo "Instalar bullet, FANN e Kvaser SDK"

cd ~/packages_carmen && sudo apt-get update && sudo apt-get -y install libbullet-dev && wget http://downloads.sourceforge.net/project/fann/fann/2.2.0/FANN-2.2.0-Source.tar.gz && wget "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest" -O linuxcan.tar.gz && tar -xvf linuxcan.tar.gz && tar -xvf FANN-2.2.0-Source.tar.gz

cd linuxcan && make && sudo make install && cd ../FANN-2.2.0-Source && mkdir build && cd build && cmake .. && make && sudo make install && cd ~/packages_carmen

echo "Instalar a versão 2.11 do flycapture"

git clone https://github.com/RhobanDeps/flycapture.git && cd flycapture && sudo sh install_flycapture.sh && cd ..

echo "Instalar bibliotecas para usar KINECT"

wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.19/libusb-1.0.19.tar.bz2 && tar -xvf libusb-1.0.19.tar.bz2 && cd libusb-1.0.19 && ./configure && make && sudo make install && cd .. && wget https://github.com/OpenKinect/libfreenect/archive/v0.5.7.tar.gz && tar -xzvf v0.5.7.tar.gz && cd libfreenect-0.5.7 && mkdir build && cd build && cmake .. && cp src/libfreenect.pc ../../ &&  make && cp ../src/libfreenect.pc.in src/libfreenect.pc && cp ../fakenect/fakenect.sh.in fakenect/fakenect.sh && sudo make install && sudo ldconfig /usr/local/lib64/ && cd ../..

echo "Instalar biblioteca G2O"

cd ~/packages_carmen && git clone https://github.com/LCAD-UFES/g2o-1.git && mv g2o-1 g2o && cd g2o && mkdir build && cd build && cmake .. && make -j 12 && sudo make install && cd ../..

sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so /usr/lib/x86_64-linux-gnu/libboost_thread-mt.so

echo "Instalar dlib:"

cd ~/packages_carmen && git clone https://github.com/davisking/dlib.git && cd dlib/ && git checkout -b v19.17 &&  git pull origin v19.17 && mkdir build && cd build && cmake .. && make -j 8 && sudo make install && cd ../..

echo "Instalar libwnn: "

git clone http://github.com/filipemtz/libwnn && cd libwnn && mkdir build && cd build && cmake .. && make -j 8 && sudo make install && cd ../..

echo "Instalar PCL 1.8 "

sudo apt -y install libpcl-dev pcl-tools libpcl-dev

# wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz && tar -xf pcl-1.8.0.tar.gz && cd pcl-pcl-1.8.0 && mkdir build && cd build && cmake .. && make && sudo make install && cd ../..



