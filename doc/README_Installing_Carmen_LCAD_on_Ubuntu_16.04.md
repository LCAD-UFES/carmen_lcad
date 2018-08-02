# Installing Carmen LCAD on Ubuntu 16.04

=== 1) Atualizar o apt: 
  sudo apt-get update 

=== 2) Instalar o git, gimp, meld e vim: 
  sudo apt-get install gimp meld vim git

 
=== 3) Baixar o Carmen e a MAE pelo git (faça o download enquanto segue os próximos passos): 
  git clone https://github.com/LCAD-UFES/carmen_lcad 
  git clone https://github.com/LCAD-UFES/MAE.git

=== 4) Instalar as bibliotecas/pacotes abaixo abaixo
  sudo apt-get install swig \
 libgtk2.0-dev \
 qt-sdk \
 libimlib2 libimlib2-dev \
 imagemagick libmagick++-dev \
 libwrap0 libwrap0-dev tcpd \
 libncurses5 libncurses5-dev \
 libgsl2 libgsl2:i386 \
 libdc1394-22 libdc1394-22-dev libdc1394-utils \
 cmake \
 libgtkglext1 libgtkglext1-dev \
 libgtkglextmm-x11-1.2-0v5:i386 libgtkglextmm-x11-1.2-0v5 \
 libglade2-0 libglade2-dev \
 freeglut3 freeglut3-dev \
 libcurl3 libcurl3-nss libcurl4-nss-dev \
 libglew1.5 libglew1.5-dev libglewmx1.5 libglewmx1.5-dev libglew-dev \
 libkml-dev \
 liburiparser1 liburiparser-dev \
 libusb-1.0-0 libusb-1.0-0-dev libusb-dev \
 libxi-dev libxi6 \
 libxmu-dev libxmu6 \
 build-essential libforms-dev \
 byacc \
 flex \
 doxygen \
 libespeak-dev libfftw3-dev build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libgstreamer-plugins-base1.0-dev

=== 5) Baixar o driver adequado para a gpu da máquina (no caso da Tesla K40 é o 390.46) e siga os passos abaixo para instalar:

 i) dpkg -i nvidia-diag-driver-local-repo-ubuntu1604-390.46_1.0-1_amd64.deb
 ii) apt-get update
 iii) apt-get install cuda-drivers
 iv) reboot 


=== 6) instalar o Cuda 9.1 (somente se tiver placa GPU instalada na maquina)

  https://developer.nvidia.com/cuda-91-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=deblocal

  sudo apt-key add /var/cuda-repo-ubuntu1604-9-1-local_9.1.85-1_amd64/7fa2af80.pub
  sudo dpkg -i cuda-repo-cuda-repo-ubuntu1604-9-1-local_9.1.85-1_amd64.deb
  sudo apt-key adv --fetch-keys 
  sudo apt-get update
  sudo apt-get install cuda

=== 6.1) testar a instalacao
  cd /usr/local/cuda-9.1/samples/1_Utilities/deviceQuery
  sudo make 
  ./deviceQuery

se tudo estiver ok irá aparecer uma tela com os dados da placa e o resultado do teste:

deviceQuery, CUDA Driver = CUDART, CUDA Driver Version = 9.1, CUDA Runtime Version = 9.1, NumDevs = 2
Result = PASS


=== 7) Instalar o CuDNN  (somente se tiver placa GPU instalada na maquina)

=== 7.1 - baixar o CuDnn 7.1.3 para Cuda 9.1

acesse https://developer.nvidia.com/cudnn , faça login e baixe os arquivos:

  cuDNN v7.1.3 Runtime Library for Ubuntu16.04 (Deb)
  cuDNN v7.1.3 Developer Library for Ubuntu16libgsl2.04 (Deb)
  cuDNN v7.1.3 Code Samples and User Guide for Ubuntu16.04 (Deb)

=== 7.2 - instalar
  sudo dpkg -i libcudnn7_7.1.3.16-1+cuda9.1_amd64.deb
  sudo dpkg -i libcudnn7-dev_7.1.3.16-1+cuda9.1_amd64.deb
  sudo dpkg -i libcudnn7-doc_7.1.3.16-1+cuda9.1_amd64.deb


=== 7.3 – Para retirar os warnning, edite o arquivo cudnn.hpp , e atualize a função cudnnGetErrorString pelas linhas abaixo.

 inline const char* cudnnGetErrorString(cudnnStatus_t status) {
   switch (status) {
     case CUDNN_STATUS_SUCCESS:
       return "CUDNN_STATUS_SUCCESS";
     case CUDNN_STATUS_NOT_INITIALIZED:
       return "CUDNN_STATUS_NOT_INITIALIZED";
     case CUDNN_STATUS_ALLOC_FAILED:
       return "CUDNN_STATUS_ALLOC_FAILED";
     case CUDNN_STATUS_BAD_PARAM:
       return "CUDNN_STATUS_BAD_PARAM";
     case CUDNN_STATUS_INTERNAL_ERROR:
       return "CUDNN_STATUS_INTERNAL_ERROR";
     case CUDNN_STATUS_INVALID_VALUE:
       return "CUDNN_STATUS_INVALID_VALUE";
     case CUDNN_STATUS_ARCH_MISMATCH:
       return "CUDNN_STATUS_ARCH_MISMATCH";
     case CUDNN_STATUS_MAPPING_ERROR:
       return "CUDNN_STATUS_MAPPING_ERROR";
     case CUDNN_STATUS_EXECUTION_FAILED:
       return "CUDNN_STATUS_EXECUTION_FAILED";
     case CUDNN_STATUS_NOT_SUPPORTED:
       return "CUDNN_STATUS_NOT_SUPPORTED";
     case CUDNN_STATUS_LICENSE_ERROR:
       return "CUDNN_STATUS_LICENSE_ERROR";
 #if CUDNN_VERSION > 7
     case CUDNN_STATUS_RUNTIME_PREREQUISITE_MISSING:
       return "CUDNN_STATUS_RUNTIME_PREREQUISITE_MISSING";
     case CUDNN_STATUS_RUNTIME_FP_OVERFLOW:
       return "CUDNN_STATUS_RUNTIME_FP_OVERFLOW";
     case CUDNN_STATUS_RUNTIME_IN_PROGRESS:
       return "CUDNN_STATUS_RUNTIME_IN_PROGRESS";
 #endif
   }
   return "Unknown cudnn status";
 }


=== 7.4 testar:
  cp -r /usr/src/cudnn_samples_v7/ $HOME
  cd  $HOME/cudnn_samples_v7/mnistCUDNN
  make clean && make
  ./mnistCUDNN


=== 7.5 - instalar outras dependencias
  sudo apt-get install libcupti-dev
  echo 'export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
  sudo apt-get install openjdk-8-jdk
  wget https://github.com/bazelbuild/bazel/releases/download/0.11.1/bazel_0.11.1-linux-x86_64.deb
  sudo dpkg -i bazel_0.11.1-linux-x86_64.deb


=== 8 – Instalar o Python
  sudo apt-get install python-numpy python-dev python-pip python-wheel

=== 9- Instalar o java
  sudo add-apt-repository ppa:webupd8team/java
  sudo apt-get update
  sudo apt-get install oracle-java8-installer
  update-alternatives --display java

=== Edite o arquivo /etc/environment e adicione no final do arquivo
  sudo gedit /etc/environment
  JAVA_HOME=/usr/lib/jvm/java-8-oracle

=== 10)Instalar o eclipse de: 

  https://www.eclipse.org/downloads/eclipse-packages/

=== 10.1) - Descompacte o ecplise 
  cd Downloads/
  sudo mv eclipse-cpp-photon-R-linux-gtk-x86_64.tar.gz  /opt
  cd /opt/
  sudo tar -xvf eclipse-cpp-photon-R-linux-gtk-x86_64.tar.gz 

=== 10.2) - Crie um link para a área de trabalho.
  sudo gedit /usr/share/applications/eclipse.desktop

Coloque o seguinte conteúdo:

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

=== 11) Install OpenCV 3: 

=== 11.2) criar uma pasta e baixar o clone da open cv
 wget https://github.com/opencv/opencv/archive/3.1.0.tar.gz opencv_3.1.0.tar.gz -O opencv_3.1.0.tar.gz
 tar -zxvf opencv_3.1.0.tar.gz
 cd opencv_3.1.0
 
=== 11.3 instalar o pacote 
 sudo mkdir build
 cd build
 cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D  WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D  BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE ..
 sudo make -j8
 sudo make install

***********************

NA compilação pode aparecer o erro de "Hdf5 nÃo encontrado", para resolver, edit o arquivo common.cmake (no diretório ../opencv/modules/python) e , (logo abaixo da linha set(PYTHON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../"), insira as seguintes linhas:

***********************
  find_package(HDF5)
  include_directories(${HDF5_INCLUDE_DIRS})

  recrie a pasta build e compile novamente

=== 12) Faça o download da bullet, da FANN, e do Kvaser SDK

  sudo su
  cd /usr/local/
  wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/bullet/bullet-2.78-r2387.tgz
  wget http://downloads.sourceforge.net/project/fann/fann/2.2.0/FANN-2.2.0-Source.tar.gz

Em seguida, acesse https://www.kvaser.com/downloads-kvaser, no item "Kvaser LINUX Driver and SDK" escolha older versions, e selecione a versão 5.22.392. Salve o arquivo linuxcan.tar.gz no mesmo diretório dos pacotes da bullet e da FANN.

=== 12.1 - Instalação da bullet, FANN, e Kvaser SDK

 tar -xvf bullet-2.78-r2387.tgz
 tar -xvf linuxcan.tar.gz
 tar -xvf FANN-2.2.0-Source.tar.gz

 mv bullet-2.78 bullet 
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

=== 12.2) - Adiciona as linhas abaixo no final do arquivo ~/.bashrc

 # OPENCV
 export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

=== 13) Edite o .bashrc (gedit ~/.bashrc) e coloque no final

 #CARMEN
 export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu/:/usr/lib/libkml
 export CARMEN_HOME=~/carmen_lcad

 #OpenJaus
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARMEN_HOME/sharedlib/OpenJAUS/libopenJaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/libjaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojTorc/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojIARASim/lib

 #MAE
 export MAEHOME=~/MAE
 export PATH=$PATH:$MAEHOME/bin

 #Darknet
 export DARKNET_HOME=$CARMEN_HOME/sharedlib/darknet
 export LD_LIBRARY_PATH=$DARKNET_HOME/lib:$LD_LIBRARY_PATH

 #Cuda
 export PATH=/usr/local/cuda-9.1/bin${PATH:+:${PATH}}
 export LD_LIBRARY_PATH=/usr/local/cuda-9.1/lib64:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
 export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH
 export CUDA_LIBS=/usr/local/cuda/lib64/

=== 14) Intalar a versao 2.11 do flycapture

 Conferir as dependências da versao 2.11
 1) libraw1394-11
 2) libgtkmm-2.4-dev
 3) libglademm-2.4-dev
 4) libgtkglextmm-x11-1.2-dev
 5) libusb-1.0-0 (version 1.0.17 ou maior)

=== 14.1 - Se necessário, instalar as dependências
 sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0

=== 14.2 Depois de instalar as dependências adicionar a linha abaixo no arquivo /etc/modules:
 raw1394 


criar os link:
 sudo ln -s /usr/lib64/libgdk_imlib.so.1.9.15 /usr/lib64/libgdk_imlib.a
 sudo ln -s /usr/src/linux-headers-3.8.0-30/ /usr/src/linux


=== 15) entre para a pasta ubuntu_packages

=== 15.1 - Baixar e instalar a versao 2.11 da FlyCapture

 sudo git clone https://github.com/RhobanDeps/flycapture.git
 cd flycapture
 sudo sh install_flycapture.sh

=== 16) PCL 1.8 
=== 16.1)  instalar dependências PCL
 sudo apt-get install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev  libflann1.8 libboost1.58-all-dev libeigen3-dev libboost-all-dev libflann-dev libvtk5-dev cmake-qt-gui

 sudo apt install libproj-dev

 wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
 tar -xf VTK-7.1.0.tar.gz
 cd VTK-7.1.0 
 mkdir build 
 cd build
 cmake ..
 make 
 sudo make install

Se acorrer o erro *** No rule to make target '/usr/lib/x86_64-linux-gnu/libGL.so', needed by 'lib/libvtkglew-7.1.so.1'. Tente reinstalar a libl1
  sudo apt-get install --reinstall libgl1-mesa-glx



=== 16.2 – Baixar e instalar a PCL1.8
 wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
 tar -xf pcl-1.8.0.tar.gz
 cd pcl-pcl-1.8.0
 mkdir build
 cd build
 cmake ..
 make
 sudo make install

=== 17) KINECT 1.0.20
 sudo apt-get install libudev-dev
 sudo su
 cd /usr/local
 wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.19/libusb-1.0.19.tar.bz2
 tar -xvf libusb-1.0.19.tar.bz2
 cd libusb-1.0.19
 ./configure
 make
 make install

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
 
=== 17.2) - Para testar Execute: 
 freenect-glview

=== 18) Instalação da biblioteca G2O 14.04:
 sudo apt-get install cmake libsuitesparse-dev libqt4-dev qt4-qmake

### O link esta quebrado e a versão atual disponível no git apresenta erro na compilação. Se o link não for restaurado, Utilize a cópia disponível em ubuntu_packet

 sudo cp -r g2o /usr/local/g2o/
 cd /usr/local/g2o/trunk
 sudo mkdir build
 cd build
 sudo make
 sudo make install

Faça o link da boost: 
 sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so /usr/lib/x86_64-linux-gnu/libboost_thread-mt.so

=== 19 - Instalação da dlib: 
 cd /usr/local
 sudo git clone https://github.com/davisking/dlib.git
 cd dlib/
 sudo mkdir build
 cd build/
 sudo cmake ..
 sudo make
 sudo make install

=== 20) Instalação da MAE
=== 20.1- Conferir se as as bibliotecas da MAE estao todas instaladas: 
 sudo apt-get install make g++ freeglut3-dev byacc libforms-dev libtiff5-dev doxygen tcsh flex libdc1394-22-dev

a) Pré-requisitos: as variáveis de ambiente MAEHOME e PATH devem estar ajustadas;
b) Entrar no diretório da MAE: 
  cd $MAEHOME"
c) Compilar a MAE: 
  make
d) Verificar se a biblioteca da MAE libnet_conn.a foi gerado em MAEHOME/lib

e) Verificar se o compilador da MAE netcomp foi gerado em MAEHOME/bin.

=== 21) Instalação da CAFFE
=== 21.1 - instalar bibliotec gflag

 sudo apt-get install -y libgflags-dev libgoogle-glog-dev liblmdb-dev

=== 21.2) Instalar outras dependências da caffe
 sudo apt-get install -y --no-install-recommends \
 build-essential \
 cmake \
 git \
 libgoogle-glog-dev \
 libgtest-dev \
 libiomp-dev \
 libleveldb-dev \
 liblmdb-dev \
 libopencv-dev \
 libopenmpi-dev \
 libsnappy-dev \
 libprotobuf-dev \
 openmpi-bin \
 openmpi-doc \
 protobuf-compiler \
 python-dev \
 python-pip 


=== 22.2) Baixa e instalar o OpenBLAS (https://sourceforge.net/projects/openblas/files/v0.3.0/)
 tar -vzxf OpenBLAS\ 0.3.0\ version.tar.gz 
 cd xianyi-OpenBLAS-939452e/
 make
 sudo make install


=== 22.3) Compilar a Caffe

Obs: o parâmetro  DCUDA_ARCH_BIN="XX" informa a arquitetura utilizada pela GPU

 cd $CARMEN_HOME/sharedlib/ENet/caffe-enet
 mkdir build && cd build
 cmake -DBLAS=open -DCUDA_ARCH_NAME=Manual -DCUDA_ARCH_BIN="50" -DCUDA_ARCH_PTX="" .. $ make all -j 20 && make pycaffe

=== 22.4) Inserir no .bashrc:

 #Caffe ENet
 export CAFFE_ENET_HOME=$CARMEN_HOME/sharedlib/ENet/caffe-enet
 export CAFFE_HOME=$CAFFE_ENET_HOME
 export PYTHONPATH=$CAFFE_ENET_HOME/python
 export LD_LIBRARY_PATH=$CAFFE_ENET_HOME/build/lib:$LD_LIBRARY_PATH

=== 22.5) - Verificar se existe a pasta/arquivo:
  $CAFFE_ENET_HOME/include/caffe/proto/caffe.pb.h. 

se nao existir, siga os passos abaixo.

 cd $CAFFE_ENET_HOME
 protoc src/caffe/proto/caffe.proto --cpp_out=.
 mkdir include/caffe/proto
 mv src/caffe/proto/caffe.pb.h include/caffe/proto/

=== 23) Instalação da libwnn: 
 cd /usr/local
 git clone http://github.com/filipemtz/libwnn
 cd libwnn
 mkdir build
 cd build
 cmake ..
 make -j 8
 sudo make install

 
=== 24) Instalar a  ZED stereo CÃ¢mera: 2.3.3
a)baixe o arquivo .run do endereco
 https://www.stereolabs.com/developers/release/2.3/
b) Mude o atributo do arquivo para que possa ser executado
  chmod  +x ZED_SDK_Linux_Ubuntu16_v2.3.3.run
c) execute o arquivo e siga a instalação padrão 
 ./ZED_SDK_Linux_Ubuntu16_v2.3.3.run

IMPORTANTE: O SDK da ZED só funciona com CUDA 9.1 e não com CUDA 9.0. Tensorflow só funciona com CUDA 9.0. Eu imagino que só algumas funcionalidades devem ser prejudicadas (por exemplo, o estéreo), mas fica o aviso.

=== 25)instalar GSL (trocar car01 pelo usuário da mãquina)
 wget ftp://ftp.gnu.org/gnu/gsl/gsl-2.4.tar.gz
 tar -zxvf gsl-*.*.tar.gz
 cd gsl-2.4
 mkdir /home/car01/gsl
 ./configure --prefix=/home/car01/gsl
 make
 make check
 make install

=== 26) instalar o Carmen 

Observações importantes:

a) A compilação do Carmen procura a biblioteca gdk_imlib, porém no Ubuntu 16 esta biblioteca faz parte do GTK 2.0. Para resolver, comente as linhas do configure do Carmen que fazem menção a esta biblioteca (ver abaixo). 

 #  print "Searching for libgdk_imlib.a...";
 #  if (-e "/usr/lib/libgdk_imlib.a" or 
 #      -e "/usr/local/lib/libgdk_imlib.a" or
 #      -e "/opt/gnome/lib/libgdk_imlib.a") {
 #      print " found\n";
 #  } else {
 #
 #      print " ${red}not found ${normal}\n\n";
 #      print "Could not find libgdk_imlib.a in /usr/lib,\n";
 #      print "/usr/local/lib, nor in /opt/gnome/lib/\n";
 #      print "Please install libgdk_imlib.a or\n${red}re-run $PROGRAM_NAME with --nographics${normal}\n";
 #      die "\n";
 #  }



=== 26.1 - Copiar os arquivo da gsl (trocar car01 pelo usuário da maquina)
 sudo cp -r /home/car01/gsl/include/gsl/ /usr/include/
 sudo cp /home/car01/gsl/lib/libgsl* /usr/lib/
 sudo cp /home/car01/gsl/lib/pkgconfig/gsl.pc /usr/lib/pkgconfig/

=== 26.2 -  fazer os links (trocar car01 pelo usuário da maquina)
 cd $CARMEN_HOME/include/carmen 
 sudo ln -s /home/car01/carmen_lcad/src/camera_boxes_to_world/camera_boxes_to_world.h
 sudo ln -s /home/car01/carmen_lcad/src/camera/camera_messages.h 
 sudo ln -s /home/car01/carmen_lcad/src/camera/camera_interface.h

=== 26.3 – executar o .configure (Se não utilizar gpu use o parâmetro  --nocuda )
 cd $CARMEN_HOME/src
 ./configure --nojava  --nozlib
 Should the C++ tools be installed for CARMEN: [Y/n] Y
 Should Python Bindings be installed: [y/N] N
 Searching for Python2.4... Should the old laser server be used instead of the new one: [y/N] N
 Install path [/usr/local/]: 
 Robot numbers [*]: 1,2

Antes de compilar o CARMEN, compile o modulo tracker.

 cd $CARMEN_HOME/src/tracker
 make

 cd ..
 make

Observações:
a) Se der erro na compilação por falta da biblioteca lippivc (modulos Road_finding, moving_objets..) localize a biblioteca libippicv (find /usr/local/ -name "libippicv*" ) e coloque a pasta onde ela está localizada no path, ou simplesmente copie a mesma para a pasta usr/loca/lib

b) Em função da ordem de compilação do Carmen, pode aparecer erro na compilação de alguns môdulos. Quando isso acontecer, tente entrar na pasta do modulo que apresentou erro, compile o mesmo e tente novamente.

  **************************************
  ****** ((*Ajustes para a IARA*))******
  **************************************
Os demais passos não foram testados!! 

a) Para configurar o OpenJAUS siga o tutorial em :
$CARMEN_HOME/sharedlib/OpenJAUS/README_ALBERTO.txt

b) Para que o GPS e o XSENS sejam configurados automaticamente ao serem conectados às portas USB, copie o seguinte arquivo do diretório data do Carmen para sua máquina:
 cd $CARMEN_HOME/data
 sudo cp 99-usb-serial.rules /etc/udev/rules.d/

c) Ajustes na rede para o GPS Trimble
Para conectar o novo GPS Trimble é necessário uma conexão com a Internet dentro da IARA. Optamos por usar um iPhone com conexão 3G.
Para o iPhone funcionar no Ubuntu 12.04 é necessário um tanto de coisas... Perdemos o histórico mas dá para achar na Internet (Google iPhone 4S ubuntu 12.04 mount). Precisa instalar uns pacotes (apt-get install ...). Se você tiver sucesso, vai ser possível usar o iPhone como Personal Hotspot, ou seja, usar a Internet de dentro da IARA.
Feito isso, é necessário criar um Gateway da máquina que tem acesso a Internet (car01) para uma subrede da IARA (192.168.0.0 - a subrede de Carro Network). Para isso (ver página de referência em https://help.ubuntu.com/community/Internet/ConnectionSharing (Gateway set up)), considerando o iPhone em eth2 e a subrede da IARA em eth1:
 sudo iptables -A FORWARD -o eth2 -i eth1 -s 192.168.0.0/24 -m conntrack --ctstate NEW -j ACCEPT
 sudo iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
 sudo iptables -t nat -F POSTROUTING
 sudo iptables -t nat -A POSTROUTING -o eth2 -j MASQUERADE
 sudo iptables-save | sudo tee /etc/iptables.sav

Os comandos acima criam um NAT do iPhone para a subrede da IARA. Em seguida, é necessário editar o /etc/rc.local e adicionar a linha abaixo antes de "exit 0":
 iptables-restore < /etc/iptables.sav
É necessário ainda:
 sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
Para tornar isso permanente, inclua as linhas abaixo em /etc/sysctl.conf:
 net.ipv4.ip_forward=1
 net.ipv4.conf.default.forwarding=1
 net.ipv4.conf.all.forwarding=1
No Network Manager, tem que setar "Use this connection only for resources on its network" (Network Manager->IPv4 Settings->Routes) em todas as redes cabeadas exceto a do iPhone.
Pronto!
