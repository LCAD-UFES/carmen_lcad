cat run_IARA_driver.bat 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/OpenJAUS/libopenJaus/lib:~/OpenJAUS/libjaus/lib:~/OpenJAUS/ojTorc/lib:~/OpenJAUS/ojIARASim/lib
/home/pi/OpenJAUS/ojNodeManager/bin/ojNodeManager /home/pi/OpenJAUS/ojNodeManager/nodeManager_IARA.conf a >> /home/pi/ojNodeManager.log 2>&1 &
sleep 1
/home/pi/OpenJAUS/ojIARASim/bin/ojIARASim can0 can1 &>> /home/pi/ojIARASim.log 2>&1 &

grep jaus ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARMEN_HOME/sharedlib/OpenJAUS/libopenJaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/libjaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojTorc/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojIARASim/lib

subir a interface CAN

sudo ip link set can0 up type can bitrate 500000

bin/ojNodeManager nodeManager_EcoTech4.conf a

bin/ojEcoTech4 can0 can0

bin/ojTorc


sudo apt-get install git wget libncurses-dev flex bison gperf python python-pip python-setuptools python-serial python-click python-cryptography python-future python-pyparsing python-pyelftools cmake ninja-build ccache

git clone --recursive https://github.com/espressif/esp-idf.git

cd ~/esp-idf

./install.sh

. $HOME/esp/esp-idf/export.sh

cd examples/get-started/hello_world

connect the ESP32 via USB

give permission to the USB port (generally it is the /dev/ttyUSB0)

sudo adduser lcad dialout

bin/ojEcoTech4 can0 can0

make

make flash 

make monitor

to stop the program ctrl+]


