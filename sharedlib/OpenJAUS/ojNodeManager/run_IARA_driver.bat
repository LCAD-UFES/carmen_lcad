export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/OpenJAUS/libopenJaus/lib:~/OpenJAUS/libjaus/lib:~/OpenJAUS/ojTorc/lib
/home/pi/OpenJAUS/ojNodeManager/bin/ojNodeManager /home/pi/OpenJAUS/ojNodeManager/nodeManager_IARA.conf a >> /home/pi/ojNodeManager.log 2>&1 &
/home/pi/OpenJAUS/ojIARASim/bin/ojIARASim can0 can1 &>> /home/pi/ojIARASim.log 2>&1

