export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/OpenJAUS/libopenJaus/lib:~/OpenJAUS/libjaus/lib:~/OpenJAUS/ojTorc/lib:~/OpenJAUS/ojIARASim/lib
/home/pi/OpenJAUS/ojSteeringBypass/bin/ojSteeringBypass can0 can1 &>> /home/pi/ojSteeringBypass.log 2>&1 &
