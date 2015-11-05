#!/bin/bash
rm -f log_voltadaufes_noite2.gz
./central &
./logger log_voltadaufes_noite2.gz &
sleep 1
./param_daemon ../src/carmen-log-voltadaufes.ini &
sleep 1
./laser &
./laserview &
./bumblebee_basic 2 &
./bumblebee_basic_view 2 &
sleep 1
./bumblebee_basic 1 &
./bumblebee_basic_view 1 &
./xsens &
sleep 1
./xsens_listener &
./gps-nmea & 
./gps-test &
#./kinect &
#./kinect_view &
