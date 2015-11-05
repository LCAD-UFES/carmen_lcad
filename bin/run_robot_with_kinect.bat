#!/bin/bash
#rm log20110525-2.log
#./logger log20110525-2.log &
./central &
sleep 1
./param_daemon --robot p2d8+ ../src/carmen-kinect.ini &
sleep 1
./kinect &
./laser &
./pioneer &
./robot &
#./localize &
#./navigator &
#./navigatorgui &
./robotgui &
#./kinect_view &
