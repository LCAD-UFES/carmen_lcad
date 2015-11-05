#!/bin/bash
#./logger log20110520.log.gz &
./central &
sleep 1
./param_daemon --robot p2d8+ ../src/carmen-pioneer-ackerman.ini &
sleep 1
./pioneerackerman &
#./kinect &
#./laser &
./robot &
#./localize &
#./navigator &
#./navigatorgui &
./robotgui &
