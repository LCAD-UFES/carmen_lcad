#!/bin/sh
P1=`ps -C road_finding,robotgui,navigatorgui,navigator,localize,gps-nmea,xsens,laser,kinect,robot,simulator,param_daemon,central -opid=`
kill -9 $P1
