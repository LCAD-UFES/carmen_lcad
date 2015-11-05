#!/bin/sh
P1=`ps -C robotgui,navigatorgui,navigator,localize,gps-nmea,xsens,robot,laser,kinect,pioneer,pioneerackerman,param_daemon,central,logger -opid=`
kill -INT $P1
