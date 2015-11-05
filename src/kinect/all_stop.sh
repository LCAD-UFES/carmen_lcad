#!/bin/sh
P1=`ps -C robotgui,robot,pioneer,laserview,laser,kinect,param_daemon,central -opid=`
kill -INT $P1
