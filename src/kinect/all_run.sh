#!/bin/bash
../../bin/central &
sleep 2
../../bin/param_daemon --robot p2d8+ ../../src/carmen-kinect.ini &
../../bin/kinect &
sleep 2
../../bin/laser &
../../bin/laserview &
../../bin/pioneer &
../../bin/robot &
../../bin/robotgui &
