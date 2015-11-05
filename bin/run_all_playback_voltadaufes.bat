#!/bin/bash
./central &
sleep 1
./param_daemon ../src/carmen-log-voltadaufes.ini &
sleep 1
./playback log_voltadaufes_test.txt &
./playback_control & 
./laserview &
./bumblebee_basic_view 2 &
sleep 2
./bumblebee_basic_view 1 &
./xsens_listener &
./gps-test &

