#!/bin/bash
./central &
sleep 1
./param_daemon ../src/carmen-bumblebee.ini &
sleep 1
./playback log20110518.log.gz &
./playback_control & 
./bumblebee_basic_view 1 &
./bumblebee_basic_view 2 &

