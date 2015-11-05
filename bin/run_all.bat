#!/bin/bash
./central &
rm log.txt
./logger log.txt &
sleep 1
./param_daemon  --robot p2d8+ ../data/longwood.map &
sleep 1
./simulator &
./robot &
./localize &
#./navigator &
#./navigatorgui &
./robotgui &
