#!/bin/bash
./central &
sleep 1
./param_daemon  --robot p2d8+ ../data/mapa_do_ctvii.map ../src/carmen-pioneer-ackerman.ini &
sleep 1
./simulator_ackerman &
./robot_ackerman &
./robot_ackerman_gui &
./localize_ackerman &
./navigator_ackerman &
./navigator_ackerman_gui &
