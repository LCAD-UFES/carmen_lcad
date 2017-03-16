#!/bin/bash 
cangw -F
#cangw -A -s can1 -d can0 -f 80:1FFFFFFF
#cangw -A -s can1 -d can0 -f 82:1FFFFFFF
cangw -A -s can1 -d can0 -f 0:0
#cangw -A -s can0 -d can1 -f 500:1FFFFFFF
cangw -A -s can0 -d can1 -f 425~1FFFFFFF
cangw -A -s can0 -d can1 -f 425:1FFFFFFF -m AND:ILD:425.8.00FFFFFFFFFFFFFF

