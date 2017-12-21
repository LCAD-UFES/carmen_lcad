#!/bin/bash
USAGE="Usage: $0 <results_file>"
[ $# -ne 1 ] && echo "$USAGE" && exit
! [ -f $1 ] && echo "$1: No such file" && echo "$USAGE" && exit
head -n 10000 $1 | grep -e "Iteration 0, Testing"
tail -n 100   $1 | grep -e ": accuracy" -e "loss =" -e ", Testing"
tail -n 10    $1 | grep -e "lr ="
head -n 100   $1 | grep -e "max_iter" 
