#!/bin/bash
[ $# -ne 1 ] && echo "Usage: $0 <results_file>" && exit
head -n 10000 $1 | grep -e "Iteration 0, Testing"
tail -n 100   $1 | grep -e ": accuracy" -e "loss =" -e ", Testing"
head -n 100   $1 | grep -e "max_iter" 
