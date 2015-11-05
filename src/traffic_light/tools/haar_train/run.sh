#!/bin/sh
# 
# File:   run.sh
# Author: tiago
#
# Created on Oct 29, 2013, 3:16:09 PM
#

./training ../train.txt ../gt_train.txt
./testing ../test.txt 0 result.txt
./result result_file.txt ../gt_filtered.txt ../test.txt result.txt