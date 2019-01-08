#!/bin/bash

origpath="lcad@10.9.8.3:~/Desktop/Base_Rodrigo_class"
destpath=/media/rcarneiro/My_Passport/lane_dataset/Base_Rodrigo_classification

dirs="BR_S01  GRI_S01  ROD_S01  ROD_S03  VIX_S02  VIX_S04  VIX_S06  VIX_S08  VIX_S10  VV_S01  VV_S03 \
      BR_S02  GRI_S02  ROD_S02  VIX_S01  VIX_S03  VIX_S05  VIX_S07  VIX_S09  VIX_S11  VV_S02  VV_S04 "

for subdir in $dirs; do
	mkdir -p $destpath/$subdir
	sshpass -p "1q2w3e4r" scp -r $origpath/$subdir/labels  $destpath/$subdir/
done

