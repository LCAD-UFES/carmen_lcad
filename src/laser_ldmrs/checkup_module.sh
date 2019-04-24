#!/bin/bash
blue="\033[0;34m"
red="\033[1;31m"
end="\033[0m" 
binaryflag=0
for arquivo in "skeleton_module_sensor" "skeleton_module_filter"
do
	if ! [ -e $arquivo ]; then
	echo -e "binary $arquivo ${red}not found$end"
	binaryflag=1
	fi
done
if [ $binaryflag = 0 ]; then
	echo -e "${blue}OK - all binaries generated$end"
fi
#make your test script here

