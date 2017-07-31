#!/bin/bash

# Script to replace ".00." for "." in filenames
# Before using the script you must grant it execution rights
# chmod +x rename.sh
# usage:
# ./rename.sh <path_to_dir>

path=`pwd`
if [ ! -z "$1" ]
then
	path=$1
fi

for file in $path/i*; do
	if [ "$file" != "${file/.00./.}" ]
	then
		#echo "Renaming " $file "to " "${file/.00./.}"
		mv "$file" "${file/.00./.}"
	fi
done
