#!/bin/bash

display_usage_exit()
{
	echo "This script does move and/or rename CARMEN LCAD log files, keeping internal integrity."
	echo "Usage:"
	echo "$0 <old_file> <new_name>"
	echo "$0 <old_file> <target_dir>"
	echo "$0 <old_file> <target_dir>/<new_name>"
	[ "$2" ] && echo $2
	exit $1
}

[ "$1" = "-h" ] && display_usage_exit 1
[ $# -lt 2 ] && display_usage_exit 1 "$*: Insufficient number of arguments."
[ $# -gt 2 ] && display_usage_exit 1 "$*: Excessive number of arguments."

[ ! -f $1 ] && display_usage_exit 1 "$1: Old log file does not exist."
DIR1=`dirname "$1"`
DIR1=`readlink -f "$DIR1"`
FILE1=`basename "$1"`
FULLPATH1=$DIR1/$FILE1

[[ $2 = *:* ]] &&  display_usage_exit 1 "$2: New log filename cannot contain ':' character."
[ -f $2 ] && display_usage_exit 1 "$2: New log filename already exists."
DIR2=`dirname "$2"`
DIR2=`readlink -f "$DIR2"`
FILE2=`basename "$2"`
if [ -d $2 ] || [[ $2 = */ ]]; then
	DIR2=$DIR2/$FILE2
	FILE2=$FILE1
fi
FULLPATH2=$DIR2/$FILE2

[ "$FULLPATH2" = "$FULLPATH1" ] && display_usage_exit 1 "$*: Target directory and filename are equal to the current."	
[ ! -d $DIR2 ] && display_usage_exit 1 "$DIR2: Target directory does not exist."

[ -f "$FULLPATH1".index ] && INDEX_FILE="$FULLPATH1".index || INDEX_FILE=""
[ -d "$FULLPATH1"_velodyne ] && VELODYNE_DIR="$FULLPATH1"_velodyne || VELODYNE_DIR=""
[ -d "$FULLPATH1"_bumblebee ] && BUMBLEBEE_DIR="$FULLPATH1"_bumblebee || BUMBLEBEE_DIR=""

[ "$INDEX_FILE" ] && [ -e "$FULLPATH2".index ] && display_usage_exit 1 "$2: New log index filename already exists."
[ "$VELODYNE_DIR" ] && [ -e "$FULLPATH2"_velodyne ] && display_usage_exit 1 "$2: New velodyne directory name already exists."
[ "$BUMBLEBEE_DIR" ] && [ -e "$FULLPATH2"_bumblebee ] && display_usage_exit 1 "$2: New bumblebee directory name already exists."

[ "$DIR1" != "$DIR2" ] && echo "Moving log file from $DIR1 to $DIR2..."
[ "$FILE1" != "$FILE2" ] && echo "Renaming log file $FILE1 to $FILE2..."

mv -n $FULLPATH1 $FULLPATH2
[ $? -ne 0 ] && exit 1

echo "Replacing internal references in log file $2..."
sed -i "s:$FULLPATH1:$FULLPATH2:g" $FULLPATH2

if [ "$INDEX_FILE" ]; then
	echo "Renaming log index file to $2.index..."
	mv -n $INDEX_FILE "$FULLPATH2".index
else
	echo "No log index file found."
fi

if [ "$VELODYNE_DIR" ]; then
	echo "Renaming velodyne directory to $2_velodyne..."
	mv -n $VELODYNE_DIR "$FULLPATH2"_velodyne
else
	echo "No velodyne directory found."
fi

if [ "$BUMBLEBEE_DIR" ]; then
	echo "Renaming bumblebee directory to $2_bumblebee..."
	mv -n $BUMBLEBEE_DIR "$FULLPATH2"_bumblebee
else
	echo "No bumblebee directory found."
fi

