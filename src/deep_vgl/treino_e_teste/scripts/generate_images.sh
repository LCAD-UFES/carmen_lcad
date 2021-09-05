#!/usr/bin/env bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
LOGTXT=$1
if [ "$LOGTXT" == "" ]; then
    echo "missing argument"
    echo $LOGTXT
    echo "usage: $0 $SCRIPTPATH/../logs.txt"
    exit 0;
fi

declare -a logs=(`cat $LOGTXT | awk '{ print $1 }'`)

declare -a dirs=(`cat $LOGTXT | awk '{ print $2 }'`)

declare -a cams=(`cat $LOGTXT | awk '{ print $3 }'`)

declare -a crops=(`cat $LOGTXT | awk '{ print $4 }'`)

declare -a fmts=(`cat $LOGTXT | awk '{ print $5 }'`)

declare -a imgsize=(`cat $LOGTXT | awk '{ print $6 }'`)

declare -a ignore_top=(`cat $LOGTXT | awk '{ print $7 }'`)

echo ${fmts[@]}

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    cam_or_lidar=${cams[$i]}
    msg="BUMBLEBEE_BASIC_STEREOIMAGE"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE"
    elif [ ${fmts[$i]} -eq 2 ]; then
        msg="CAMERA"
    elif [ ${fmts[$i]} -eq "3" ]; then
        msg="VELODYNE_PARTIAL_SCAN_IN_FILE"
        cam_or_lidar=""
    elif [ ${fmts[$i]} -eq "4" ]; then
        echo "Esse vai demorar... é log antigo"
        msg="VELODYNE_PARTIAL_SCAN"
        cam_or_lidar=""
    fi
    
    if [ -z "$2" ]; then
        fgrep ${msg}${cam_or_lidar} ${logs[$i]} > /dados/log2png${i}.txt
    fi
    
    if [ ${fmts[$i]} -gt 2 ]; then
       echo "python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -f ${fmts[$i]} -al ${crops[$i]} -ar ${ignore_top[$i]}"
       #python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -f ${fmts[$i]} -al ${crops[$i]} -ar ${ignore_top[$i]}
    else
       echo "python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]} -t ${ignore_top[$i]}"
       #python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]} -t ${ignore_top[$i]}
    fi
done

