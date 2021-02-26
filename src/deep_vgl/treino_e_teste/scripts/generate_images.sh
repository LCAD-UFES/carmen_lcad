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

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    
    msg="BUMBLEBEE_BASIC_STEREOIMAGE"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE"
    elif [ ${fmts[$i]} -eq 2 ]; then
        msg="CAMERA"
    fi
    
    fgrep ${msg}${cams[$i]} ${logs[$i]} > /dados/log2png${i}.txt
#    echo "python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]} -t ${ignore_top[$i]}"
    python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -g ${logs[$i]} -o ${dirs[$i]} -s ${imgsize[$i]} -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]} -t ${ignore_top[$i]}
done

