#!/usr/bin/env bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

declare -a logs=(`cat logs.txt | awk '{ print $1 }'`)

declare -a dirs=(`cat logs.txt | awk '{ print $2 }'`)

declare -a cams=(`cat logs.txt | awk '{ print $3 }'`)

declare -a crops=(`cat logs.txt | awk '{ print $4 }'`)

declare -a fmts=(`cat logs.txt | awk '{ print $5 }'`)

declare -a imgsize=(`cat logs.txt | awk '{ print $6 }'`)

declare -a ignore_top=(`cat logs.txt | awk '{ print $7 }'`)

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    
    msg="BUMBLEBEE_BASIC_STEREOIMAGE"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE"
    fi
    
    fgrep ${msg}${cams[$i]} ${logs[$i]} > /dados/log2png${i}.txt
    python2.7 $SCRIPTPATH/log2png.py -i /dados/log2png${i}.txt -o ${dirs[$i]} -s ${imgsize[$i]} -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]} -t ${ignore_top[$i]}
 
done

