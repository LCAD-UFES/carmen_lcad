#!/usr/bin/env bash

: <<'END'
"/dados/log_voltadaufes-20140418.txt"
END
declare -a logs=(
"/dados/log_voltadaufes-20160902.txt"
"/dados/log_voltadaufes-20160906.txt"
)

: <<'END'
"/dados/ufes/20140418"
END
declare -a dirs=(
"/dados/ufes/20160902"
"/dados/ufes/20160906-01"
)

: <<'END'
8
END
declare -a cams=(
8
8
)

: <<'END'
364
END
declare -a crops=(
350
350
)

: <<'END'
0
END
declare -a fmts=(
0
0
)

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    
    msg="BUMBLEBEE_BASIC_STEREOIMAGE"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE"
    fi
    
    fgrep ${msg}${cams[$i]} ${logs[$i]} > /dados/log2png${i}.txt
    python log2png.py -i /dados/log2png${i}.txt -o ${dirs[$i]} -s 640x480 -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]}
 
    for left in ${dirs[$i]}/*.l.png; do
        right="${left/l.png/r.png}"
        disp="${left/l.png/d.png}"
        if ! [ -f "$disp" ]; then
            echo "processing ${left} ${right}"
            #~/deepslam/spsstereo/build/spsstereo $left $right
            #mv disparity.png ${disp}
        fi
    done
done

