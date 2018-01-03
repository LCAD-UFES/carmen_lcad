#!/usr/bin/env bash

: <<'END'
"/dados/log_voltadaufes-20160825.txt" 
"/dados/log_voltadaufes-20160830.txt"
"/dados/log_voltadaufes-20160906-ponte.txt"
"/dados/log_voltadaufes-tracker-20161021.txt"
"/dados/log_voltadaufes-tracker-20161112-estacionamento-1.txt"
"/dados/log_voltadaufes-tracker-20161112-estacionamento.txt"
"/dados/log_ponte-20170220-2.txt"
"/dados/log_guarapari-20170403.txt"
"/dados/log_ponte-20170220.txt"
"/dados/log_ponte-20161228.txt"
"/dados/log_ponte-20170119.txt"
END
declare -a logs=(
"/dados/log_voltadaufes-20160825-01.txt" 
"/dados/log_voltadaufes-20160825-02.txt" 
"/dados/log_volta_da_ufes-20171122.txt" 
)

: <<'END'
"/dados/ufes/20160825"
"/dados/ufes/20160830"
"/dados/ufes/20160906-02"
"/dados/ufes/20161021"
"/dados/ufes/20161112-01"
"/dados/ufes/20161112-00"
"/dados/ufes/20170220-02"
"/dados/ufes/20170403"
"/dados/ufes/20170220"
"/dados/ufes/20161228"
"/dados/ufes/20170119"
END
declare -a dirs=(
"/dados/ufes/20160825-01"
"/dados/ufes/20160825-02"
"/dados/ufes/20171122"
)

: <<'END'
1
1
1
1
1
1
1
1
1
1
1
END
declare -a fmts=(
0
1
1
)

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    
    msg="BUMBLEBEE_BASIC_STEREOIMAGE3"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3"
    fi
    
    #fgrep ${msg} ${logs[$i]} > /dados/log2png${i}.txt
    #python log2png.py -i /dados/log2png${i}.txt -o ${dirs[$i]} -s 640x480 -c 3 -f ${fmts[$i]} -m 380
 
    for left in ${dirs[$i]}/*.l.png; do
        right="${left/l.png/r.png}"
        disp="${left/l.png/d.png}"
        if ! [ -f "$disp" ]; then
            echo "Processing ${left} ${right}"
            ~/deepslam/spsstereo/build/spsstereo $left $right
            mv disparity.png ${disp}
        fi
    done
done

