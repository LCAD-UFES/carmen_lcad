#!/usr/bin/env bash

#"/dados/log_voltadaufes-20160721.txt"
declare -a logs=(
"/dados/log_voltadaufes-20160721_ambiental_1.txt"
"/dados/log_voltadaufes-20160721_ambiental.txt"
"/dados/log_voltadaufes-20160825.txt" 
"/dados/log_voltadaufes-20160825-02.txt"
"/dados/log_voltadaufes-20160830.txt"
"/dados/log_voltadaufes-20160906-ponte.txt"
"/dados/log_voltadaufes-tracker-20161021.txt"
"/dados/log_voltadaufes-tracker-20161112-estacionamento-1.txt"
"/dados/log_voltadaufes-tracker-20161112-estacionamento.txt"
"/dados/log_ponte-20170220-2.txt"
"/dados/log_guarapari-20170403-2.txt"
"/dados/log_guarapari-20170403.txt"
"/dados/log_volta_da_ufes-20170926.txt"
)

#"/dados/ufes/20160721"
declare -a dirs=(
"/dados/ufes/20160721-01"
"/dados/ufes/20160721-00"
"/dados/ufes/20160825"
"/dados/ufes/20160825-02" 
"/dados/ufes/20160830"
"/dados/ufes/20160906-02"
"/dados/ufes/20161021"
"/dados/ufes/20161112-01"
"/dados/ufes/20161112-00"
"/dados/ufes/20170220-02"
"/dados/ufes/20170403-02"
"/dados/ufes/20170403"
"/dados/ufes/20170926"
)

#8
declare -a cams=(
8
8
3
3
3
3
3
3
3
3
3
3
3
)

#"640x310"
declare -a crops=(
"640x310"
"640x310"
"640x380"
"640x380" 
"640x380"
"640x380"
"640x380"
"640x380"
"640x380"
"640x380"
"640x380"
"640x380"
"640x380"
)

for i in "${!dirs[@]}"; do
    #mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
 
    #fgrep BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE${cams[$i]} ${logs[$i]} > /tmp/log2png.txt
    #python log2png.py -i /tmp/log2png.txt -o ${dirs[$i]} -s 640x480 -c ${cams[$i]}
 
    rm -f ${dirs[$i]}/*crop.png
    find ${dirs[$i]} -name '*.png' | parallel 'convert {.}.png -set colorspace RGB -gravity north -crop ${crops[$i]}+0+0 {.}.crop.png'
    
    #~/sps-stereo/build/spsstereo ${dirs[$i]}
done

