#!/usr/bin/env bash

declare -a logs=("/dados/log_voltadaufes-20160721.txt"
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
"/dados/log_ponte-20170220.txt"
"/dados/log_ponte-20161228.txt"
"/dados/log_volta_da_ufes_localizer-20170303-2.txt"
"/dados/log_volta_da_ufes_localizer-20170303.txt")

declare -a dirs=("/dados/ufes/20160721"
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
"/dados/ufes/20170220"
"/dados/ufes/20161228"
"/dados/ufes/20170303-02"
"/dados/ufes/20170303")

declare -a cams=(8 8 8
3 3 3 3 3 3 3 3 3 3 3
3 3 3 3)

declare -a crops=(310 310 310
380 380 380 380 380 380 380 380 380 380 380
380 380 380 380)

declare -a fmts=(1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1)

for i in "${!dirs[@]}"; do
    mkdir -p ${dirs[$i]}
    echo "Saving at ${dirs[$i]}"
    
    msg="BUMBLEBEE_BASIC_STEREOIMAGE"
    if [ ${fmts[$i]} -eq 1 ]; then
        msg="BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE"
    fi
    
    echo "fgrep ${msg}${cams[$i]} ${logs[$i]} > /dados/log2png${i}.txt"
    python log2png.py -i /dados/log2png${i}.txt -o ${dirs[$i]} -s 640x480 -c ${cams[$i]} -f ${fmts[$i]} -m ${crops[$i]}
 
    #~/sps-stereo/build/spsstereo ${dirs[$i]}
done

