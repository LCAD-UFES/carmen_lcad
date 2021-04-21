#!/bin/bash
SCRIPT=$(readlink -f "$0")
echo $SCRIPT
SCRIPTPATH=$(dirname "$SCRIPT")
echo $SCRIPTPATH
source $SCRIPTPATH/config.txt

echo $image_path $output_path $base_offset $live_offset

echo "image paths to train"
cat $SCRIPTPATH/../logs.txt | awk '{ print $2 }' | awk -F '/' '{ print $NF }' 
cat $SCRIPTPATH/../logs.txt | awk '{ print $2 }' | awk -F '/' '{ print $NF }' > /tmp/input_images_directories.txt

echo "creating basepos and livepos files"
python $SCRIPTPATH/dataset.py -i $image_path -o $output_path -b $base_offset -l $live_offset -I /tmp/input_images_directories.txt

echo "creating dataset image/pose lists"
python $SCRIPTPATH/dataset_concat.py -i $image_path -o $output_path -b $base_offset -l $live_offset -I /tmp/input_images_directories.txt

train=`ls ${output_path}*-TRAIN-*`
valid=`ls ${output_path}*-VALID-*`

if [[ ! -d ${image_path}train/  || ! -f ${image_path}train/log.txt ]]; then

    echo "generating darknet image names structure for train"
    $SCRIPTPATH/gera_dataset_darknet.sh $train ${image_path}train

    val=`cat $image_path/train/log.txt | wc -l`
    if [ $val -gt 0 ]; then
        echo "antes de prosseguir, remova os links quebrados listados em ${image_path}train/log.txt, se existirem."
        rm `cat ${image_path}train/log.txt | awk '{print $1}' | cut -f1 -d':'`
    fi
fi

if [[ ! -d ${image_path}valid/  || ! -f ${image_path}valid/log.txt ]]; then

    echo "generating darknet image names structure for valid"
    $SCRIPTPATH/gera_dataset_darknet.sh $valid ${image_path}valid

    val=`cat $image_path/valid/log.txt | wc -l`
    if [ $val -gt 0 ]; then
        echo "antes de prosseguir, remova os links quebrados listados em ${image_path}valid/log.txt, se existirem."
        rm `cat ${image_path}valid/log.txt | awk '{print $1}' | cut -f1 -d':'`
    fi
fi



cd $image_path
echo "generating darknet image list for train"
ls `pwd`/train/*.png > $image_path/ordered.list
shuf $image_path/ordered.list > $image_path/train.list

echo "generating darknet image list for valid"
ls `pwd`/valid/*.png > $image_path/ordered.list
shuf $image_path/ordered.list > $image_path/valid.list