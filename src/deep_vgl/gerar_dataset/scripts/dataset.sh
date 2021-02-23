#!/bin/bash
SCRIPT=$(readlink -f "$0")
echo $SCRIPT
SCRIPTPATH=$(dirname "$SCRIPT")
echo $SCRIPTPATH
source $SCRIPTPATH/config.txt

echo $image_path $output_path $base_offset $live_offset

echo "creating basepos and livepos files"
python $SCRIPTPATH/dataset.py -i $image_path -o $output_path -b $base_offset -l $live_offset

echo "creating dataset image/pose lists"
python $SCRIPTPATH/dataset_concat.py -i $image_path -o $output_path -b $base_offset -l $live_offset

train=`ls $output_path/*UFES-TRAIN-LAPS*`
valid=`ls $output_path/*UFES-VALID-LAP*`
test=`ls $output_path/*UFES-TEST-LAP*`

if [[ ! -d $image_path/train/  || ! -f $image_path/train/log.txt ]]; then

    echo "generating darknet image names structure for train"
    $SCRIPTPATH/gera_dataset_darknet.sh $train $image_path/train

    val=`cat $image_path/train/log.txt | wc -l`
    if [ $val -gt 0 ]; then
        echo "remove the image broken links listed on $image_path/train/log.txt"
        exit 0;
    fi
fi

if [[ ! -d $image_path/valid/  || ! -f $image_path/valid/log.txt ]]; then

    echo "generating darknet image names structure for validation"
    $SCRIPTPATH/gera_dataset_darknet.sh $valid $image_path/valid


    val=`cat $image_path/valid/log.txt | wc -l`
    if [ $val -gt 0 ]; then
        echo "remove the image broken links listed on $image_path/valid/log.txt"
        exit 0;
    fi
fi

if [[ ! -d $image_path/test  || ! -f $image_path/test/log.txt ]]; then


    echo "generating darknet image names structure for test"
    $SCRIPTPATH/gera_dataset_darknet.sh $test $image_path/test


    val=`cat $image_path/test/log.txt | wc -l`
    if [ $val -gt 0 ]; then
        echo "remove the image broken links listed on $image_path/test/log.txt"
        exit 0;
    fi
fi



cd $image_path
echo "generating darknet image list for train"
find `pwd`/train -name \*.png > $image_path/ordered.list
shuf $image_path/ordered.list > $image_path/train.list

echo "generating darknet image list for validation"
find `pwd`/valid -name \*.png > $image_path/ordered.list
shuf $image_path/ordered.list > $image_path/valid.list

echo "generating darknet image list for test"
find `pwd`/test -name \*.png > $image_path/ordered.list
shuf $image_path/ordered.list > $image_path/test.list

if [[ ! -d $darknet_path || ! -f $darknet_path/darknet ]]; then
    echo "downloading darknet"
    cd `dirname $darknet_path`
    git clone https://github.com/AlexeyAB/darknet
    cd $darknet_path
    wget https://pjreddie.com/media/files/darknet19_448.conv.23
    echo "adjust the Makefile and compile the project"
fi;

cp $SCRIPTPATH/../darknet_cfg/labels.txt $darknet_path/data/ufes_labels.txt
cp $SCRIPTPATH/../darknet_cfg/ufes.cfg $darknet_path/cfg/ufes.cfg
cp $SCRIPTPATH/../darknet_cfg/ufes_test.cfg $darknet_path/cfg/ufes_test.cfg

# for validation
echo "
classes=651
train  = $image_path/train.list
valid  = $image_path/valid.list
labels = data/ufes_labels.txt
backup = backup/
top=1
" > $darknet_path/cfg/ufesvalid.data

# for test
echo "
classes=651
train  = $image_path/train.list
valid  = $image_path/test.list
labels = data/ufes_labels.txt
backup = backup/
top=1
" > $darknet_path/cfg/ufestest.data

# for sanity check
echo "
classes=651
train  = $image_path/train.list
valid  = $image_path/train.list
labels = data/ufes_labels.txt
backup = backup/
top=1
" > $darknet_path/cfg/ufestrain.data

echo "After compile darknet run the following code:"
echo "./darknet classifier train cfg/ufesvalid.data cfg/ufes.cfg darknet19_448.conv.23 -topk"