#!/bin/bash

echo "Initializing update..."

echo "Removing old file..."
rm -rf $CARMEN_HOME/lib/libpv_porcupine.a

echo "Training data.."
$CARMEN_PACKAGES/Porcupine/tools/optimizer/linux/x86_64/pv_porcupine_optimizer -r $CARMEN_PACKAGES/Porcupine/resources/optimizer_data/ -p linux -o $CARMEN_PACKAGES/Porcupine/ -w "ok e ara" 

echo "Copying files..."
cp $CARMEN_PACKAGES/Porcupine/ok\ e\ ara_linux.ppn $CARMEN_HOME/data/voice_interface_data/hotword_oi_iara.ppn
cp $CARMEN_PACKAGES/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
cp $CARMEN_PACKAGES/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
cp $CARMEN_PACKAGES/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a

echo "Linking file..."
ln -s $CARMEN_PACKAGES/Porcupine/libpv_porcupine.a $CARMEN_HOME/lib/libpv_porcupine.a

echo "Porcupine updated!"
