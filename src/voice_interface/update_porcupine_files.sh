#!/bin/bash

echo "Initializing update..."

echo "Removing old file..."
rm -rf ~/carmen_lcad/lib/libpv_porcupine.a

echo "Training data.."
~/carmen_packages/Porcupine/tools/optimizer/linux/x86_64/pv_porcupine_optimizer -r ~/carmen_packages/Porcupine/resources/optimizer_data/ -p linux -o . -w "ok e ara" 

echo "Copying files..."
cp ~/carmen_packages/Porcupine/ok\ e\ ara_linux.ppn $CARMEN_HOME/data/voice_interface_data/hotword_oi_iara.ppn
cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
cp ~/carmen_packages/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
cp ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a

echo "Linking file..."
ln -s ~/carmen_packages/Porcupine/libpv_porcupine.a $CARMEN_HOME/lib/libpv_porcupine.a

echo "Porcupine updated!"
