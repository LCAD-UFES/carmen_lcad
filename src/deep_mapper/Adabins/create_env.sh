#!/bin/bash

echo "preparing virtual environment - python"
pip3 install virtualenv

echo "make venv"
virtualenv venv

echo "activating venv"
source venv/bin/activate

echo "installing pip dependencies on venv..."
pip3 install torch==1.10.1+cu111 torchvision==0.11.2+cu111 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install pytorch3d
pip3 install pillow 
pip3 install wandb
pip3 install tqdm
pip3 install matplotlib 
pip3 install scikit-{learn,image} 
pip3 install opencv-python==4.2.0.32

deactivate

