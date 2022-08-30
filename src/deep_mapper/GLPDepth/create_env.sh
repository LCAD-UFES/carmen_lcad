python3 -m pip install --upgrade pip
# Install virtualenv tool
#python3 -m pip install virtualenv --user

#sudo apt-get install python3-virtualenv 
#Cleaning old venv
#rm -rf venv

# Create virtualenv and activating
virtualenv venv
source venv/bin/activate

# Installing dependencies for run GLP in virtualenv
pip install torch==1.10.1+cu111 torchvision==0.11.2+cu111 -f https://download.pytorch.org/whl/torch_stable.html

pip install -r $CARMEN_HOME/src/deep_mapper/GLPDepth/requirements.txt

# Deactivate virtualenv
deactivate
