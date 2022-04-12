# Install virtualenv tool
sudo python3.7 -m pip install virtualenv

#Cleaning old venv
rm -rf venv/

# Create virtualenv and activating
virtualenv -p /usr/bin/python3.7 venv
source venv/bin/activate

#pip3 install scikit-build
#pip3 install numpy
# Installing dependencies for run squeezeseg in virtualenv
pip3 install -r $CARMEN_HOME/src/deep_mapper/GLPDepth/requirements.txt

# Deactivate virtualenv
deactivate
