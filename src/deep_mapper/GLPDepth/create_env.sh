# Install virtualenv tool
sudo python3.7 -m pip install virtualenv

# Create virtualenv and activating
virtualenv -p /usr/bin/python3.7 venv
source venv/bin/activate

# Installing dependencies for run squeezeseg in virtualenv
pip3 install -r $CARMEN_HOME/src/deep_mapper/GLPDepth/requirements.txt

# Deactivate virtualenv
deactivate
