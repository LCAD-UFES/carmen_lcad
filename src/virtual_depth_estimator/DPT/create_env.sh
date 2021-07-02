# Install virtualenv tool
sudo python3 -m pip install virtualenv

# Create virtualenv and activating
virtualenv -p /usr/bin/python3 venv
source venv/bin/activate

# Installing dependencies for run squeezeseg in virtualenv
pip install -r $CARMEN_HOME/src/virtual_depth_estimator/DPT/requirements.txt

# Deactivate virtualenv
deactivate
