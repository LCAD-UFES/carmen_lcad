# Install virtualenv tool
sudo python2 -m pip install virtualenv

# Create virtualenv and activating
virtualenv -p /usr/bin/python2 squeezeseg_env
source squeezeseg_env/bin/activate

# Installing dependencies for run squeezeseg in virtualenv
pip install -r $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/requirements.txt

# Deactivate virtualenv
deactivate
