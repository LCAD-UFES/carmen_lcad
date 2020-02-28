# Install virtualenv tool
sudo python3 -m pip install virtualenv

# Create virtualenv and activating
virtualenv -p /usr/bin/python3 inplace_env
source inplace_env/bin/activate

# Installing dependencies for run squeezeseg in virtualenv
pip install -r $CARMEN_HOME/sharedlib/inplace_abn/requirements.txt

# Deactivate virtualenv
deactivate
