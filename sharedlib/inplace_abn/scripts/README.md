
For working from any place, go to your .bashrc and insert the line at the end of file:

```
# InplaceABN
export PYTHONPATH=$CARMEN_HOME/sharedlib/inplace_abn/scripts:$PYTHONPATH

# Installing the toolkit virtualenv
sudo python2 -m pip install virtualenv

# Creating virtualenv and activating
virtualenv -p /usr/bin/python3 inplace_env

source $CARMEN_HOME/sharedlib/inplace_abn/inplace_env/bin/activate

# Install dependencies
pip install -r $CARMEN_HOME/sharedlib/inplace_abn/scripts/requirements.txt

# Deactivate virtualenv
deactivate
