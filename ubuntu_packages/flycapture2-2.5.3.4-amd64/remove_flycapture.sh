#!/bin/bash

MY_PROMPT='$ '
MY_YESNO_PROMPT='(y/n)$ '

echo "This is a script to assist with the removal of the FlyCapture2 SDK.";
echo "Would you like to continue and remove all the FlyCapture2 packages?";
echo -n "$MY_YESNO_PROMPT"
read confirm

if [ $confirm != "y" ] && [ $confirm != "Y" ] && [ $confirm != "yes" ] && [ $confirm != "Yes" ]
then
    exit 0
fi

echo

echo "Removing FlyCapture packages...";

sudo dpkg -r updatorgui
sudo dpkg -r flycapture-doc
sudo dpkg -r flycap
sudo dpkg -r libmultisync2
sudo dpkg -r libflycapturegui-c2-dev
sudo dpkg -r libflycapturegui-c2
sudo dpkg -r libflycapture-c2-dev
sudo dpkg -r libflycapture-c2
sudo dpkg -r libflycapturegui2-dev
sudo dpkg -r libflycapturegui2
sudo dpkg -r libflycapture2-dev
sudo dpkg -r libflycapture2

echo "Removing rules file";

if [ -e "/etc/udev/rules.d/40-pgr.rules" ]
then
    sudo rm /etc/udev/rules.d/40-pgr.rules
fi

echo "Complete";

exit 0
