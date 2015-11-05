#!/bin/bash

# Copyright (c) 2007 Glynn Tucker Consulting Engineers
# License: Dual-license under GPL/BSD license
# 	GPL (v2 or any later version): http://www.fsf.org/licensing/licenses/gpl.txt
#	MIT License: http://www.opensource.org/licenses/mit-license.html

# Define our functions
setup_ssh_certificates()
{
	# parameters: 
	#	1: ip/hostname of the mirror
	#	2: username to login with
	#	3: password
	# purpose: 	Set up passwordless login between this host and the mirror
       #		specified using ssh public key authentication
	MIRROR_IP=$1
	MIRROR_USER=$2
	MIRROR_PASSWORD=$3

	# Verify we can login
	CAN_SSH="`ssh -o PasswordAuthentication=no $MIRROR_IP exit > /dev/null 2>&1; echo $?`"

	if [ "$CAN_SSH" -eq 255 ]; then
		# SSH server is listening, we can login and do our work
		echo -n Attempting to setup passwordless login via SSH keys...

		if [ -f ~/.ssh/id_rsa -o -f ~/.ssh/id_rsa.pub ]; then
			echo
			echo "Using pre-existing keys at ~/.ssh/id_rsa*"
		else
			if ssh-keygen -t rsa -f ~/.ssh/id_rsa -N ""; then
				echo Sorry, failed to create public/private key pair
				exit 1
			fi
		fi
	  echo -n Finish id_dsa check\n

		# Log in to mirror and install keys
		# This requires some nasty tricks because ssh
		# won't let us supply a password
		# Quick steps:
		# 1. Set DISPLAY to something
		# 2. Generate a script that outputs the password
		# 3. Set SSH_ASKPASS to the name of that script
		# 4. Use setsid to disassociate ssh from this script's tty

		ASKPASS_SCRIPT_NAME=/tmp/thisisarandomscriptname.sh
		# We certainly don't want our password hanging around if the script fails
		#sleep 10 && rm $ASKPASS_SCRIPT_NAME 2>/dev/null &
		
		if [ "$DISPLAY" == "" ]; then
			DISPLAY=localhost:0.0
		fi
		cd
		expect -c "spawn scp .ssh/id_rsa.pub $MIRROR_USER@$MIRROR_IP:~/`hostname`_id.pub;expect *password*;send -- $MIRROR_PASSWORD\r;interact"
		sleep 1
		expect -c "spawn ssh $MIRROR_USER@$MIRROR_IP cat ~/`hostname`_id.pub >> .ssh/authorized_keys;expect *password*;send -- $MIRROR_PASSWORD\r;interact"
		cd -
	fi

	echo -n Testing remote login ability...
	if ssh -o PasswordAuthentication=no $MIRROR_USER@$MIRROR_IP exit; then
		echo OK
	else
		echo Failed!
		echo Sorry, I was unable to set up the automatic logins you requested
		exit 5
	fi
}

# Ensure we have a network
echo -n Checking for network interface...
if sudo ifconfig | grep -q eth[0-9]; then
	echo OK
else
	echo Failed!
	echo You need to be connected to the network to run this script.
	echo This script assumes your network interface is on eth[0-9]
	exit 2
fi

# Ask for password of the mirror
echo -n "What is the password for the remote host?: "
read -s MIRRORHOST_PASSWORD
echo

for HOST in bx1 bx2 bx3 bx4 bx5 bx6 bx7 bx8
do

	# Ask for IP of the mirror
	echo -n "=============Setup $HOST=========== : "
	if [ "$HOST" != "" ]; then
		MIRROR_IP=$HOST
	fi

	# Check for existence of this mirror
	echo -n Attempting to ping the mirror at $MIRROR_IP...
	if ping -c 1 -w 5 -n $MIRROR_IP > /dev/null 2>&1; then
		echo OK
	else
		echo Failed!
		echo The mirror you specified can\'t be pinged on the network.
		echo Please ensure you have a linux installation at $MIRROR_IP then run this script again.
		exit 3
	fi

	# Ask for username of the mirror
	MIRRORHOST_USER=$HOST
	echo


	# Create public key certificates
	setup_ssh_certificates $MIRROR_IP $MIRRORHOST_USER $MIRRORHOST_PASSWORD
done
