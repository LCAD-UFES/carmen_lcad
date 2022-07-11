#!/bin/bash

set -a
LDAP_DI_user=""      # type your LDAP DI user within the quotes (get from LAR/UFES)
LDAP_DI_password=""  # type your password within the quotes

if [[ "$1" == "-h" ]]; then

	echo -e "\nThis script makes sure every minute that PPPoEDI is up and running."
	echo -e "It is intended to run at startup boot on systemd Linux.\n"
	echo -e "Prerequisites:\n"
	echo -e "       Valid LDAP DI username and password (LAR/UFES)"
	echo -e "       Install PPPoEDI package from https://git.inf.ufes.br/LAR-UFES/pppoe-di\n"
	exit
fi

if [[ -n "$1" ]]; then
	LDAP_DI_user="$1"
elif [[ -z "$LDAP_DI_user" ]]; then
	read -p "Type your LDAP DI user (get from LAR/UFES): " LDAP_DI_user
fi
if [[ -n "$2" ]]; then
	LDAP_DI_password="$2"
elif [[ -z "$LDAP_DI_password" ]]; then
	read -sp "Type your LDAP DI password: " LDAP_DI_password
fi

function interrupt()
{
	exit
}

trap interrupt SIGINT

while [ 1 ]
do
	PPP=$(ifconfig | grep UP | grep POINTOPOINT | grep RUNNING)
	if [[ -z "$PPP" ]]; then
		# /usr/local/bin/pppoedi-cli ${LDAP_DI_user} ${LDAP_DI_password}
		python3 - << end_python3
###
from pppoediplugin.PppoeDiCli import PppoeDiCli
import sys, os
if __name__ == '__main__':
	for i in range(3 - len(sys.argv)):
		sys.argv.append('')
	sys.argv[1] = os.getenv("LDAP_DI_user")
	sys.argv[2] = os.getenv("LDAP_DI_password")
	pppoe = PppoeDiCli()
end_python3
###
	fi
	sleep 60
done
