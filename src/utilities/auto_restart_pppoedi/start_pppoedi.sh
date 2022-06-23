#!/bin/bash

LDAP_DI_user="type here your LDAP DI user (get from LAR/UFES)"
LDAP_DI_password="type here your password"

if [[ "$1" = "-h" ]]; then

	echo -e "\nThis script makes sure every minute that PPPoEDI is up and running."
	echo -e "It is intended to run at startup boot on systemd Linux.\n"
	echo -e "Prerequisites:\n"
	echo -e "       Valid LDAP DI username and password (LAR/UFES)"
	echo -e "       Install PPPoEDI package from https://git.inf.ufes.br/LAR-UFES/pppoe-di\n"
	exit
fi

while [ 1 ]
do
	PPP=$(ifconfig | grep UP | grep POINTOPOINT | grep RUNNING)
	if ! [ -n "$PPP" ]; then
		/usr/local/bin/pppoedi-cli ${LDAP_DI_user} ${LDAP_DI_password}
	fi
	sleep 60
done

