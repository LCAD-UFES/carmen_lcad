****************************************************************
* Module  : MULTICENTRAL_BRIDGE
* Comment : CARMEN MULTICENTRAL_BRIDGE module to conect centrals in diferent robots
****************************************************************

This module is a bridge to comunicate two centrals  (or more, we didn't test more than two).
It subscribes to messages in central_1 and publish to central_2 (and vice versa).
The central can be in the same computer in diferent ports, or in different computers.
To choose the centrals use the parameter: -central centrals_list.txt
(eg. ./multibridge_traffic_light -central centrals_list.txt)

OBS: The name of centrals in central_list, should be a string, don't use numbers. If you need to refer a central's IP adress,
edit the hosts file (/etc/hosts) and give a name to the IP adress of the other central. So use this name in central_list.txt


To know more about multicentral mode, see: IPC Manual, Cap 7, pg. 27.($CARMEN_HOME/doc/IPC_Manual.pdf)
