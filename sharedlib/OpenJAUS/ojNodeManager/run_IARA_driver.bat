#!/bin/bash

ip=192.168.25.59
port=3794
data_received=`nping --tcp -c 1 -p $port $ip | grep RCVD`

while [ -z "$data_received" ]; do
	echo "vazio"
	data_received=`nping --tcp -c 1 -p $port $ip | grep RCVD`
done
echo "valor: " $data_received

