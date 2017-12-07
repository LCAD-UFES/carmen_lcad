#!/bin/bash

cnt=0

while true
do
#	while [ $cnt -lt 2047 ]
	while [ $cnt -lt 32 ]
	do
		msg=$(printf '%03x\n' $cnt)
		msg="114#0"$msg"00000000"
		cansend can0 $msg
		echo "cansend can0 "$msg
		(( cnt+=4 ))
		sleep 1
	done
#	while [ $cnt -gt 0 ]
	while [ $cnt -gt 4 ]
	do
		msg=$(printf '%03x\n' $cnt)
		msg="114#0"$msg"00000000"
		echo "cansend can0 "$msg
		cansend can0 $msg
		(( cnt-=4 ))
		sleep 1
	done
done
