#!/bin/bash
#Para usar: ./audit_remove_empty /dados/pasta_audit/

audit_path=$1

#Condições de segurança
if [ "$audit_path" == "/" ] ; then
	echo "Por segurança, o programa bloqueou seu uso na pasta '/'."
	exit
fi

if [ "$audit_path" == "/dados" ] || [ "$audit_path" == "/dados/" ] ; then
	echo "Por segurança, o programa bloqueou seu uso na pasta '/dados/'."
	exit
fi
#########

if [ "${audit_path: -1}" != "/" ] ; then
	audit_path+="/"
fi

echo $audit_path

find "$audit_path" -type d -empty -delete
