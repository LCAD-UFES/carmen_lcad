#!/bin/bash
blue="\033[0;34m"
red="\033[1;31m"
end="\033[0m" 
typeset -l path
path=$1
if [ -e error.out ]; then
	totalwarning=$(cat error.out | grep 'aviso\|warning\|erro\|error' | wc -l)
	if [ $totalwarning = 0 ]; then
		echo -e "${blue}OK - 0 warnings$end"
	else
		echo -e "${red}warning(s): $totalwarning$end"
	fi
else
	echo -e "error.out not found"
fi
if [ -e checkup_module.sh ]; then
	./checkup_module.sh
else
	echo -e "module test not found"
fi
