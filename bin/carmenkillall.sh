#!/bin/bash
 # Começa com um espaço em cada coluna para que o script não apareça no history do terminal
 HISTFILE=~/.bash_history
 set -o history 
 
 if [ "$#" -ne 1 ]; then
	 echo "Illegal number of parameters"
	 echo "Run ./carmenkillall process-example.ini"
	 exit
 fi
 
 process_name=$1
 
 if [ $1 == "-h" ]; then
	 history | grep ./proccontrol | perl -e 'print reverse <>' | awk '{print $2, $3}' > /tmp/carmenkillall.txt
	 while read line; do
		 check_proc=( $line )
		 if [[ ${check_proc[0]} == "./proccontrol" && ${check_proc[1]} == *".ini" ]]; then
			 echo "Encontrou" $line
			 process_name=${check_proc[1]}
			 echo $process_name
			 break;
		 fi
		 
	 done < /tmp/carmenkillall.txt
 fi
 
 if [[ $process_name == "-h" ]]; then
	 echo "Process not found in history"
	 exit
 fi
 
 echo "Killing proccontrol"
 killall -9 proccontrol
 while read line; do
 if [[ ${line:0:1} != "#" && ${line:0:1} != "" ]]; then
	temp_line=( $line )
	program_extracted=${temp_line[4]}
	if [[ ${temp_line[2]} == 1 && $program_extracted != *"python"* ]]; then
		echo "### " $program_extracted
		echo "Killing process "$program_extracted 
 #		killall -9 ${program_extracted}
		if [[ ${program_extracted:0:2} == "./" ]]; then
	  		killall -9 ${program_extracted:2}
		else
	  		killall -9 $program_extracted
		fi
	fi
 fi
 done < "$process_name"
 
