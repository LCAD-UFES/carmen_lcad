#!/bin/bash

DEFAULT_OUTDIR="."

function display_usage_exit()
{
	if [[ "$2" ]]; then echo -e "\n$2\n"; fi
	echo -e "\nThis script analyzes a CAN log and plots charts of the 4-nibble values of each CAN message type\n"
	echo -e "Usage: $0  <CAN log filename>  <output directory>\n"
	exit "$1"
}

function is_float()
{
	[[ $1 =~ ^[-]?[0-9]+\.?[0-9]*$ ]] || [[ $1 =~ ^[-]?[0-9]*\.?[0-9]+$ ]]
	return $? 
}

function is_positive_float()
{
	if ! is_float "$1"; then return 1; fi
	evaluation=$(bc <<< "$1 > 0" 2>/dev/null)
	test $evaluation = 1
	return $?
}

if [[ $# -eq 0 ]] || [[ "$1" = "-h" ]]; then display_usage_exit 1; fi

if [[ $# -gt 2 ]]; then display_usage_exit 1 "Too many arguments"; fi

LOGFILE="$1"
NEWFILE=$(dirname ${LOGFILE})"/formatted_"$(basename ${LOGFILE})
rm -f ${NEWFILE}

if [ ! -f ${LOGFILE} ]; then display_usage_exit 1 "${LOGFILE}: CAN log file does not exist"; fi

if [[ $# -eq 2 ]]; then OUTDIR="$2"; else OUTDIR=${DEFAULT_OUTDIR}; fi

TOTRECNUM=$(wc -l ${LOGFILE} | cut -d' ' -f 1)

while read LOGREC; do
	RECNUM=$(($RECNUM + 1))
	if [[ $(($RECNUM % 1000)) -eq 0 ]]; then echo "${RECNUM} records done ("$(bc <<< "scale=2; 100 * $RECNUM / $TOTRECNUM")"%)"; fi
	if [[ $(echo ${LOGREC} | tr -s ' ' | cut -d' ' -f 3)  = "" ]]; then echo "Log record #${RECNUM}: Too few data fields: ${LOGREC}"; continue; fi
	if [[ $(echo ${LOGREC} | tr -s ' ' | cut -d' ' -f 4) != "" ]]; then echo "Log record #${RECNUM}: Too many data fields: ${LOGREC}"; continue; fi
	TIMESTAMP=${LOGREC%% *}
	if [[ ${TIMESTAMP} != '('*')' ]] || ! is_positive_float ${TIMESTAMP:1:-1}; then echo "Log record #${RECNUM}: Invalid timestamp format: ${LOGREC}"; continue; fi
	if [[ ${FIRST_TIMESTAMP} = "" ]]; then FIRST_TIMESTAMP=${TIMESTAMP}; fi
	DIFF_TIMESTAMP=$(bc <<< "scale=6; ${TIMESTAMP} - ${FIRST_TIMESTAMP}")
	CANMSG=${LOGREC##* }
	TYPE=${CANMSG%%#*}
	NIBBLES=${CANMSG##*#}
	if [[ ${TYPE} = "" ]] || [[ ${NIBBLES} = "" ]] || [[ ${TYPE}"#"${NIBBLES} != ${CANMSG} ]]; then echo "Log record #${RECNUM}: Invalid CAN message: ${LOGREC}"; continue; fi
	if [[ ${#NIBBLES} -ne 16 ]]; then continue; fi  # Record will be ignored because CAN message does not contain 16 nibbles
	if [[ ${ALLTYPES} = "" ]]; then ALLTYPES=${TYPE}; fi
	if [[ ${ALLTYPES} != *${TYPE}* ]]; then ALLTYPES=${ALLTYPES}" "${TYPE}; fi
	ALLINTa=""; ALLINTb=""
	for N1 in {0..12..2}; do N2=$(($N1 + 2)); ALLINTa=${ALLINTa}$'\t'$(bc <<< "ibase=16; x="${NIBBLES:$N2:2}${NIBBLES:$N1:2}"; if (x > 7FFF) {x - 10000} else x"); done
	for N1 in {0..12..2}; do N2=$(($N1 + 2)); ALLINTb=${ALLINTb}$'\t'$(bc <<< "ibase=16; x="${NIBBLES:$N2:2}${NIBBLES:$N1:2}"; if (x > 7FFF) {8000 -  x} else x"); done
	echo ${DIFF_TIMESTAMP}$'\t'${TYPE}${ALLINTa}${ALLINTb}$'\t'${RECNUM}$'\t'${LOGREC} >> ${NEWFILE}
done < ${LOGFILE}

SORTED=$(dirname ${LOGFILE})"/sorted_"$(basename ${LOGFILE})
sort -k2,2 -k1,1n ${NEWFILE} > ${SORTED}

echo -e "\nCAN message types: ${ALLTYPES}"
echo -e "Sorted log file: ${SORTED}\n"

mkdir -p ${OUTDIR}

for TYPE in ${ALLTYPES}; do
	echo "Plotting CAN messages ${TYPE}..."
	for N in {1..14}; do
		if [[ $N -le 7 ]]; then VAR="a$N"; else VAR="b"$(($N - 7)); fi 
		OUTFILE="${OUTDIR}/"$(basename ${LOGFILE})"_${TYPE}_${VAR}.png"
		gnuplot -e "set terminal png size 800,600; set output '"${OUTFILE}"'; set xlabel 'Time from start (s)'; set ylabel '"${TYPE}" ("${VAR}")'; set grid xtics ytics; \
                            plot '"${SORTED}"' u 1:(stringcolumn(2) eq '"${TYPE}"' ? \$"$(($N + 2))" : 1/0) w l t '"$(basename ${LOGFILE})"'"
	done
done

