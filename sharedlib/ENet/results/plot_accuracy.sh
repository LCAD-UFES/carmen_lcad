#!/bin/bash
USAGE="Usage: $0 <results_file>"
[ $# -ne 1 ] && echo "$USAGE" && exit
! [ -f $1 ] && echo "$1: No such file" && echo "$USAGE" && exit
grep "Train net output #0: accuracy" $1 > accuracy_$1
EXAMPLES=`grep -m 1 -e "examples" $1 | cut -d' ' -f 8`
BATCH=`grep -m 1 -e "batch_size:" $1 | awk '{$1=$1}1' | cut -d' ' -f 2`
EPOCH=$(( $EXAMPLES / $BATCH + 2 * $EXAMPLES / $BATCH % 2 ))
MAX_ITER=`grep -m 1 -e "max_iter:" $1 | cut -d: -f 2 | cut -d# -f 1 | awk '{$1=$1}1'`
DISPLAY_RESULTS=`grep -m 1 -e "display" $1 | cut -d' ' -f 2`
echo "set xlabel \"Batch # (1 batch = $BATCH examples, 1 epoch = $EPOCH batches, max_iter = $MAX_ITER)\" " > accuracy_$1.gp  
echo "set ylabel \"Accuracy per Batch\" " >> accuracy_$1.gp  
echo "set grid xtics ytics" >> accuracy_$1.gp
echo "plot [][0.0:1.0]\"accuracy_$1\" u (\$0 * $DISPLAY_RESULTS):11 w l t \"$1\" " >> accuracy_$1.gp 
echo "pause -1" >> accuracy_$1.gp
echo "Press any key to finish..."
gnuplot accuracy_$1.gp 
