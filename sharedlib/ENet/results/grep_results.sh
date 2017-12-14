#!/bin/bash
[ $# -ne 1 ] && echo "Usage: $0 <results_file>" && exit
grep "Train net output #0: accuracy" $1 > accuracy_$1
echo "set xlabel \"Batch # (1 batch = 100 samples, 1 epoch = 3360 batches)\" " > accuracy_$1.gp  
echo "set ylabel \"Overall Accuracy\" " >> accuracy_$1.gp  
echo "set grid xtics ytics" >> accuracy_$1.gp
echo "plot [][0.0:1.0]\"accuracy_$1\" u (\$0 * 20):11 w l t \"$1\" " >> accuracy_$1.gp 
echo "pause -1" >> accuracy_$1.gp
echo "Press any key to finish..."
gnuplot accuracy_$1.gp 
