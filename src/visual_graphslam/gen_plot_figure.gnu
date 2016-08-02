set term pngcairo
set termoption dash
set output "output.png"
set key bottom right

set xlabel 'metros'
set ylabel 'metros'

set xtics 50
set ytics 50

set xrange[-100:100]
set yrange[-100:100]

plot 'data-log_voltadaufes-20130916-tiago-plot.txt' using 1:2 with lines lt 2 lw 1 lc rgb 'black'  title 'GPS', \
 	'data-log_voltadaufes-20130916-tiago-plot.txt' using 3:4 with lines lt 2 lw 2 lc rgb 'blue'  title 'GraphSLAM', \
 	'data-log_voltadaufes-20130916-tiago-plot.txt' using 5:6 with lines lt 1 lw 1 lc rgb 'black'  title 'Error Interval', \
 	'data-log_voltadaufes-20130916-tiago-plot.txt' using 7:8 with lines lt 1 lw 1 lc rgb 'black'  title ''
