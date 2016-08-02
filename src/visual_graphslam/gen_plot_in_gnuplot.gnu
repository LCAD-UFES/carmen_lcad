set termoption dash
set key top right
set xlabel 'metros'
set ylabel 'metros'
set xtics 10
set ytics 10
#set xrange[-150:150]
#set yrange[-150:130]
plot 'data-log_voltadaufes-20131015-filipe-gnuplot.txt' using 1:2 with lines lt 2 lw 1 lc rgb 'red'  title 'GPS', 'data-log_voltadaufes-20131015-filipe-gnuplot.txt' using 3:4 with lines lt 2 lw 2 lc rgb 'blue'  title 'GraphSLAM', 'data-log_voltadaufes-20131015-filipe-gnuplot.txt' using 5:6 with lines lt 1 lw 1 lc rgb 'black'  title 'Erro do GPS', 'data-log_voltadaufes-20131015-filipe-gnuplot.txt' using 7:8 with lines lt 1 lw 1 lc rgb 'black'  title ''
