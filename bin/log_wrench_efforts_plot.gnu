!cat log_wrench_efforts.txt | grep ODOM_ACK > odom.txt
!cat log_wrench_efforts.txt | grep BASEMOTION_ACK > comm.txt

set xlabel "secs"
set ylabel "rad"

set yrange [-1:1]
set y2range [-1:1]
set xrange [0:35]
#set ytics 1 tc lt 1
#set y2tics 0.1 tc lt 2

plot "comm.txt" using 55:4 with lines axes x1y1
replot "odom.txt" using 10:6 with lines axes x1y2

#set term png enhanced
#set output "log_wrench_efforts.png"
#replot

pause -1
