-> Para comparar o que o ROS manda com o que o ford_escape recebe salve os prints do ros em um arquivo "ros_odometry.txt", e os do ford_escape em um chamado "results_pid.txt", e faça:

grep ODOMETRY ../../bin/results_pid.txt > results_odometry.txt
grep ROS_ODOMETRY ../../bin/ros_odometry.txt > results_ros_odometry.txt

-> Depois:

gnuplot

-> E dentro do gnuplot:

k = "`head -1 results_ros_odometry.txt | awk '{print $7}'`"

set title "Comparacao de Velocidade e Angulo entre FORDEscape e ROS"
set xlabel "Tempo (s)"
set ylabel "Valores"
set grid
set key left top

plot \
    "results_odometry.txt" using ($7 - k):3 with lines title "Ford v", \
    "results_ros_odometry.txt" using ($7 - k):3 with lines title "ROS v", \
    "results_odometry.txt" using ($7 - k):5 with lines title "Ford phi", \
    "results_ros_odometry.txt" using ($7 - k):5 with lines title "ROS phi"



-> Para plotar o ângulo durante a direção registrado em um log

grep ROBOTVELOCITY_ACK LOG_PATH > results_log.txt

-> Depois:

gnuplot

-> E dentro do gnuplot:

set title "Plot do ângulo"
set xlabel "Linha"
set ylabel "ãngulo"
set grid

plot 'results_log.txt' using 0:3 with lines title 'Valor' lw 2

