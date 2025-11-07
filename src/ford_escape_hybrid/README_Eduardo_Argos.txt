-> Para comparar o que o ROS manda com o que o ford_escape recebe salve os prints do ros em um arquivo "ros_odometry.txt", e os do ford_escape em um chamado "results_pid.txt", e faça:

Para velocidade:

```
grep VEL ../../bin/results_pid.txt > results_pid-velocity.txt
grep ROS_ODOMETRY ../../bin/ros_odometry.txt > results_odom_ros.txt
gnuplot
```

```
grep STE ../../bin/results_pid.txt > results_pid-steering.txt
grep ROS_ODOMETRY ../../bin/ros_odometry.txt > results_odom_ros.txt
gnuplot
```

-> Agora no gnuplot:

Para velocidade X velocidade bruta X velocidade filtrada:
```
k = "`head -1 ./results_odom_ros.txt | awk '{print $32}'`"
plot "./results_pid-velocity.txt" using ($20-k):13 with lines title "cvFord", "./results_pid-velocity.txt" using ($20-k):14 with lines title "dvFord", "./results_odom_ros.txt" using ($32-k):24 with lines title "cvRos", "./results_odom_ros.txt" using ($32-k):28 with lines title "filtered-cvRos"
```

Para phi X phi bruto X phi filtrado:
```
k = "`head -1 ./results_odom_ros.txt | awk '{print $32}'`"
plot "./results_pid-steering.txt" using ($14-k):8 with lines title "cPhiFord", "./results_pid-steering.txt" using ($14-k):9 with lines title "dPhiFord", "./results_odom_ros.txt" using ($32-k):(-$26) with lines title "filtered-cPhiRos", "./results_odom_ros.txt" using ($32-k):(-$22) with lines title "cPhiRos"
```

-> Para fazer teste PID com argos

- Velocidade
grep VEL ../../bin/results_pid.txt > results_pid-velocity.txt
gnuplot

k = "`head -1 ./results_pid-velocity.txt | awk '{print $20}'`"
plot "./results_pid-velocity.txt" using ($20-k):13 with lines title "cv", "./results_pid-velocity.txt" using ($20-k):14 with lines title "dv", "./results_pid-velocity.txt" using ($20-k):15 with lines title "error", "./results_pid-velocity.txt" using ($20-k):($16/10) with lines title "throt", "./results_pid-velocity.txt" using ($20-k):($17/100) with lines title "breaks", "./results_pid-velocity.txt" using ($20-k):($18/1) with lines title "integ", "./results_pid-velocity.txt" using ($20-k):($19/10) with lines title "deriv"

- Para steering
grep STE ../../bin/results_pid.txt > results_pid-steering.txt
gnuplot

k = "`head -1 ./results_pid-steering.txt | awk '{print $14}'`"
plot "./results_pid-steering.txt" using ($14-k):8 with lines title "cphi", "./results_pid-steering.txt" using ($14-k):9 with lines title "dphi", "./results_pid-steering.txt" using ($14-k):10 with lines title "e", "./results_pid-steering.txt" using ($14-k):11 with lines title "i", "./results_pid-steering.txt" using ($14-k):($12/10) with lines title "d", "./results_pid-steering.txt" using ($14-k):($13/1000) with lines title "s"



