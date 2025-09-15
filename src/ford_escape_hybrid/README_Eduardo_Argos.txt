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
plot "./results_pid-velocity.txt" using ($20-k):13 with lines title "cvFord", "./results_odom_ros.txt" using ($32-k):24 with lines title "cvRos", "./results_odom_ros.txt" using ($32-k):28 with lines title "filtered-cvRos"
```

Para curvatura X curvatura bruta X curvatura filtrada:
```
k = "`head -1 ./results_odom_ros.txt | awk '{print $32}'`"
plot "./results_pid-steering.txt" using ($14-k):8 with lines title "ccFord", "./results_odom_ros.txt" using ($32-k):(-$30/$24) with lines title "filtered-ccRos", "./results_odom_ros.txt" using ($32-k):(-$20/$24) with lines title "ccRos"
```
