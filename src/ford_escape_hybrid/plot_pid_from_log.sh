#/bin/bash
 if [ "$#" != "1"  ]; then
	 exit
 fi
 vel_pid="velocity_pid_from_log.txt"
 ste_pid="steering_pid_from_log.txt"
 grep FORD_ESCAPE_VELOCITY_PID_DATA_MESSAGE "$1" > $vel_pid 
 grep FORD_ESCAPE_STEERING_PID_DATA_MESSAGE "$1" > $ste_pid 
 
 # numero da coluna do timestamp do vel_pid
t_vel=$(head -1 $vel_pid | awk '{ print NF}') 
t_vel=$(($t_vel-2))
 # numero da coluna do timestamp do ste_pid
t_ste=$(head -1 $ste_pid | awk '{ print NF}') 
t_ste=$(($t_ste-2))
gnuplot --persist -e " 
 set terminal qt title 'Velocity Pid'; 
    plot \"$vel_pid\" u $t_vel:3 w l title 'current velocity' , 
         \"$vel_pid\" u $t_vel:4 w l title 'desired velocity' ,
         \"$vel_pid\" u $t_vel:5 w l title 'error t' ,
         \"$vel_pid\" u $t_vel:6 w l title 'integral t' ,
         \"$vel_pid\" u $t_vel:7 w l title 'derivative t' ,
         \"$vel_pid\" u $t_vel:8 w l title 'throttle command',
         \"$vel_pid\" u $t_vel:9 w l title 'brakes command' ,

; pause mouse close" &

gnuplot --persist -e " 
 set terminal qt 1 title 'Steering Pid';
    plot \"$ste_pid\" u $t_ste:2 w l title 'atan current curvature',
         \"$ste_pid\" u $t_ste:3 w l title 'atan desired curvature',
         \"$ste_pid\" u $t_ste:4 w l title 'error t',
         \"$ste_pid\" u $t_ste:5 w l title 'integral t',
         \"$ste_pid\" u $t_ste:6 w l title 'derivative t',
         \"$ste_pid\" u $t_ste:7 w l title 'effort',
 ; pause mouse close" &
