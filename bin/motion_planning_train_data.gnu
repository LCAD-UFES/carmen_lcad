!cat motion_train_data_01.txt | grep ODOM_ACK > odom.txt
!cat motion_train_data_01.txt | grep BASEMOTION_ACK > comm.txt

#set terminal png
#set output "motion_cmd.png"
set xlabel "secs"
set ylabel "m/s"

plot [*:*] [-1:1] "odom.txt" using 10:6 with lines, "comm.txt" using 55:4 with lines

pause -1
