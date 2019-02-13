

## IMPORTANT: see doc/ouster_manuals/info-uteis.txt


# how to run the driver: 
./ouster -sensor_hostname <SENSOR IP or NAME> -ip_destination_computer <IP OF THE COMPUTER THAT WILL RECEIVE THE POINTCLOUDS> -sensor_id <ID TO ASSIGN TO THE VARIABLE VELODYNE MESSAGES: [0-9]> -publish_imu <[ON | OFF]>

# example
./ouster -sensor_hostname os1-991901000584 -ip_destination_computer 192.168.100.101 -sensor_id 0 -publish_imu off

# how to run the viewer:
./ouster_viewer -sensor_id <ID TO ASSIGNED TO THE VARIABLE VELODYNE MESSAGES: [0-9]>

#example
./ouster_viewer -sensor_id 0

