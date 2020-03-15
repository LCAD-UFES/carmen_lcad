

## IMPORTANT: see doc/ouster_manuals/info-uteis.txt - To use connected on pc, you need to use a DHCP program (see the manual)

Se já não tiver sido feito, para setar um IP fixo conecte ao laser (usando um roteador para ele pegar um IP via DHCP)

  sudo apt install httpie
  echo '"192.168.1.200/24"' | http -v PUT http://192.168.1.101/api/v1/system/network/ipv4/override

Onde 192.168.1.200/24 é o IP novo
http://192.168.1.101 é o IP dado pelo Roteador
Foi necessário para usá-lo no switch

# how to run the driver: 
./ouster -sensor_hostname <SENSOR IP or NAME> -ip_destination_computer <IP OF THE COMPUTER THAT WILL RECEIVE THE POINTCLOUDS> \
    -sensor_id [0-9] -publish_imu [ON | OFF]  -intensity_type [1-3] \
    -horizontal_resolution [512 | 1024 | 2048] -revolution_frequency [10 | 20] 

# example
./ouster -sensor_hostname os1-991901000584 -ip_destination_computer 192.168.100.101 -sensor_id 0 -publish_imu off -intensity_type 1 -horizontal_resolution 1024 -revolution_frequency 10

# how to run the viewer:
./ouster_viewer -sensor_id <ID TO ASSIGNED TO THE VARIABLE VELODYNE MESSAGES: [0-9]>

#example
./ouster_viewer -sensor_id 0

