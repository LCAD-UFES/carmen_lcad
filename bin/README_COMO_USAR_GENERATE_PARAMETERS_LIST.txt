Como usar o programa ./generate_parameters_list:

Compile o modulo ouster_sdk_driver

Antes de processeguir é necessário ter feito todos os passos do tutorial descritos em: https://github.com/LumeRobotics/docs/tree/main/install_lidar_ouster

1.Rode no terminal:

cd ~/carmen_lcad/bin
./central

2.Execute em outra aba do terminal:

cd ~/carmen_lcad/
./param_daemon <caminho>/carmen-<de interesse>.ini 

3. Para executar o ./generate_parameters_list é nessesário passar nos campos designados o IP do sensor, a porta do lidar, a porta da IMU do lidar, o IP do computador que ira receber as nuvens de pontos, e a posição do lidar no veículo (0 - Sensor-Box, 5 - Left side, 7 - Right side ) 

./generate_parameters_list -sensor_ip <SENSOR IP> -port <LIDAR-PORT> -imu_port <IMU-LIDAR-PORT> -host_ip <IP OF THE COMPUTER THAT WILL RECEIVE THE POINTCLOUDS> -lidar_id <integer position for lidar in the vehicle >
 

## exemplo
./generate_parameters_list  -sensor_ip 192.168.1.200 -port 7502 -imu_port 7503 -host_ip 192.168.1.1 -lidar_id 7

