## Ouster from ouster C++ SDK version 0.4.1

Esse módulo usa o SDK mais atual da ouster (09/08/2022) e está pronto para rodar todos os sensores pegando os parâmetros 
do sensor automaticamente (sem necessidade de gravar no carmen-ini ou fixar para sempre um numero de laser)

TODO:

- Os raios do ouster tem azimuth offset diferente em relação ao zero, ou seja, os 32 raios não estão alinhados
dessa forma, para gerar uma nuvem de pontos mais correta possivel, seria necessário que além de usar o Vertical angle, usar o azimuth angle na hora de converter os pontos
para cartesiano.
Como a mensagem Variable Partial scan assume que os raios de cada shot esteja alinhado, a correção é feita apenas cnsiderando o offset do primeiro raio.
Isso só é útil para o viewer 3D e talvez para módulos que usem o velodyne_camera_calibration para converter a posição dos pontos corretamente, para a evidencia de obstáculo, por ser pouca a diferença
traria mais problemas do que ganho (Opnião do Vinicius)

- Publicar imagem gerada pela reflexão do sensor, para testarmos detecção na imagem do lidar

###Instalação

Baixe o SDK 0.4.1 usado -> https://drive.google.com/file/d/1L5wQ-yqk9REpW62EWWOS1pmnvYHFsHPE/view?usp=sharing \
baixe as dependencias: \
sudo apt install build-essential cmake libjsoncpp-dev libeigen3-dev libcurl4-openssl-dev \
                   libtins-dev libpcap-dev libglfw3-dev libglew-dev \
                   
mova a pasta baixada com nome ouster_example-master para a pasta packages_carmen \
	build \
	cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_VIZ=ON -DBUILD_PCAP=ON -DBUILD_EXAMPLES=ON .. \
	make \
	sudo make instal \

compile o modulo ouster_sdk_driver em $CARMEN_HOME/src/ouster_sdk_driver \

## how to run the driver: 
./ouster -sensor_ip <SENSOR IP> -host_ip<IP OF THE COMPUTER THAT WILL RECEIVE THE POINTCLOUDS> -sensor_id <ID TO THE VARIABLE VELODYNE MESSAGE AND CAMEN.INI PARAMETER: [0-9]> -mode<512x10|512x20|1024x10|1024x20|2048x10> -publish_imu [on | off]  -intensity_type [1 | 2 | 3]

## example
./ouster -sensor_ip 192.168.1.200 -host_ip 192.168.1.1 -sensor_id 0 -mode 1024x20 -publish_imu off -intensity_type 1

## if intensity_type == 1 -> INTENSITY 
## if intensity_type == 2 -> REFLECTIVITY
## if intensity_type == 3 -> NOISE

## if mode == 1024x20 -> horizontal resolution 1024, frequency 20Hz

##ANY QUESTION ABOUT PARAMETERS, CHECK: $CARMEN_HOME/src/sensors/ouster/PARAMETROS_LIDAR_OUSTER.txt

our sensors ip list:
 OS164 SN: 991901000584 IP: 192.168.1.200
 OS132 SN: 122144001108 IP: 192.168.1.205
 OS032 SN: 122145000764 IP: 192.168.1.206
 OS032 SN: 122144000315 IP: 192.168.1.207
 OS132-LCAD SN:122144001128 IP: 192.168.1.208

to access sensors configuration page http://os-<IP or Serial>.local/
 ex: http://os-122144000315.local/
---------------------------------------------------------------------------------
Initial Configuration
---------------------------------------------------------------------------------
## On the first use of the sensor you must set a fixed IP on the sensor and configure it to run at 20hz
You can connect at the first time by:
 (i) conecting the sensor to a router, so it will receive an IP vie DHCP or
 (ii) conecting the sensor direct to your Ubuntu/linux and creating an ethernet conection Link-local-Only. 
To identify the sensor IP, run:
 ping os-<serialNumber>.local,


  sudo apt install httpie
  echo '"192.168.1.208/24"' | http -v PUT http://169.254.110.2/api/v1/system/network/ipv4/override

192.168.1.200/24 is the new IP
http://169.254.188.238 is the IP given by the router
This command will print a message saying that its OK and the new IP Adress.
Disconnect from the link-local-Only network and connect the SensorBox Network

## Set the laser to run at 20hz
got to the page http://os-122144000315.local/ or http://192.168.1.200/
got to the Tab: Configuration and:
	 change the Lidar Mode to 1024x20
	 and if you will use two LIDARS ouster together, change the lidar and imu ports
 	Apply Config and Save Config


To get the vertical angles and azimuth_angles (the rays of some ouster sensors aren't aligned!)
run:
  nc <sensor_ip> 7501
  get_beam_intrinsics
It should print the ouster32_altitude_angles(our vertical angles), beam_azimuth_angles(rays offset, in one shot, the lasers aren't pointing to the same direction) and lidar_origin_to_beam_origin_mm
OS164 is not Aligned, there are 4 sets of 16 rays aligned (this driver will publish 4 four messages)
OS132 is Aligned (this driver publish only one message)
OS032 is not Aligned ALL RAYS have differet azimuth angles (this driver doesn't work with this one yet) 



---------------------------------------------------------------------------------
General Informatios About OS1 Sensor
---------------------------------------------------------------------------------
- Driver/Examples provided by Ouster: https://github.com/ouster-lidar/ouster_example
    - The driver requires libjsoncpp-dev 
    - The viewer requires libvtk6-dev libeigen3-dev
- The sensor typically consumes 14-16 Watts. 
- The recommended T_heatsink is < 25 °C above ambient -- meaning the sensor can get to 45-50 degrees Celsius with a heat sink, or even higher, without performance degradation.
- No manual, o IP UDP de destino eh o IP do computador que vai receber os dados do sensor.
- Ele se conecta em redes dhcp, mas parece que ele so pede o IP quando o cabo de energia e ligado. 
- Eh possivel configurar no sensor o endereco UDP para o qual os dados serao enviados.
- Se for ligar o sensor em um computador ao inves de um switch, ler no manual as instrucoes. 
    - ler secao "Basic DHCP Setup" no link de instalacao do "dnsmasq".
- Vertical field of view: +16.6° to -16.6° (32.2°) with uniform spacing (OS-1-16  has multiple options)
- Horizontal Resolution: 512-2048 (configurable)
- Data Per Point: Range, intensity, ambient, reflectivity, angle, timestamp
- IMU Samples Per Second: 1000. Data Per Sample: 3-axis gyro, 3-axis accelerometer.
- Operation modes: One of “512x10”, “1024x10”, “2048x10”, “512x20”, “1024x20”.
- Interesting comment regarding operation modes: Each 50% the total number of points gathered is reduced - e.g., from 2048x10 to 1024x10 - extends range by 15-20%.
- range resolution 1.2cm 
- range:
    - 0.5 m - 120 m @ 80% reflective lambertian target, 225 w/m2 sunlight, SNR of 12 
    - 0.5 m - 40 m @ 10% reflective lambertian target, 225 w/m2 sunlight, SNR of 12 
    - range of 0.0-80m (min range of 0.0m) in enhanced low range mode
    - NOTE: Zero bias for lambertian targets, slight bias for retroreflectors.
- sensor hostname: "os1-991901000584.local"
- Vertical: ±0.01degrees / Horizontal: ±0.01degrees 


---------------------------------------------------------------------------------
Basic connectivity test using netcat (program from linux)
---------------------------------------------------------------------------------
## Connect to "os1-991901000584" (the sensor IP can be used instead) in port 7501. If the command fails, the sensor is unreachable.
## After using the command, the terminal will lock waiting for requests.
>> nc os1-991901000584 7501

## Send a request of sensor information. The expected output is presented bellow (note the "running" status in the end).
## Use Ctrl+C to close the netcat program.
>> get_sensor_info
{"prod_line": "OS-1-64", "prod_pn": "840-101396-03", "prod_sn": "991901000584", "base_pn": "000-101323-03", "base_sn": "101837000389", "image_rev": "ousteros-image-prod-aries-v1.10.0-20181211235856", "build_rev": "v1.10.0", "proto_rev": "v1.1.1", "build_date": "2018-12-11T23:23:23Z", "status": "RUNNING"}
---------------------------------------------------------------------------------
Sensor Angles
---------------------------------------------------------------------------------
## OS1-32-BH
{"beam_altitude_angles": [-0.98, -1.67, -2.38, -3.07, -3.77, -4.49, -5.18, -5.89, -6.59, -7.27, -7.96, -8.66, -9.34, -10.04, -10.72, -11.4, -12.08, -12.75, -13.43, -14.09, -14.75, -15.41, -16.08, -16.72, -17.38, -18, -18.66, -19.27, -19.91, -20.52, -21.14, -21.74], "beam_azimuth_angles": [4.2, -1.4, 4.21, -1.4, 4.22, -1.41, 4.2, -1.41, 4.2, -1.39, 4.22, -1.4, 4.22, -1.4, 4.22, -1.39, 4.22, -1.39, 4.23, -1.39, 4.23, -1.39, 4.24, -1.39, 4.23, -1.38, 4.23, -1.38, 4.24, -1.38, 4.24, -1.37], "lidar_origin_to_beam_origin_mm": 15.806}
## OS1-32-U
{"beam_altitude_angles": [20.51, 19.25, 17.99, 16.68, 15.38, 14.06, 12.72, 11.37, 10.01, 8.619999999999999, 7.24, 5.87, 4.46, 3.06, 1.64, 0.23, -1.17, -2.58, -3.99, -5.39, -6.78, -8.16, -9.539999999999999, -10.92, -12.28, -13.63, -14.94, -16.26, -17.56, -18.85, -20.1, -21.33], "beam_azimuth_angles": [-1.5, -1.49, -1.48, -1.49, -1.48, -1.47, -1.46, -1.45, -1.45, -1.45, -1.44, -1.42, -1.43, -1.42, -1.42, -1.41, -1.4, -1.4, -1.39, -1.38, -1.37, -1.36, -1.36, -1.37, -1.35, -1.35, -1.33, -1.33, -1.33, -1.33, -1.31, -1.29], "lidar_origin_to_beam_origin_mm": 15.806}
## OS2-32-BH
"beam_altitude_angles": [-0.04, -0.41, -0.74, -1.11, -1.46, -1.8, -2.15, -2.5, -2.85, -3.19, -3.54, -3.89, -4.23, -4.58, -4.93, -5.29, -5.62, -5.98, -6.31, -6.67, -7, -7.34, -7.68, -8.039999999999999, -8.380000000000001, -8.73, -9.039999999999999, -9.4, -9.73, -10.08, -10.41, -10.75], "beam_azimuth_angles": [2.08, -0.6899999999999999, 2.08, -0.6899999999999999, 2.08, -0.68, 2.1, -0.67, 2.08, -0.67, 2.09, -0.6899999999999999, 2.08, -0.67, 2.1, -0.68, 2.09, -0.68, 2.09, -0.68, 2.09, -0.67, 2.1, -0.68, 2.1, -0.68, 2.1, -0.68, 2.09, -0.68, 2.1, -0.67], "lidar_origin_to_beam_origin_mm": 13.762
