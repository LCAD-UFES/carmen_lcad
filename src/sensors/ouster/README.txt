
# how to run the driver: 
./ouster -sensor_ip <SENSOR IP> -host_ip<IP OF THE COMPUTER THAT WILL RECEIVE THE POINTCLOUDS> -mode<512x10|512x20|1024x10|1024x20|2048x10>
    -sensor_id [0-9] -publish_imu [on | off]  -intensity_type [1 | 2 | 3]

# if intensity_type == 1 -> INTENSITY 
# if intensity_type == 2 -> REFLECTIVITY
# if intensity_type == 3 -> NOISE

# if mode == 1024x20 -> horizontal resolution 1024, frequency 20Hz

# example
./ouster -sensor_ip 192.168.1.200 -host_ip 192.168.1.1 -sensor_id 0 -mode 1024x20 -publish_imu off -intensity_type 1

sensors ip list:
 OS164 192.168.1.200
 OS132 SN: 122144001108 IP: 192.168.1.205
 OS132 192.168.1.206
 OS032 192.168.1.207

# how to run the viewer:
./ouster_viewer -sensor_id <ID TO ASSIGNED TO THE VARIABLE VELODYNE MESSAGE: [0-9]>

to access sensors configuration page http://os-<IP or Serial>.local/
 ex: http://os-122144000315.local/
---------------------------------------------------------------------------------
Initial Configuration
---------------------------------------------------------------------------------
# On first acces you must set a fixed IP on the sensor and configure it to run at 20hz
You can connect at the first time by:
 (i)conecting the sensor to a router, so it will receive an IP vie DHCP or
 (ii) conecting the sensor direct to your Ubuntu/linux and creating an ethernet conection Link-local-Only. 
To identify the sensor IP, run: ping os-<serialNumber>.local,


  sudo apt install httpie
  echo '"192.168.1.207/24"' | http -v PUT http://169.254.188.238/api/v1/system/network/ipv4/override

192.168.1.200/24 is the new IP
http://192.168.1.101 is the IP given by the router
This command will print a message saying that its OK and the new IP Adress.
Disconnect from the link-local-Only network and connect the SensorBox Network

# Set the laser to run at 20hz
got to the page http://os-122144000315.local/
got to the Tab: Configuration and change the Lidar Mode to 1024x20 and Apply Config

To get the vertical angles and azimuth_angles (the rays of some ouster sensors aren't aligned!)
run:
  nc <sensor_ip> 7501
  get_beam_intrinsics
It should print the ouster32_altitude_angles(our vertical angles), beam_azimuth_angles(rays offset, in one shot, the lasers aren't pointing to the same direction) and lidar_origin_to_beam_origin_mm
OS132 is Aligned (this driver publish only one message)
OS132 is not Aligned (this driver publish 4 four messages)



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
# Connect to "os1-991901000584" (the sensor IP can be used instead) in port 7501. If the command fails, the sensor is unreachable.
# After using the command, the terminal will lock waiting for requests.
>> nc os1-991901000584 7501

# Send a request of sensor information. The expected output is presented bellow (note the "running" status in the end).
# Use Ctrl+C to close the netcat program.
>> get_sensor_info
{"prod_line": "OS-1-64", "prod_pn": "840-101396-03", "prod_sn": "991901000584", "base_pn": "000-101323-03", "base_sn": "101837000389", "image_rev": "ousteros-image-prod-aries-v1.10.0-20181211235856", "build_rev": "v1.10.0", "proto_rev": "v1.1.1", "build_date": "2018-12-11T23:23:23Z", "status": "RUNNING"}


---------------------------------------------------------------------------------
Coordinate Systems
---------------------------------------------------------------------------------
- The Sensor Coordinate Frame follows the right-hand rule convention and is defined at the center of the sensor housing on the bottom, with the x-axis pointed forward, y-axis pointed to the left and z-axis pointed towards the top of the sensor. The external connector is located in the negative x direction.
- The origin and axes of the lidar Coordinate Frame are defined by the position of the lidar lens aperture stop in the sensor and the 0º position of the rotary encoder, which is aligned with the sensor connector and the negative X axis of the Sensor Coordinate Frame
- The origin and axes of the lidar Coordinate Frame are defined by the position of the lidar lens aperture stop in the sensor and the 0º position of the rotary encoder, which is aligned with the sensor connector and the negative X axis of the Sensor Coordinate Frame


---------------------------------------------------------------------------------
Time Synchronization
---------------------------------------------------------------------------------
All lidar and IMU data are timestamped to a common timer with 10 nanosecond precision. The common timer can be programmed to run off one of three clock sources: 
- An internal clock derived from a high accuracy, low drift oscillator 
- An opto-isolated digital input from the external connector for timing off an external hardware trigger such as a GPS. The polarity of this input signal is programmable. For instance, both a GPS PPS pulse and a 30 Hz frame sync from an industrial camera can supply a timing signal to the OS-1.
- Using the IEEE 1588 Precision Time Protocol. PTP provides the convenience of configuring timing over a network that supports IEEE 1588 with no additional hardware signals.


---------------------------------------------------------------------------------
Calibration Information (obtained from the sensor using netcat)
---------------------------------------------------------------------------------
- JSON-formatted lidar transformation matrix needed to adjust to the Sensor Coordinate Frame:
    {"lidar_to_sensor_transform": [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1]}
- JSON-formatted imu transformation matrix needed to adjust to the Sensor Coordinate Frame:
    {"imu_to_sensor_transform": [1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1]}
- JSON-formatted beam altitude and azimuth offsets, in degrees.
    {"beam_altitude_angles": [16.367, 15.767, 15.204, 14.658, 14.175, 13.587, 13.038, 12.5, 12.034, 11.453, 10.909, 10.375, 9.906000000000001, 9.347, 8.804, 8.244999999999999, 7.794, 7.237, 6.695, 6.149, 5.688, 5.136, 4.591, 4.046, 3.589, 3.034, 2.497, 1.943, 1.483, 0.9409999999999999, 0.393, -0.147, -0.618, -1.156, -1.727, -2.261, -2.721, -3.267, -3.811, -4.359, -4.821, -5.363, -5.897, -6.456, -6.927, -7.471, -8.012, -8.57, -9.041, -9.582000000000001, -10.122, -10.69, -11.161, -11.702, -12.259, -12.823, -13.305, -13.848, -14.405, -14.982, -15.471, -16.017, -16.586, -17.188], "beam_azimuth_angles": [3.092, 0.918, -1.241, -3.382, 3.073, 0.913, -1.217, -3.346, 3.052, 0.9379999999999999, -1.201, -3.301, 3.04, 0.9419999999999999, -1.192, -3.282, 3.047, 0.951, -1.146, -3.237, 3.053, 0.953, -1.134, -3.217, 3.076, 0.987, -1.113, -3.195, 3.089, 1.007, -1.093, -3.175, 3.11, 1.026, -1.128, -3.16, 3.132, 1.04, -1.042, -3.129, 3.166, 1.077, -1.035, -3.129, 3.194, 1.101, -0.996, -3.112, 3.238, 1.138, -0.984, -3.111, 3.275, 1.159, -0.961, -3.103, 3.337, 1.2, -0.945, -3.091, 3.394, 1.227, -0.922, -3.112]}

- IMPORTANTE: para readquirir esses valores, siga as instrucoes do final da pag. 4 do manual. Os passos sao:
    - instalar o dnsmasq e configurar um dhcp local
    - criar uma rede local entre o sensor e o seu computador
    - descobrir o nome do sensor na rede dhcp
    - iniciar o netcat para trocar informacoes com o sensor (ex.: "nc os1-991901000584 7501")
    - enviar comandos (ex.: "get_lidar_intrinsics")



