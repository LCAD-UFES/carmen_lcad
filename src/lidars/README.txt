ROBOSENSE ESTA ENVIANDO OS DADOS PARA UM IP FIXO-MUDAR NO DRIVER DELE PARA MANDAR PARA O HOST 192.168.1.1

Este módulo atualmente será usado para publicar Puck (VLP16) e Robosense (RS16) usando a mensagem carmen_velodyne_variable_scan_message (velodyne_messages).
Pode também publicar o velodyne HL-32 porém precisa ser testado e comparado com o drive atual que publica a partial_scan.
 Pode substituir o módulo do velodyne mas usa a velodyne_interface.h e messagens dele.

Para rodar:
Compile o modulo velodyne para ter acesso a mensagem carmen_velodyne_variable_scan_message
entre no módulo #CARMEN_HOME/src/lidars e dê make clean && make
rode 
./lidars <ID>
onde ID é o número do lidar que você irá usar (parametros lidar0, lidar1 ... no carmen-ini)
ex:
./lidars 4
(todos os parâmetros serão ilistrados adiante):

segue a ideia das bumblebee, cada laser tem um número especifíco e suas configurações todos publicam mensagem 
carmen_velodyne_variable_scan_message<id>
o lidarID_model irá definir qual drive será rodado 

RS16 para lidars robosense de 16 raios

VLP16 para lidars Puck (velodyne) de 16 raios

HDL32 para lidars Velodyne de 32 raios

O Ouster tem um módulo separado só para ele em ../src/sensors/ouster.


Para visualizar no viewer 3D é necessário usar as flags abaixo

./viewer_3D -fv_flag on ir no menu Options>Lidars e ativar o seu lidar ID
 
 Se não estiver com GPS use a flag -fv_flag on para visualizar o velodyne
a flag -fv_flag serve para que o viewer start sem o GPS

Os parâmetros que devem estar no seu carmen.ini do laser seguem a logica do lidarID e os atuais (25/11/2021) sao:
EVITE SUBSTITUIR PARÂMETROS DE OUTROS LASERS, CRIE UM NOVO lidar para manter sempre um conjunto estável de parâmetros para cada modelo de lidar.

 -Para o lidar4 e nesse exemplo model RS16:

#--------- Lidar 4 ------------
lidar4_model		 RS16 # VLP16
lidar4_ip            192.168.1.200
lidar4_port 		 6699
lidar4_shot_size    16
lidar4_min_sensing	 1000 			# TODO verify in the manual
lidar4_max_sensing	 24000 			# TODO verify in the manual
lidar4_range_division_factor  200
lidar4_time_between_shots     0.000046091445    # approximately 1.0 / (1808.0 * 12.0) (see Velodyne manual)
lidar4_x       0.145
lidar4_y       0.0
lidar4_z       0.48
lidar4_roll    0.0
lidar4_pitch  -0.0227
lidar4_yaw    -0.01
lidar4_ray_order 0 1 2 3 4 5 6 7 15 14 13 12 11 10 9 8
lidar4_vertical_angles -15.0094 -13.014 -11.017 -9.0134 -7.0086 -5.0078 -3.0124 -1.0097 14.9927 12.997 10.9928 9.003 6.991 4.9935 2.991 0.9954


 -Para o Localize:

localize_ackerman_lidar4				on
localize_ackerman_lidar4_locc 				
localize_ackerman_lidar4_lfree				
localize_ackerman_lidar4_l0				
localize_ackerman_lidar4_unexpeted_delta_range_sigma 	
localize_ackerman_lidar4_range_max_factor		

 -Para o Mapper:

mapper_lidar4        on
mapper_lidar4_locc   
mapper_lidar4_lfree  
mapper_lidar4_l0     
mapper_lidar4_unexpeted_delta_range_sigma   


