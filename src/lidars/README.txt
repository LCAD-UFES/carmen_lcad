Este módulo atualmente será usado para publicar velodyne HL-32 e Puck usando a mensagem variable velodyne. Pode substituir o módulo do velodyne mas usa a velodyne_interface.h e messagens dele

Para rodar:
Compile o modulo velodyne para ter acesso a mensagem velodyne variable scan
entre no módulo lidars e dê make clean && make
rode 
./lidars -velodyne_number 2
onde velodyne_number é o número do lidar que você irá usar (parametros velodyne0..1... no carmen-ini)
segue a ideia das bumblebee cada laser tem um número especifíco e suas configurações todos publicam mensagem 
carmen_velodyne_variable_scan_message<id>

Para visualizar no viewer 3D é necessário usar as flags abaixo

./viewer_3D -velodyne_active 2 -fv_flag on
 
 Se não estiver com GPS use a flag -fv_flag on para visualizar o velodyne
a flag -fv_flag serve para que o viewer ignore o GPS

