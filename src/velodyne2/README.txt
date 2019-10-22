Este módulo atualmente pode substituir o módulo do velodyne

Para rodar:
Limpe o modulo velodyne
entre no módulo velodyne2 e dê make clean && make
rode 
./velodyne -velodyne_number 2
onde velodyne_number é o número do velodyne que você irá usar
segue a ideia das bumblebee cada laser tem um número especifíco e suas configurações todos publicam mensagem 
carmen_velodyne_variable_scan_message<id>

Para visualizar no viewer 3D é necessário usar as flags abaixo

./viewer_3D -velodyne_active 2 -fv_flag on
 
a flag -fv_flag serve para que o viewer ignore o GPS

