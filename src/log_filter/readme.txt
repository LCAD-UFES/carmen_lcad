- Para rodar e capturar os dados de log
 ./proccontrol process-volta_da_ufes_playback_viewer_3D.ini {altere para o log de interesse}
 ./log_filter bumblebee images with_image_pose {de dentro do src/log_filter}
 - log_filter vai gerar o arquivo image_pose.txt
 
- Para gerar o dataset ver arquivo make_dataset_from_raw_data.lua

- Para treinar e testar a rede ver arquivo main.lua
