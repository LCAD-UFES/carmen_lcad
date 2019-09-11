# Camera IP UNV modelo IPC322ER3-DVPF28

-> Acesso através da criação de uma rede ethernet no Ubuntu com IP 192.168.0.1 e mask 24.
-> Entra na camera através do ip 192.168.0.23
- Setup
- No menu esquerdo, em Video & Audio, altera para o modo de captura PAL
- Em Video Compression, alterar para MJPEG
- Alterar Image Quality para Quality
- Smoothing - Clear

# Parametrização Carmen

- No carmen-ford-escape.ini colocar a linha para camera 2
#--------- UNV Camera 2 ------------
camera2_width      640 #1280 #1920
camera2_height     480 #720  #1080

Os dados de altura e comprimento são inicializados quando a câmera é iniciada com o comando ./unv_camera_client_driver 2 192.168.0.23

Qualquer mudança no tamanho da imagem no menu de Video & Audio da câmera, deve-se reiniciar o módulo unv_camera_client_driver para que faça a reinicialização dos parâmetros.

# Para testar execute a central, proccontrol e o driver.
~/carmen_lcad/bin$ ./central
~/carmen_lcad/bin$ ./proccontrol process-volta_da_ufes_playback_viewer_3D.ini
~/carmen_lcad/bin$ ./unv_camera_client_driver 2 192.168.0.23