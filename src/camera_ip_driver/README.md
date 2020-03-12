# Cameras

## Intelbras

Infos Camera 1 VIP 3220 B:

IP: 192.168.1.108
Login: lcad
Senha: 1q2w3e4r

```
./rtsp_camera_driver 10
```

Infos Camera 2 VIP 1020 B G2:

IP: 192.168.1.109
Login: lcad
Senha: 1q2w3e4r

```
./rtsp_camera_driver 11
```

Infos Camera 3 VIP 1020 B G2:

IP: 192.168.1.110
Login: lcad
Senha: 1q2w3e4r

```
./rtsp_camera_driver 12
```

! Atenção: Caso dê algum erro, lembre-se de colocar a contra-barra antes do & (e comercial) no rtsp_address !

# Com qualquer camera que tenha protocolo RTSP, faça:

### Intelbras VIP 3220 B
Usando o canal extra, compressão MJPEG, 640x480, 30fps, taxa de bit 6144 e latência de 1 décimo de segundo: 
```
./rtsp_camera_driver 10 
``` 
com bumblebee_basic10_rtsp_address no carmen-ford-escape.ini -> rtsp://admin:lcad123456@192.168.1.108:554/cam/realmonitor?channel=1\&subtype=1

Mainstream, H264H, 1280x720, 15fps, 1280 bit e latência de 3 décimos de segundo:
``` 
./rtsp_camera_driver 10  
``` 
com bumblebee_basic10_rtsp_address no carmen-ford-escape.ini -> rtsp://admin:lcad123456@192.168.1.108:554/cam/realmonitor?channel=1\&subtype=0

### Exemplo de trecho do carmen-ford-escape.ini:
``` 
#--------- IP Camera 1 IntelBras VIP 3220-B Sensorbox ------------

bumblebee_basic10_width		640 #1280 #1920
bumblebee_basic10_height	480 #720  #1080
bumblebee_basic10_fx		883.2282992047736
bumblebee_basic10_fy		885.2728294386309
bumblebee_basic10_cu		699.0716498673814
bumblebee_basic10_cv		380.0716498673814
bumblebee_basic10_baseline	0.0
bumblebee_basic10_pixel_size 0.00000375
bumblebee_basic10_is_legacy 	off
bumblebee_basic10_is_rectified 	on
bumblebee_basic10_model 		KITTI
bumblebee_basic10_fov		0.576
bumblebee_basic10_tlight_roi_x		160   	# image_width / 4
bumblebee_basic10_tlight_roi_y		0 		# 0
bumblebee_basic10_tlight_roi_w		320 	# image_width / 2
bumblebee_basic10_tlight_roi_h		240		# image_height / 2
bumblebee_basic10_tlight_focal_dist	1500.0
bumblebee_basic10_tlight_dist_correction	6.0
bumblebee_basic10_k1		-0.3764240432184582 #calibration parameters
bumblebee_basic10_k2		0.1480679133077704
bumblebee_basic10_p1		-0.0010284565476715443
bumblebee_basic10_p2		-0.005797863284201866
bumblebee_basic10_k3		-0.03274951846122666
bumblebee_basic10_rtsp_address rtsp://admin:lcad123456@192.168.1.108:554/cam/realmonitor?channel=1\&subtype=1
``` 

### UNV Camera 
``` 
./rtsp_camera_driver 2 
``` 
com bumblebee_basic2_rtsp_address no carmen-ford-escape.ini: rtsp://admin:123456@192.168.0.23/media/video1.cgi?.mjpg


# Camera IP UNV modelo IPC322ER3-DVPF28

-> Acesso através da criação de uma rede ethernet no Ubuntu com IP 192.168.0.1 e mask 24.
-> Entra na camera através do ip 192.168.0.23
- Setup
- No menu esquerdo, em Video & Audio, altera para o modo de captura PAL
- Em Video Compression, alterar para MJPEG
- Alterar Image Quality para Quality
- Smoothing - Clear

## Parametrização Carmen UNV

- No carmen-ford-escape.ini colocar a linha para camera 2
```
#--------- UNV Camera 2 ------------
bumblebee_basic2_width      640 #1280 #1920
bumblebee_basic2_height     480 #720  #1080
```
Os dados de altura e comprimento são inicializados quando a câmera é iniciada com o comando 
```
./unv_camera_client_driver 2 192.168.0.23
```

Qualquer mudança no tamanho da imagem no menu de Video & Audio da câmera, deve-se reiniciar o módulo unv_camera_client_driver para que faça a reinicialização dos parâmetros.

## Para testar execute a central, proccontrol e o driver.
```
~/carmen_lcad/bin$ ./central
~/carmen_lcad/bin$ ./proccontrol process-volta_da_ufes_playback_viewer_3D.ini
~/carmen_lcad/bin$ ./unv_camera_client_driver 2 192.168.0.23

```

