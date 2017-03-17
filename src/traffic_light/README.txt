************************************
Fazer Anotações de Semáforo:
************************************

(1) Execute o central
(2) Modifique o process-traffic-light-annotation.ini para que ele faça playback de seu log:
	./playback [seu log]
(3) Modifique também a parte rddf_annotation_manager para que ele faça as anotações no final do arquivo que você quer:
	./rddf_annotation_manager [nome do seu arquivo de anotações].
(4) Modifique  também a parte rndf do arquivo process-traffic-light-annotation.ini para o arquivo de suas anotações:
	./rddf_play ../data/rndf/rddf-log_voltadaufes-20160513.kml [nome do seu arquivo de anotações]
(5) Rode o playback do seu log(aperte play)
(6) Vá na aba do programa viewer3D aperte o botão options, depois disso o botão velodyne e após isso o botão annotations. 
    Note que os pontos que aparecem no viewer3D engordaram após isso.
(7) Volte seu playback desde o início.
(8) Quando você encontrar um ponto que corresponde a um semáforo no viewer3D, comparando com as imagens vindas da câmera do bumblebee,
aperte pause. Clique com botão direito no ponto que corresponde a um semáforo, veja que apareceu um ponto branco aonde você deu um clique.
Após isso aperte no traffic light, note que o ponto que era branco antes agora ficou verde,indicando que esse ponto foi anotado com sucesso
no seu arquivo de anotações.
(9) Faça o oitavo passo até o final do seu playback.

******************************************* 
Visualisar Anotações de Semáforo:
*******************************************

(1) Verifique se no arquivo carmen-ford-escape.ini da pasta src a resolução da câmera do bumblebee que você usando está em 1280X960.
(2) Ajuste a resolução do traffic_light_viewer para a resolução que deseja visualisar a janela do traffic_light_viewer
(3) Execute o central
(4) Modifique o process-traffic-light-annotation.ini para que ele faca playback de seu log:
		./playback [seu log]
(5) Modifique  também a parte rndf para o arquivo de suas anotações:
		./rddf_play ../data/rndf/rddf-log_voltadaufes-20160513.kml [nome do seu arquivo de anotações]
(6) Execute ./proccontrol process-traffic-light-viewer.ini

************************************
Treinar Algorítmo de Detecção:
************************************
