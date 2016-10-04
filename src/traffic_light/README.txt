Como fazer as anotações do semáforo:

OBS: CASO VOCÊ TENHA FEITO AS ANOTAÇÕES DO SEMÁFORO VÁ PARA PARTE COMO VISUALISAR SUAS ANOTAÇÕES DE SEMÁFORO

1.Execute o central
2.Modifique o process-traffic-light-annotation.ini para que ele faça playback de seu log:
		./playback [seu log]
3.Modifique também a parte rddf_annotation_manager do process-traffic-light-annotation.ini para que ele faça as anotações no arquivo 
que você quer:
	 ./rddf_annotation_manager [nome do seu arquivo de anotações].
4.Rode o playback do seu log(aperte play)
5.Vá na aba do programa viewer3D aperte o botão options, depois disso o botão velodyne e após isso o botão annotations. 
Note que os pontos que aparecem no viewer3D engordaram após isso.
6.Volte seu playback desde o início.
7.Quando você encontrar um ponto que corresponde a um semáforo no viewer3D, comparando com as imagens vindas da câmera do bumblebee,
aperte pause. Clique com botão direito no ponto que corresponde a um semáforo, veja que apareceu um ponto branco aonde você deu um clique.
Após isso aperte no traffic light, note que o ponto que era branco antes agora ficou verde,indicando que esse ponto foi anotado com sucesso
no seu arquivo de anotações.
8.Faça o sétimo passo até o final do seu playback.

Como visualisar suas anotações de semáforo:

1.Execute o central
2.Modifique o process-traffic-light-annotation.ini para que ele faca playback de seu log:
		./playback [seu log]
3.Modifique  também a parte rndf do arquivo process-traffic-light-viewer.ini para o arquivo de suas anotações:
		./rddf_play ../data/rndf/rddf-log_voltadaufes-20160513.kml [nome do seu arquivo de anotações]
4.Execute ./proccontrol process-traffic-light-viewer.ini.
 
