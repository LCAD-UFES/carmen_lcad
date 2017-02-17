Como fazer as anotações[quebra-molas = bump, cancela=barrier]:

1.Execute o central

2.Modifique o process-volta_da_ufes_playback_viewer_3D.ini para que ele faça playback de seu log:
		./playback [seu log]
3.Certifique-se de comentar a linha do rddf_play no process-volta_da_ufes_playback_viewer_3D.ini:
		./rddf_play ../data/rndf/rddf-log_voltadaufes-20160513.kml [nome do seu arquivo de anotações]

4.Execute:
	 ./rddf_annotation_manager [nome do seu arquivo de anotações].

5.Rode o playback do seu log(aperte play)

6.Vá na aba do programa viewer3D aperte o botão options, depois disso o botão velodyne e após isso o botão annotations. 
Note que os pontos do velodyne que aparecem no viewer3D engordaram após isso.

7.Quando você encontrar um ponto que corresponde a uma anotação, comparando com as imagens vindas da câmera do bumblebee[caso tenha],
aperte pause. Clique com botão direito no ponto que corresponde a uma anotação, veja que apareceu um ponto branco aonde você deu um clique.
Após isso aperte o botão do tipo que você quer, note que o ponto que era branco antes agora ficou verde,indicando que esse ponto foi anotado com sucesso
no seu arquivo de anotações.

8.Faça o setimo passo até o final do seu playback.

9. kill ./rddf_annotation_manager and process

 
