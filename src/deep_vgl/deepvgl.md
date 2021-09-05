# DeepVGL

## Utilizando o módulo 

Para utilizar o módulo deep_vgl integrado ao carmen precisaremos dos seguintes arquivos:
* deepvgl.cfg               # as configurações da rede
* poses_and_labels.txt      # tabela relacionando poses  (x, y, yaw) com as imagens aprendidas durante o treino
* deepvgl_final.weights     # os pesos da rede, gerados durante a execução do treino da darknet (vários serão gerados e podem ser avaliados)

Todos esses arquivos foram gerados nas etapas de treinamento e de teste da rede.

Copie-os para a pasta $CARMEN_HOME/data/deep_vgl colocando o nome do mapa como prefixo. Para o exemplo mostrado em treino.md e teste.md,
a cópia seria como abaixo: 


```bash 
cp $CARMEN_HOME/sharedlib/darknet4/backup/deepvgl_final.weights 	$CARMEN_HOME/data/deep_vgl/map_volta_da_ufes-20210131-art2-deepvgl_final.weights
cp /dados/ufes/deepvgl.cfg 									$CARMEN_HOME/data/deep_vgl/map_volta_da_ufes-20210131-art2-deepvgl.cfg
cp $CARMEN_HOME/src/deep_vgl/config/poses_and_labels.txt 			$CARMEN_HOME/data/deep_vgl/map_volta_da_ufes-20210131-art2-poses_and_labels.txt
```

e podem ser utilizados normalmente. Para isso, inicie o carmen com a seguinte sequencia de comandos.

1° rode a central:

```bash
cd $CARMEN_HOME/bin
./central
```

Em outro terminal execute o process.ini de sua preferência que publique imagens de câmera. Lembre-se de desligar o gps_xyz, pois queremos utilizar o deep_vgl para essa função.

Edite o process escolhido para o seu caso e rode o proccontrol ( em nosso exemplo utilizamos o process-volta_da_ufes_playback_viewer_3D.ini)

```bash
cd $CARMEN_HOME/bin
./proccontrol process-volta_da_ufes_playback_viewer_3D.ini

```

Depois disso é só rodar o módulo com seus parâmetros:

```bash
 ./deep_vgl  ../src/deep_vgl/config/deepvgl_lidar.cfg ../src/deep_vgl/config/deepvgl_lidar_270.weights ../src/deep_vgl/config/poses_and_labels_lidar.txt 0 0 640 480 1 135 135 -camera_id 1
```

(Lembre-se de colocar o id da câmera utilizada no log, nesse exemplo é 1)

Acima, temos:

```bash
01- a configuração da rede;
02- são os pesos da rede;
03- são as poses e labels
04- crop_width (ver treino.md);
05- crop_height (ver treino.md);
06- width da imagem (ver treino.md);
07- height da imagem (ver treino.md);
08- use_lidar: 0 para não utilizar lidar e 1 para utilizar lidar
09- angle_left: ângulo inicial do velodyne (180 pega tudo à esquerda);
10- angle_right: ângulo inicial do velodyne (180 pega tudo à direita);
11- camera_id. câmera desejada.
```

O módulo dnn_visual_gl vai publicar mensagens gps_xyz e gps gphdt que permitirão localização global.
Use bin/gps_xyz-test se quiser examinar (só as informações essenciais à localização global são incluídas nas mensagens).

