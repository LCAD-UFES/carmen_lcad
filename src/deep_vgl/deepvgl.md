# DeepVGL

## Utilizando o módulo 

Para utilizar o módulo deep_vgl integrado ao carmen precisaremos dos seguintes arquivos:
* deepvgl.cfg               # as configurações da rede
* poses_and_labels.txt      # tabela relacionando poses  (x, y, yaw) com as imagens aprendidas durante o treino
* deepvgl_final.weights     # os pesos da rede, gerados durante a execução do treino da darknet (vários serão gerados e podem ser avaliados)

Todos esses arquivos foram gerados nas etapas de treinamento e de teste da rede.

Copie-os para a pasta $CARMEN_HOME/data/deep_vgl colocando o nome do mapa como prefixo. Para o exemplo mostrado em treino.md e teste.md,
a cópia seria como abaixo: 

./deep_vgl-test --poses_and_labels config/poses_and_labels.txt --weights_file $CARMEN_HOME/sharedlib/darknet4/backup/deepvgl_final.weights --config_file treino_e_teste/darknet_cfg/deepvgl.cfg  --images_list $CARMEN_HOME/src/deep_vgl/config/test-20210120.txt 


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
 ./deep_vgl /dados/ufes/deepvgl.cfg $CARMEN_HOME/sharedlib/darknet4/backup/deepvgl_final.weights config/poses_and_labels.txt 2 0 0 640 480 1 45 45 -camera_id 1
```

(Lembre-se de colocar o id da câmera utilizada no log, nesse exemplo é 1)

Acima, temos:
01- a configuração da rede;
02- são os pesos da rede;
03- são as poses e labels
04- crop_width;
05- crop_height;
06- width da imagem;
07- height da imagem;
08- use_lidar: 0 para não utilizar lidar e 1 para utilizar lidar
09- angle_left: um bom valor é 45, mas outras opções estão sendo estudadas;
10- angle_right: um bom valor é 45, mas outras opções estão sendo estudadas;
11- camera_id. Seguindo o mesmo raciocínio utilizado na etapa de treinamento.

O módulo dnn_visual_gl vai publicar mensagens gps_xyz e gps gphdt que permitirão localização global.
Use bin/gps_xyz-test se quiser examinar (só as informações essenciais à localização global são incluídas nas mensagens).

