Informações dos arquivos gerados pelo HYPERGRAPHSCLAM:



Como fazer o mapa com o HYPERGRAPHSCLAM

==============================================================================
0. Verifique se no seu computador tem as seguintes pastas, caso não tenha crie:

    ```
    mkdir /dados/tmp
    mkdir /dados/tmp/sick
    mkdir /dados/tmp/velodyne
    mkdir /dados/tmp/lgm
    mkdir /dados/tmp/lgm/sick
    mkdir /dados/tmp/lgm/velodyne
    mkdir /dados/tmp/images
    ```

Alem disso, apagar os arquivos antigos nessas pastas
==============================================================================

==============================================================================
1. Calibre a odometria:
https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/odometry_calibration

Os bias de saída da calibração devem ser colocados no arquivo src/hypergraphsclam/config/parser_config.txt em:

V_MULT_ODOMETRY_BIAS <primeiro parâmetro do bias v da saída da calibração>
PHI_MULT_ODOMETRY_BIAS <primeiro parâmetro do bias phi da saída da calibração>
PHI_ADD_ODOMETRY_BIAS <segundo parâmetro do bias phi da saída da calibração>
==============================================================================

2. Execute o parser do log (ele leh o log e constroi um hypergrafo) dentro da pasta src/hypergraphsclam/
Este processo demora pois tem que processar o log.

## OBS 1: O parser contém alguns parâmetros que podem ser ajustados no arquivo src/hypergraphsclam/config/parser_config.txt
## O arquivo contém comentários sobre do que se tratam os parâmetros.
## Temos, por exemplo, o parâmetro ICP_THREAD_POOL_SIZE que define
## a quantidade de threads disponíveis para as estimativas de movimento
## dos sensores do tipo LiDAR.
## O usuário pode alterar livremente o arquivo.

## OBS 2: Caso seja do seu interesse, utilize um terceiro argumento no parser
## para especificar outro arquivo de configuração. Caso não seja fornecido o terceiro parâmetro,
## o parser vai procurar pelo arquivo parser_config.txt na pasta do hypergraphsclam/config/

## OBS 3: As nuvens de pontos acumuladas pelo método de estimativa de movimentos de sensores do tipo LiDAR
## são salvas nas pastas /dados/tmp/lgm/velodyne e /dados/tmp/lgm/sick.
## Esses arquivos servem apenas para visualização posterior do acúmulo de nuvens.
## O parâmetro SAVE_ACCUMULATED_POINT_CLOUDS no arquivo src/hypergraphsclam/config/parser_config.txt é utilizado para
## habilitar esse recurso. Por padrão, essa opção está desabilitada e para habilitá-la descomente esse parâmetro.
## Tenha em mente que esse recurso consome muito espaço no disco pois as nuvens de pontos são mais densas.

    ```
    cd src/hypergraphsclam
    ./parser <seu log>.txt sync-<seu log>.txt
    ```
    exemplo:

    ```
    cd src/hypergraphsclam
    ./parser /dados/log_volta_da_ufes-20171106.txt sync-log_volta_da_ufes-20171106.txt
    ```

O parser gera 7 arquivos:
    1. as poses do GPS: gps.txt
    2. a odometria com calibração inicial: odometry.txt
    3. a odometria sem calibração: raw_odometry.txt
    4. poses do ICP do velodyne: velodyne.txt
    5. poses do ICP do sick: sick.txt
    6. poses da câmera: bumblebee.txt
    7. o arquivo de saída <output> contendo o hipergrafo.

Os arquivos acima estão no seguinte formato:

    1. x y theta cos(theta) sin(theta)
    2. x y theta cos(theta) sin(theta)
    3. x y theta cos(theta) sin(theta)
    4. x y theta cos(theta) sin(theta)
    5. x y theta cos(theta) sin(theta)
    6. x y theta cos(theta) sin(theta)
    7. VERTEX_ID x y theta timeStamp vertexType -> nó do grafo
        ...
       ODOM_EDGE VERTEX_ID VERTEX_ID dX dY dTheta velocityWithBias phiWithBias dt -> aresta de odometria
        ...
       GPS_EDGE VERTEX_ID x y theta GPSQuality -> aresta do GPS
        ...
       VELODYNE_SEQ VERTEX_ID VERTEX_ID dX dY dTheta  -> aresta de estimativa de movimento do Velodyne
        ...
       VELODYNE_LOOP VERTEX_ID VERTEX_ID dX dY dTheta -> aresta de estimativa de fechamento de loop do Velodyne
        ...
       SICK_SEQ VERTEX_ID VERTEX_ID dX dY dTheta  -> aresta de estimativa de movimento do sick
        ...
       SICK_LOOP VERTEX_ID VERTEX_ID dX dY dTheta -> aresta de estimativa de fechamento de loop do sick
        ...
       BUMBLEBEE_SEQ VERTEX_ID VERTEX_ID dX dY dTheta  -> aresta de estimativa de movimento da bumblebee
        ...
       BUMBLEBEE_LOOP VERTEX_ID VERTEX_ID dX dY dTheta -> aresta de estimativa de fechamento de loop da bumblebee


## OBS 3: o parser gera muitos arquivos nas pastas /dados/tmp/*.
## As nuvens de pontos do velodyne, por exemplo, estão na pasta /dados/tmp/velodyne.
## As nuvens de pontos do velodyne acumuladas no ICP estão na pasta /dados/tmp/lgm/velodyne.
## Portanto, é bom remover esses dados ao terminar de construir o mapa

3. Execute o hypergraphsclam dentro da pasta src/hypergraphsclam (voce pode rodar o hypergraphsclam varias vezes com diferentes parametros sem ter que rodar o parser novamente):

    ```
    cd src/hypergraphsclam
    ./hypergraphsclam sync-<seu log>.txt poses-opt-<seu log>
    ```

    Exemplo:

    ```
    cd src/hypergraphsclam
    ./hypergraphsclam sync-log_volta_da_ufes-20171106.txt poses-opt-log_volta_da_ufes-20171106
    ```

## OBS 1: Nos prints do programa hypergraphsclam, chi2 representa o erro do graphslam.
## OBS 2: O otimizador também utiliza um arquivo de configuração que está na pasta hypergraphsclam/config/, intitulado optimization_config.txt
## Altere os parâmetros no arquivo para ajustar a otimizacao conforme os parametros de seus sensores, etc.
## Se preferir, utilize um terceiro argumento para especificar um arquivo de configuração.
## O arquivo optimization_config.txt contém os parâmetros de covariâncias de cada tipo de aresta, bem como especificação de quantas iterações
## o algoritmo de otimização deve empregar.
## O arquivo ainda permite habilitar/desabilitar aretas do hipergrafo (exemplo, nao usar GPS, etc.).

    Exemplo com arquivo de configuração próprio:

    ```
    ./hypergraphsclam sync-log_volta_da_ufes-20171106.txt poses-opt-log_volta_da_ufes-20171106.txt minha_configuração.txt
    ```
O otimizador gera 4 arquivos:
    1. Poses do carro no mundo
    2. Poses do velodyne no mundo
    3. Poses da bumblebee no mundo
    4. Poses do sick no mundo

Os quatro arquivos acima estão no seguinte formato:
    x y theta timeStamp cos(theta) sin(theta)

Os dados de "poses-opt-log_volta_da_ufes-20171106.txt" podem estar fora de ordem (timestamp). Assim, 
reoordene com sort (troque abaixo para o nome de seu log)
    sort -k 4 poses-opt-log_volta_da_ufes-20171106.txt > caco.txt
    mv caco.txt poses-opt-log_volta_da_ufes-20171106.txt

Após a otimização, os dados podem ser visualizados pelo gnuplot:
    Exemplo:
    ```
    gnuplot
    plot '<gps.txt ou odom.txt ou velodyne.txt ou poses> u 1:2 w l
    replot '<gps.txt ou odom.txt ou velodyne.txt ou poses>' u 1:2 w l
    ```

Talvez o aquivo de poses não esteja ordenado pelo timestamp após rodar a otimização. Para ordenar pelo timestamp execute um `sort` para a 4ª coluna. Exemplo:
    ```
    sort -k4 poses-opt-log_dante_michelini-20181116.txt > sorted-poses-opt-log_dante_michelini-20181116.txt
    ```

4. Modifique no process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini as saidas dos programas playback, rdd_build, graphslam_publish:
 playback 		support 	1		0			./playback <seu log>.txt
 rndf_build		interface	1		0			./rddf_build ../data/rndf/rddf_<seu log>.txt
 publish_poses		graphslam	0		0   			./graphslam_publish poses-opt-<seu log>.txt

5. As seguintes variaveis do carmen-ford-escape.ini serao utilizadas na criacao de mapas:

## The parameters below are used with mapper -mapping_mode options
mapper_mapping_mode_on_update_and_merge_with_snapshot_map	off
mapper_mapping_mode_on_global_map 				off
mapper_mapping_mode_on_merge_with_offline_map 			off
mapper_mapping_mode_on_decay_to_offline_map			off
mapper_mapping_mode_on_update_and_merge_with_mapper_saved_maps	on
mapper_mapping_mode_on_build_snapshot_map			off
mapper_mapping_mode_on_velodyne_range_max		 	70.0
mapper_mapping_mode_on_velodyne_range_max_factor 		4.0
mapper_mapping_mode_on_create_map_sum_and_count			off
mapper_mapping_mode_on_use_remission				on
mapper_mapping_mode_on_laser_ldmrs 				off


6. Se existir a pasta data/mapper_teste2 limpe-a com o comando "rm -rf $CARMEN_HOME/data/mapper_teste2/*".
Caso a pasta não exista, utilize o comando "mkdir $CARMEN_HOME/data/mapper_teste2" para criá-la.

7. Execute o ./central e o process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini

8. Execute no proccontrol o publish_poses

9. Modifique o speed do playback para 0.3 e aperte Enter para confirmar.

10. Acione o play no playback e espere terminar de tocar o log.

## OBS: selecionar a opção "Map" no menu "Maps" na janela "Carmen-LCAD Navigator" (Ou aperte ctrl+m)

11. Ao final do log seu mapa está pronto em ../data/mapper_teste2/ !!!! Pode matar o proccontrol no terminal e copiar seu novo mapa para seu lugar definitivo.

12. Volte os parametros de construir mapa do carmen-ford-escape.ini

==================================================================================================================================================
Para limpar o mapa use os programas
	bin/build_complete_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -map_type m
	bin/map_editor <path do diretorio do mapa>/complete_map.map
	bin/complete_map_to_block_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -block_size_in_meters 210.0
	Exclua o complete_map.map após finalizar antes de subir para o git

