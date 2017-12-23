Como fazer o mapa com o HYPERGRAPHSCLAM

==============================================================================
0. Verifique se no seu computador tem as seguintes pastas, caso não tenha crie:

    mkdir /dados/tmp
    mkdir /dados/tmp/sick
    mkdir /dados/tmp/velodyne
    mkdir /dados/tmp/lgm
    mkdir /dados/tmp/lgm/sick
    mkdir /dados/tmp/lgm/velodyne
    mkdir /dados/tmp/images

Alem disso, apagar os arquivos antigos nessas pastas
==============================================================================
==============================================================================

1. Execute o parser do log (ele leh o log e constroi um hypergrafo) dentro da pasta src/hypergraphsclam/
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

## OBS 3: As nuvens de pontos ...

    cd src/hypergraphsclam
    ./parser <seu log>.txt sync-<seu log>.txt

    exemplo:

    cd src/hypergraphsclam
    ./parser /dados/log_volta_da_ufes-20171106.txt sync-log_volta_da_ufes-20171106.txt

O parser gera 7 arquivos:
    as poses do GPS: gps.txt
    a odometria com calibração inicial: odometry.txt
    a odometria sem calibração: raw_odometry.txt
    poses do ICP do velodyne: velodyne.txt
    poses do ICP do sick: sick.txt
    poses da câmera: bumblebee.txt
    o arquivo de saída <output> contendo o hipergrafo.

## OBS 3: o parser gera muitos arquivos nas pastas /dados/tmp/*.
## As nuvens de pontos do velodyne, por exemplo, estão na pasta /dados/tmp/velodyne.
## As nuvens de pontos do velodyne acumuladas no ICP estão na pasta /dados/tmp/lgm/velodyne.
## Portanto, é bom remover esses dados ao terminar de construir o mapa

2. Execute o hypergraphsclam dentro da pasta src/hypergraphsclam (voce pode rodar o hypergraphsclam varias vezes com diferentes parametros
sem ter que rodar o parser novamente):

    cd src/hypergraphsclam
    ./hypergraphsclam sync-<seu log>.txt poses-opt-<seu log>.txt

    Exemplo:
    
    cd src/hypergraphsclam
    ./hypergraphsclam sync-log_volta_da_ufes-20171106.txt poses-opt-log_volta_da_ufes-20171106.txt

## OBS 1: Nos prints do programa hypergraphsclam, chi2 representa o erro do graphslam.
## OBS 2: O otimizador também utiliza um arquivo de configuração que está na pasta hypergraphsclam/config/, intitulado optimization_config.txt
## Altere os parâmetros no arquivo para ajustar a otimizacao conforme os parametros de seus sensores, etc.
## Se preferir, utilize um terceiro argumento para especificar um arquivo de configuração.
## O arquivo optimization_config.txt contém os parâmetros de covariâncias de cada tipo de aresta, bem como especificação de quantas iterações
## o algoritmo de otimização deve empregar.
## O arquivo ainda permite habilitar/desabilitar aretas do hipergrafo (exemplo, nao usar GPS, etc.).

    Exemplo com arquivo de configuração próprio:

    ./hypergraphsclam sync-log_volta_da_ufes-20171106.txt poses-opt-log_volta_da_ufes-20171106.txt minha_configuração.txt

Após a otimização, os dados podem ser visualizados pelo gnuplot:
    Exemplo:
    gnuplot
    plot '<gps.txt ou odom.txt ou velodyne.txt ou poses> u 1:2 w l
    replot '<gps.txt ou odom.txt ou velodyne.txt ou poses>' u 1:2 w l

3. Modifique no process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini as saidas dos programas playback, rdd_build, graphslam_publish:
 playback 		support 	1		0			./playback <seu log>.txt
 rndf_build		interface	1		0			./rddf_build ../data/rndf/rndf_<seu log>.txt
 publish_poses		graphslam	0		0   			./graphslam_publish poses-opt-<seu log>.txt

4. Modifique o carmen-ford-escape.ini para ativar a criacao de mapas comentando e descomentando o trecho abaixo (ver o carmen-ford-escape.ini).

## Use the parameters below for building maps
mapper_update_and_merge_with_snapshot_map		off
mapper_global_map 					off
mapper_merge_with_offline_map 				off
mapper_decay_to_offline_map				off
mapper_update_and_merge_with_mapper_saved_maps		on
mapper_build_snapshot_map				off
mapper_velodyne_range_max		 		70.0
mapper_velodyne_range_max_factor 			4.0
mapper_create_map_sum_and_count				off
mapper_use_remission					on
mapper_laser_ldmrs 					off

5. Execute o ./central e o process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini

6. Execute no proccontrol CleanMap

7. Execute no proccontrol o publish_poses

8. Modifique o speed do playback para 0.3 e aperte Enter para confirmar.

9. Acione o play no playback e espere terminar de tocar o log.

##OBS: selecionar map na aba map do carmen navegatior

10. Ao final do log seu mapa está pronto em ../data/mapper_teste2/ !!!! Pode matar o proccontrol no terminal e copiar seu novo mapa para seu lugar definitivo.


==================================================================================================================================================
##OBS:
======
O process já constroe o rndf(rddf) usando o ./rddf_build ../data/rndf/rndf.kml
	Após criar o mapa, renomear o arquivo ../data/rndf/rndf.kml para ../data/rndf/rddf-<nome do log>.txt
		mv ../data/rndf/rndf.kml ../data/rndf/rddf-<nome do log>.txt

==================================================================================================================================================
Para limpar o mapa use os programas
	bin/build_complete_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -map_type m
	bin/map_editor <path do diretorio do mapa>/complete_map.map
	bin/complete_map_to_block_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -block_size_in_meters 210.0
	Exclua o complete_map.map após finalizar antes de subir para o git

