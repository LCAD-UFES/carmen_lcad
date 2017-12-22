Como fazer o mapa com o HYPERGRAPHSCLAM

==============================================================================
Verifique se no seu computador tem as seguintes pastas, caso não tenha crie:

    mkdir $CARMEN_HOME/data/mapper_teste2
    mkdir /dados/sick
    mkdir /dados/velodyne
    mkdir /dados/tmp/
    mkdir /dados/tmp/lgm
    mkdir /dados/tmp/lgm/sick
    mkdir /dados/tmp/lgm/velodyne
    mkdir /dados/tmp/images

Alem disso, apagar os arquivos antigos nessas pastas
==============================================================================
==============================================================================


1. Modifique no process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini as saidas dos programas playback, rdd_build, graphslam_publish:
 playback 		support 	1		0			./playback <seu log>
 rndf_build		interface	1		0			./rddf_build ../data/rndf/rndf_<nome_do_log>.txt
 publish_poses		graphslam	0		0   			./graphslam_publish <output do ./hypergraphsclam - ver na etapa 7>

2. Modifique o carmen-ford-escape.ini para ativar a criacao de mapas comentando e descomentando o trecho abaixo (ver o carmen-ford-escape.ini).

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

3. Execute o parser no log dentro da pasta src/hypergraphsclam/:

    ./parser <log> <output>

    exemplo:

    ./parser log_aeroporto_vila_velha_20170726.txt sync-log_aeroporto_vila_velha_20170726.txt

O parser gera 7 arquivos:
    as poses do GPS: gps.txt
    a odometria com calibração inicial: odometry.txt
    a odometria sem calibração: raw_odometry.txt
    poses do ICP do velodyne: velodyne.txt
    poses do ICP do sick: sick.txt
    poses da câmera: bumblebee.txt
    o arquivo de saída <output> contendo o hipergrafo.

## OBS 1: o parser contém alguns parâmetros que podem ser ajustados no arquivo parser_config.txt
## o arquivo contém comentários sobre do que se tratam os parâmetros
## temos como exemplo, o parâmetro ICP_THREAD_POOL_SIZE que define a quantidade de threads disponíveis para as estimativas de movimento
## dos sensores do tipo LiDAR.

## OBS 2: caso seja do interesse do usuário, utilize um terceiro argumento no parser para especificar outro arquivo de configuração.
## Caso não seja fornecido o terceiro parâmetro, então o parser vai procurar pelo arquivo parser_config.txt na pasta do hypergraphsclam/config/

    ./parser <log> <output> <config_file>

    Exemplo:

        ./parser log_aeroporto_vila_velha_20170726.txt sync-log_aeroporto_vila_velha_20170726.txt outro_parser_config.txt

## OBS 2: o parser gera muitos arquivos nas pastas /dados/tmp/*. As nuvens de pontos do velodyne, por exemplo, estão na pasta /dados/tmp/velodyne. As nuvens de pontos do velodyne acumuladas no ICP estão na pasta /dados/tmp/lgm/velodyne.
## Portanto, é bom remover esses dados ao terminar de construir o mapa

7. Execute o hypergraphsclam dentro da pasta src/hypergraphsclam:

    ./hypergraphsclam <output do ./parser> <poses-nome_do_log>

    exemplo:
        ./hypergraphsclam sync-log_aeroporto_vila_velha_20170726.txt poses-log_aeroporto_vila_velha_20170726.txt

##OBS 1: chi2 representa o erro do graphslam.
##OBS 2: o

para visualizar os dados use o gnuplot.
	exemplo:
	$ gnuplot
	plot '<gps.txt ou odom.txt ou velodyne.txt ou poses> u 1:2 w l
	replot '<gps.txt ou odom.txt ou velodyne.txt ou poses>' u 1:2 w l

8. Execute o ./central e o process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini

9. Execute no proccontrol CleanMap

10. Execute no proccontrol o publish_poses

11. Modifique o speed do playback para 0.3 e aperte Enter para confirmar.

12. Der o o play no playback e espere terminar de tocar o log.

##OBS: selecionar map na aba map do carmen navegatior

13. Ao final do log seu mapa está pronto em ../data/mapper_teste2/ !!!! Pode matar o proccontrol no terminal e copiar seu novo mapa para seu lugar definitivo.


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

