Como fazer o mapa com o HYPERGRAPHSCLAM

==============================================================================
Verifique se no seu computador tem as seguintes pastas, caso não tenha criei:

    mkdir $CARMEN_HOME/data/mapper_teste2
    mkdir /dados/sick
    mkdir /dados/velodyne
    mkdir /dados/tmp/
    mkdir /dados/tmp/lgm
    mkdir /dados/tmp/lgm/sick
    mkdir /dados/tmp/lgm/velodyne    
==============================================================================

1. Calibre a a odometria, Ver em ../src/odometry_calibration/README.txt

2. Modificar o valor da odometria calibrada no Messages/StampedOdometry.cpp nas linhas:

    v *= 0.976350;
    phi = phi * 0.50000 - 0.001324;    

3. compile o codigo

4. Execute o programa:
    
    ./parser <log> <output>
    
    exemplo:
        ./parser log_guarapari-20170403-2.txt sync.txt
        
5. Execute o programa:

     ./hypergraphsclam <output do ./parser> <output>
     
     exemplo:
     ./hypergraphsclam sync.txt poses.txt
     
6. Apague a mapper_teste2 caso tenha algum mapa antigo
    rm -r ../data/mapper_teste2/*

7. Execute o central

8. Modifique o process-volta_da_ufes_playback_viewer_3D_map_generation.ini para que ele faça playback de seu log:
		./playback [seu log]

9. Modifique o carmen-ford-escape.ini para ativar a criacao de mapas, comente o trecho dos parâmetros acima e descomente o de baixo.

## Use the parameters below for building maps
mapper_update_and_merge_with_snapshot_map	off
mapper_global_map 							off
mapper_merge_with_offline_map 				off
mapper_decay_to_offline_map				off
mapper_update_and_merge_with_mapper_saved_maps	on
mapper_build_snapshot_map					off
mapper_velodyne_range_max		 			70.0
mapper_velodyne_range_max_factor 			4.0
mapper_create_map_sum_and_count			off
mapper_use_remission						on
mapper_laser_ldmrs 						off


10. iniciar o ./proccontrol process-volta_da_ufes_playback_viewer_3D_map_generation.ini

14. Execute o programa com a poses otimizadas do hypergraphsclam "./graphslam_publish poses.txt"

15. Faca o playback com velocidade 0.3 e espere terminar de tocar o log.

16. Ao final do log seu mapa está pronto em ../data/mapper_teste2/ !!!! Pode matar o proccontrol e copiar seu novo mapa para seu lugar definitivo.

==================================================================================================================================================
OBS:
=====
O process já constroe o rndf(rddf) usando o ./rddf_build ../data/rndf/rndf.kml
	Após criar o mapa, renomear o arquivo ../data/rndf/rndf.kml para ../data/rndf/rddf-log_voltadaufes-<data do log>.kml
		mv ../data/rndf/rndf.kml ../data/rndf/rddf-log_voltadaufes-<data do log>.kml
		
==================================================================================================================================================		
Para limpar o mapa use os programas
	bin/build_complete_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -map_type m
	bin/map_editor <path do diretorio do mapa>/complete_map.map
	bin/complete_map_to_block_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -block_size_in_meters 210.0
	Exclua o complete_map.map após finalizar antes de subir para o git

