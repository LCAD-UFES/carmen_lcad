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

Alem disso, apagar os arquivos antigos nessas pastas
==============================================================================
==============================================================================


1. Calibre a a odometria, Ver em ../src/odometry_calibration/README.txt

2. Modificar o valor da odometria calibrada no Messages/StampedOdometry.cpp nas linhas com o valor da calibracao da etapa anterior:

    v *= 0.976350;
    phi = phi * 0.50000 - 0.001324;    

3. compile o codigo do hypergraphsclam

4. Modifique no process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini  as saidas dos programas:
	 playback, rdd_build, graphslam_publish


     playback 		support 	1		0			./playback <seu log>

	exemplo:
	    ./playback /dados/log_aeroporto_vila_velha_20170726-2.txt
	

    rndf_build		interface	1		0			./rddf_build ../data/rndf/rndf_<nome_do_log>.txt
	
	exemplo:
	    ./rddf_build ../data/rndf/rndf-aeroporto_vila_velha_20170726.txt


     Publish_poses		graphslam	0		0   ./graphslam_publish <output do ./hypergraphsclam - ver na etapa 7>

        exemplo:
            ./graphslam_publish  ../src/hypergraphsclam/poses-log_aeroporto_vila_velha_20170726.txt


5. Modifique o carmen-ford-escape.ini para ativar a criacao de mapas, comente o trecho dos parâmetros acima e descomente o de baixo.

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

6. Execute o parser no log dentro da pasta src/hypergraphsclam:

    ./parser <log> <output>
    
    exemplo:
        ./parser log_aeroporto_vila_velha_20170726.txt sync-log_aeroporto_vila_velha_20170726.txt

O parser gera 3 arquivos de poses do gps(gps.txt), odometria(odom.txt) e do icp do velodyne(velodyne.txt)

##OBS: Caso der erro nessa etapa, eh preciso apagar os arquivos no /dados/tmp/

7. Execute o hypergraphsclam dentro da pasta src/hypergraphsclam:

    ./hypergraphsclam <output do ./parser> <poses-nome_do_log>

    exemplo:
        ./hypergraphsclam ../data/graphslam/sync-log_aeroporto_vila_velha_20170726.txt ../data/graphslam/poses-log_aeroporto_vila_velha_20170726.txt

##OBS: chi2 representa o erro do graphslam.

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

