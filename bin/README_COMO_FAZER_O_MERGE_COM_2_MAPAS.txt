
1- mude os parametros do carmen-ford-escape.ini para construção dos mapas
-Descomente essas linhas e comente as mesmas acima.

## Use the parameters below for building maps
#mapper_update_and_merge_with_snapshot_map	off
#mapper_global_map 				off
#mapper_merge_with_offline_map 			off
#mapper_decay_to_offline_map			off
#mapper_update_and_merge_with_mapper_saved_maps	on
#mapper_build_snapshot_map			off
#mapper_velodyne_range_max		 	50.0
#mapper_velodyne_range_max_factor 		4.0
#mapper_create_map_sum_and_count		off


2- Altere o caminho do mapa no process-volta_da_ufes_playback_viewer_3D.ini

	map_server		support		1		0			./map_server -map_path ../data/mapper_teste2 ...
	mapper			SLAM		1		0			./mapper -map_path ../data/mapper_teste2 ...
	navigator_gui		interface	1		0			./navigator_gui2 -map_path ../data/mapper_teste2 ...

3- copie o mapa que queria juntar para a pasta ../data/mapper_teste2

4- Rode o process-volta_da_ufes_playback_viewer_3D.ini com o log que queria fazer o merge

Pronto, vai ser feito o merge do log novo com o mapa antigo.
	
