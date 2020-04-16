O processo descrito abaixo funciona quando se deseja refazer um trecho de um mapa usando um novo log.
Para que ele funcione, o localizer tem que conseguir se localizar no mapa velho, pois o novo mapa,
produzido com o log, vai ser um merge do mapa antigo com o mapa gerado pelo mapper dinamicamente,
sendo que o mapper vai usar a pose do localizer, que localizara usando o mapa velho e sua versao 
atualizada a partir do log (merge). A versao atualizada eh salva a cada troca de mapas.

Se quer gerar um mapa novo a partir de 2 logs, existem outras formas como ex:
  README_COMO_FAZER_MERGE_DE_MAPAS_USANDO_O_HYPERGRAPHSCLAM.txt

1- Mude os parametros do carmen-ford-escape.ini para construção de mapas
 - Para isso, descomente essas linhas e comente as equivalentes que ficam acima no arquivo.
o parametro #mapper_use_remission_threshold deve ser off no caso que se deseja sobreescrever as informações do mapa anterior

## Use the parameters below for building maps
#mapper_update_and_merge_with_snapshot_map	off
#mapper_global_map 				off
#mapper_merge_with_offline_map 			off
#mapper_decay_to_offline_map			off
#mapper_update_and_merge_with_mapper_saved_maps	on
#mapper_build_snapshot_map			off
#mapper_velodyne_range_max		 	70.0
#mapper_velodyne_range_max_factor 		4.0
#mapper_create_map_sum_and_count		off
#mapper_use_remission				on
#mapper_laser_ldmrs 				off
#mapper_use_remission_threshold			off



2- Altere o caminho do mapa no process-volta_da_ufes_playback_viewer_3D.ini
 - Pode ser uma boa ideia criar um diretorio /dados/mapper_teste2 e fazer um link para ../data/mapper_teste2

	map_server		support		1		0			./map_server -map_path ../data/mapper_teste2 ...
	mapper			SLAM		1		0			./mapper -map_path ../data/mapper_teste2 ...
	navigator_gui		interface	1		0			./navigator_gui2 -map_path ../data/mapper_teste2 ...

3- Copie o mapa velho para a pasta ../data/mapper_teste2
 - Poder ser uma boa ideia copiar apenas os mapas m*.map (mapas usados na localizacao; os mapas de remission velho e novo sao acumulados
   estatisticamente).

4- Rode o process-volta_da_ufes_playback_viewer_3D.ini com o log novo com o qual se deseja fazer o merge
 - Pronto, vai ser feito o merge do log novo com o mapa antigo.

5- Copie o mapa atualizado para a pasta definitiva desejada.
