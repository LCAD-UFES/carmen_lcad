O processo descrito abaixo funciona quando se deseja ampliar/sobrescrever um trecho de um mapa usando um novo log.
Para que ele funcione, o localizer tem que conseguir se localizar no mapa velho; pois o novo mapa,
produzido com o log, vai ser um merge do mapa antigo com o mapa gerado pelo mapper dinamicamente,
sendo que o mapper vai usar a pose do localizer, que localizara usando o mapa velho e sua versao 
atualizada a partir do log (merge). A versao atualizada eh salva a cada troca de mapas.

######################
# Se quer gerar um mapa novo a partir de 2 logs, existem outras formas como, por exemplo, README_COMO_FAZER_MAPAS_USANDO_O_HYPERGRAPHSCLAM.txt
# Esse processo otimiza as poses dos dois logs juntas para que o mapa se encaixe o mais corretamente possível.
#######################

1- Verifique os parametros do carmen ini do log que sera usado para ampliar/refazer o mapa de interesse
   (especificado no process process-volta_da_ufes_playback_viewer_3D_map_generation_with_localize.ini).

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
mapper_mapping_mode_on_use_merge_between_maps 			on

 1.1 O parametro mapper_mapping_mode_on_use_merge_between_maps deve estar on, ele é o responsável por fazer o merge de
     forma inteligente sem apagar informações importantes do mapa de interesse de um log para outro. 
     Ela deve ser off nos casos em que se deseja sobreescrever as informações do mapa.
 1.2 O parametro mapper_rays_threshold_to_merge_between_maps deve ser ajustado para melhor merge dos mapas. 
     Ele define o numero de raios que bateram em uma célula para saber se ela deve ser atualizada, é adhoc.

2- Copie o mapa a ser ampliado/sobrescrito para a pasta ../data/mapper_teste2
 rm ../data/mapper_teste2/*
 cp ../data/map_voltadaufes-201903025-2/* ../data/mapper_teste2/

 - Note que pode ser uma boa ideia copiar apenas os mapas m*.map (mapas usados na localizacao; os mapas de remission velho e novo sao acumulados
   estatisticamente).
 
3- Escolha o carmen ini correpondente ao log no process-volta_da_ufes_playback_viewer_3D_map_generation_with_localize.ini

4- Rode o process-volta_da_ufes_playback_viewer_3D_map_generation_with_localize.ini com o log novo com o qual se deseja fazer o merge.
 4.1 Toque o log ate o ponto onde deseja iniciar o merge, certifique-se de que tem boa localizacao e pare. 
 4.2 Desligue o mapper (botao mapper)
 4.3 Desligue o map_server
 4.4 Ligue o map_server
 4.5 Ligue o mapper em mapping mode (botao mapping)
 4.6 Toque o log na area de interesse observando a localizacao

Pronto, vai ser feito o merge do log novo com o mapa antigo.
Copie o novo mapa ampliado/sobrescrito para o local apropriado:
 cp ../data/mapper_teste2/* ../data/map_voltadaufes-201903025-2/

  OBS: NO CASO DA LOCALIZAÇÃO SE PERDER NO MEIO DO PROCESSO DE MAPEAMENTO, VOLTE AO PASSO 1  E:
   DE TEMPOS EM TEMPOS DÊ PAUSE NO PLAYBACK, MATE O MAP_SERVER E O MAPPER, RODE-OS NOVAMENTE E DÊ PLAY NO PLAYBACK. ISSO VAI GARANTIR QUE
   HAJAM FEATURES NOVAS RECENTEMENTE CRIADAS CARREGADAS EM MEMÓRIA.

5- Copie o mapa atualizado para a pasta definitiva desejada.


-------OLD-----
1- Ative o modo de criação de mapas passando a flag -mapping_mode on no mapper.
  
  ./mapper -map_path ../data/mapper_teste2 -mapping_mode on ...

Verifique os parametros do carmen-ford-escape.ini para construção de mapas.

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
mapper_mapping_mode_on_use_merge_between_maps 			on

 1.1 O parametro mapper_mapping_mode_on_use_merge_between_maps deve estar on, ele é o responsável por fazer o merge de
     forma inteligente sem apagar informações importantes de um log para outro. 
     Ela deve ser off no caso que se deseja sobreescrever as informações do mapa anterior.
 1.2 O parametro mapper_rays_threshold_to_merge_between_maps deve ser ajustado para melhor merge dos mapas. 
     Ele define o numero de raios que bateram em uma célula para saber se ela deve ser atualizada, é adhoc.

2- Altere o caminho do mapa no process-volta_da_ufes_playback_viewer_3D_map_generation_with_localize.ini e dê um nome para o seu rddf no rndf_build
 - Pode ser uma boa ideia criar um diretorio /dados/mapper_teste2 e fazer um link para ../data/mapper_teste2

	map_server		support		1		0			./map_server -map_path ../data/mapper_teste2 ...
	mapper			SLAM		1		0			./mapper -map_path ../data/mapper_teste2 -mapping_mode on ...
	navigator_gui		interface	1		0			./navigator_gui2 -map_path ../data/mapper_teste2 ...

3- Copie o mapa velho para a pasta ../data/mapper_teste2
 - Poder ser uma boa ideia copiar apenas os mapas m*.map (mapas usados na localizacao; os mapas de remission velho e novo sao acumulados
   estatisticamente).

4- Rode o process-volta_da_ufes_playback_viewer_3D.ini com o log novo com o qual se deseja fazer o merge
 - Pronto, vai ser feito o merge do log novo com o mapa antigo.
  OBS: NO CASO DA LOCALIZAÇÃO SE PERDER NO MEIO DO PROCESSO DE MAPEAMENTO, VOLTE AO PASSO 1. AO INVÊS DE RODAR O MAP_SERVER E O MAPPER PELO PROCCONTROL, RODE VIA
  LINHA DE COMANDO E DE TEMPOS EM TEMPOS DÊ PAUSE NO PLAYBACK, MATE O MAP_SERVER E O MAPPER, RODE-OS NOVAMENTE E DÊ PLAY NO PLAYBACK. ISSO VAI GARANTIR QUE
  HAJAM FEATURES NOVAS RECENTEMENTE CRIADAS CARREGADAS EM MEMÓRIA.

5- Copie o mapa atualizado para a pasta definitiva desejada.


-------VERY OLD-----
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
