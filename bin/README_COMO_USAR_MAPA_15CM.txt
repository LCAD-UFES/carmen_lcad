#######################
COMO USAR MAPA DE 15CM
#######################

#OBS: VERIFICAR A RESOLUÇÃO ANTES DE FAZER O COMMIT PARA O GITHUB. VOLTE PARA A RESOLUÇÃO PADRÃO USADA POR TODOS.

1- Modifique no carmen-ford-escape.ini a resolução do mapa nos seguintes parametros para 0.15

	grid_mapping_map_grid_res 		0.15
	mapper_map_grid_res 			0.15
	map_server_map_grid_res			0.15

#OBS: NA CONSTRUÇÃO DO MAPA O PARAMETRO "map_server_map_grid_res" DEVE ESTAR NA RESOLUÇÃO DO MAPA 
      USADO NO COMANDO ./map_server DO process-xxx.ini. 
      			

2- Modifique no process-xxx.ini o caminho do mapa que queria executar

	../data/map_voltadaufes-xxxxxxxx por ../data/map_voltadaufes-xxxxxxxx-15cm  


