No carmen_...ini o parametro 
	mapper_height_max_level 
Indica a quantidade de niveis de altura criados pelo mapper. O parametro:
	mapper_safe_height_from_ground_levelX
Indica a altura segura a partir do chão para o nivelX, onde X vai de 1 ate mapper_height_max_level. Ou seja objetos abaixo do valor dado por este parametro serão desconsiderados no mapa de levelX. 

No process_...ini copie copie a linha contendo a inicialização do mapper, cole a baixo adicionando o parametro -height_level que sera criado por este mapper.
Exemplo:
	mapper 		SLAM		1		0			./mapper -map_path ../data/mapper_teste2 -map_x 7756450 -map_y -364200
	mapper 		SLAM		1		0			./mapper -map_path ../data/mapper_teste2 -map_x 7756450 -map_y -364200 -height_level 1

NOTA: O mapper com -height_level 0 ou sem este parametro cria o mapa de nivel0, ou seja considerando todos os objetos  
TODO: Extender para mais de dois níveis