*** Edite o carmen_...ini

O parametro mapper_height_max_level indica a quantidade de niveis de altura criados pelo mapper.
mapper_height_max_level  O # Mapa de apenas 1 nivel
mapper_height_max_level  O # Mapa de apenas 2 niveis

O parametro mapper_safe_height_from_ground_levelX indica a altura segura a partir do chão para o nivelX, onde X vai de 1 ate mapper_height_max_level. 
mapper_safe_height_from_ground_level1 0.2 # Desconsidera obstaculas abaixo desta altura (0.2m) para o mapa de segundo nivel 

*** Edite o process_...ini

Adicione uma linha de execucao do mapper para cada nivel, o parametro -height_level indica nivel do mapa.
- O mapper com -height_level 0 ou sem este parametro cria o mapa de nivel 0, -height_level 1 mapa de nivel 1...
	mapper 		SLAM		1		0			./mapper -map_path ../data/mapper_teste2 -map_x 7756450 -map_y -364200
	mapper 		SLAM		1		0			./mapper -map_path ../data/mapper_teste2 -map_x 7756450 -map_y -364200 -height_level 1 

*** Edite o process_...ini
Prossiga com o processo de mapeamento normalmente.

TODO: Extender para mais de dois níveis
