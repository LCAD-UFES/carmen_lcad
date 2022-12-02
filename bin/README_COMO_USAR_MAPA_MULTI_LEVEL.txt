# COMO USAR MAPA MULTI LEVEL

este modo publica a mensagem CARMEN_MAPPER_MAP_MESSAGE_NAME como sendo CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME, podendo utilizar duas instâncias de mapas.
note que apenas o esta mensagem é publicada no mapper extra, as demais NÃO SÃO DUPLICADAS!


1- ajuste em carmen-*ini:

    mapper_height_max_level                 1
    mapper_safe_height_from_ground         -20.0    # altura a ser publicada na mensagem principal
    mapper_safe_height_from_ground_level1 	0.10    # altura a ser publicada na mensagem extra

todos os demais parâmetros relacionados ao mapping_mode podem ser sobrescritos opcionalmente no mapper extra:

    mapper_mapping_level1_update_and_merge_with_snapshot_map
    mapper_mapping_level1_global_map
    mapper_mapping_level1_merge_with_offline_map
    mapper_mapping_level1_decay_to_offline_map
    mapper_mapping_level1_update_and_merge_with_mapper_saved_maps
    mapper_mapping_level1_build_snapshot_map
    mapper_mapping_level1_velodyne_range_max        # imagino que não esteja sendo utilizado
    mapper_mapping_level1_velodyne_range_max_factor
    mapper_mapping_level1_create_map_sum_and_count
    mapper_mapping_level1_use_remission
    mapper_mapping_level1_laser_ldmrs
    mapper_mapping_level1_use_merge_between_maps


2- e no process*.ini defina o novo map_server e mapper com flag *-level_msg 1* para publicarem o mapa com a nova mensagem

    ./map_server -map_path ${MAP_NAME_N} -level_msg 1
    ./map_server -map_path ${MAP_NAME}

    ./mapper -map_path ${MAP_NAME} -mapping_mode off -calibration_file ${CALIBRATION_FILE}
    ./mapper -map_path ${MAP_NAME_N} -mapping_mode off -calibration_file ${CALIBRATION_FILE} -level_msg 1

depois é só subscrever os módulos com a nova mensagem:

    ./obstacle_distance_mapper -level_msg 1
    ./railway_anti_collision_system intelbras1 1 -graph ${GRAPH_PATH} -map_level 1



# COMO LIMPAR MAPA COM O ROBO

este modo limpa apenas os obstáculos do mapa embaixo do robô, com a geometria definida no arquivo de colisão *_col.txt.
veja que neste caso é necessário utilizar duas instâncias de mapas rodando, como neste exemplo do multi level. uma para a localização do robô, e outra para realizar a limpeza.
isto ocorre pois a limpeza é realizada descartando todos os raios do LiDAR, e limpando apenas as células dentro do raio de colisão do robô.


1. defina o arquivo *_col.txt com a geometria adequada


2. como a localização pode ser prejudicada neste modo, crie uma cópia do mapa que você deseja limpar e rode ele no movo multi level. veja acima.


3. no mapper level1, adicione a flag *-clean_bellow_car on* e *-mapping_mode on*

    ./mapper -map_path ${MAP_NAME_N} -mapping_mode on -calibration_file ${CALIBRATION_FILE} -level_msg 1 -clean_bellow_car on

o mapa limpo estará salvo na pasta ${MAP_NAME_N}.
