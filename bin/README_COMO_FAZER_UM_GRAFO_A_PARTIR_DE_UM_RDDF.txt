Como fazer um grafo do route_planner a partir de um rddf:

1- Crie um arquivo com o nome do rddf dentro (suponha que o nome do rddf eh rddf-log_portocel-20210617-3.txt):
 cat > rddf_file_name.txt
 /data/rndf/rddf-log_portocel-20210617-3.txt
 crtl+d {digite control d}

2- Rode o programa road_network_generator:
 ./road_network_generator -c rddf_file_name.txt <graph_name.gr> <nearby_lanes_range_in_meters> <offline_map_dir> <-process_crossroad 0 or 1 (default is 0)>
Na linha acima, <graph_name.gr> eh o nome do grafo gerado e <nearby_lanes_range_in_meters> eh a distancia a considerar no estabelecimento de nearby lanes no grafo,
e <offline_map_dir> eh o diretorio do mapa.

Examplo de comando:
 ./road_network_generator -c rddf_file_name.txt ../data/graphs/graph-rddf-log_portocel-20210617-3.gr 250.0 /data/map_portocel-20210617-3

