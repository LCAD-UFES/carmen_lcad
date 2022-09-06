Como fazer um grafo do route_planner a partir de um rddf:

1 - Rode o programa road_network_generator:
 ./road_network_generator --rddf <rddf_filename.txt> <graph_filename_gr> <nearby_lane_range_in_meters>

Na linha acima, <rddf_filename.txt> eh o caminho para o arquivo rddf, <graph_name.gr> eh o nome do grafo gerado e <nearby_lanes_range_in_meters> eh a distancia a considerar no estabelecimento de
nearby lanes no grafo

Examplo de comando:
 ./road_network_generator --rddf ../data/rndf/rddf.txt ../data/graphs/graph-rddf.gr 250.0

