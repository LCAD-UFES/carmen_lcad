Como fazer um grafo do route_planner a partir de um rddf:

1- Crie um arquivo com o nome do rddf dentro (suponha que o nome do rddf eh rddf-poc-rhi-20210709-percurso-2.txt):
 cat > rddf_file_name.txt
 /data/rndf/rddf-poc-rhi-20210709-percurso-2.txt
 crtl+d {digite control d}

2- Rode o programa road_network_generator:
 ./road_network_generator -c rddf_file_name.txt <graph_name.gr> <nearby_lanes_range_in_meters>
Na linha acima, <graph_name.gr> eh o nome do grafo gerado e <nearby_lanes_range_in_meters> eh a distancia a considerar no estabelecimento de nearby lanes no grafo.
Examplo de comando:
 ./road_network_generator -c rddf_file_name.txt ../data/graphs/graph-rddf-poc-rhi-20210709-percurso-2.gr 150.0

