
### GERAR IMAGEM .PNG DO MAPA COMPLETO ###

./save_map_images  -input_dir <map>  -out_dir <map_img>

EXEMPLO:
./save_map_images  -input_dir ../maper_test2 -out_dir ../maper_test2



### EDITE O GRAFO NO INKSCAPE ###

./edit_graph_inkscape.py <rddf> -i <input_dir> -o <output_dir>

EXEMPLO:
./edit_graph_inkscape.py actros4844_mosaic/rddf-map-mosaic-20220811-2-volta-1.txt -i actros4844_mosaic/map-mosaic-20220811-2-volta/ -o actros4844_mosaic/map-mosaic-20220811-2-volta/

>>> Apos a edição, salve e feche o inkscape. O programa edit_graph_inkscape exibira o comando que converte o rddf em grafo

GRAPH="actros4844_mosaic/map-mosaic-20220811-2-volta//grafo-rddf_rota_estacionamento_teatro-2-1.gr";  RANGE="150";  echo -e '/home/lcad/carmen_lcad/bin/actros4844_mosaic/map-mosaic-20220811-2-volta/rddf-map-mosaic-20220811-2-volta-1.txt' > rddf_files_list.txt;  $CARMEN_HOME/bin/road_network_generator  -c  rddf_files_list.txt  $GRAPH  $RANG

GRAPH="actros4844_mosaic/map-mosaic-20220811-2-volta//grafo-rddf_rota_estacionamento_teatro-2-1.gr";  RANGE="150";  echo -e '/home/lcad/carmen_lcad/bin/actros4844_mosaic/map-mosaic-20220811-2-volta/rddf-map-mosaic-20220811-2-volta-1.txt' > rddf_files_list.txt;  $CARMEN_HOME/bin/road_network_generator  -c  rddf_files_list.txt  $GRAPH  $RANG



### INSTALAR O INKSCAPE ###

sudo apt install inkscape

