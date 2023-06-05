
### GERAR IMAGEM .PNG DO MAPA COMPLETO ###

./save_map_images  -input_dir <map>  -out_dir <map_img>

EXEMPLO:
./save_map_images  -input_dir ../maper_test2 -out_dir ../maper_test2



### EDITE O GRAFO NO INKSCAPE ###

./edit_graph_inkscape.py <rddf> -i <input_dir> -o <output_dir>

EXEMPLOS:
./edit_graph_inkscape.py ../data/maper_test2/rddf.txt -i ../data/maper_test2/ -o ../data/maper_test2/
./edit_graph_inkscape.py  -i ../data/mapper_teste2  -o ../data/mapper_teste2 -d 0.1 -s 0.04 --window  980.0  980.0  40.0  40.0  '../data/rndf/rddf-log_ct13-20220531.txt'

>>> Apos a edição, salve e feche o inkscape;   O programa edit_graph_inkscape exibira o comando que converte o rddf em grafo;   Voce pode editar os nomes e caminhos do comando de acordo com a conveniencia.

GRAPH="../data/maper_test2/graf-rff.gr";  RANGE="150";  echo -e '/home/lcad/carmen_lcad/data/maper_test2/rddf.txt' > rddf_files_list.txt;  $CARMEN_HOME/bin/road_network_generator  -c  rddf_files_list.txt  $GRAPH  $RANG



### INSTALAR O INKSCAPE ###

sudo apt install inkscape

