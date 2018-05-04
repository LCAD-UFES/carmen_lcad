### Generating input dataset
####Using python code
This code generates the input dataset for neural mapper training.
Como usar:
1 - Criar meta-data a partir de um playback qualquer:
- Usando o codigo do mapper em anexo, chamar o mapper:

./mapper -map_path ../data/map_ida_guarapari-20170403-2 -map_x 7756450 -map_y -364200 -generate_neural_mapper_dataset on -neural_mapper_max_distance_meters 60 -neural_mapper_data_pace 60

-> Parametros:
-generate_neural_mapper_dataset => flag de geracao do dataset (on/off)

-neural_mapper_max_distance_meters => raio de leitura maxima dos lasers (em metros)

-neural_mapper_data_pace => numero 'n' inteiro (a cada n timestamps e retira uma amostra para o banco de dados)


Obs. 1: Os outros paramatros sao normais do mapper

2 - usar o cogido parse_neural_mapper_metadata.py para gerar o dado processado:
 -> primeiro, alterar a variavel path para a pasta onde se encontra a banco o metadata gerado em 1;
 -> colocar na variavel outpath o caminho onde deseja que o banco de dados processado seja gerado
 -> alterar variavel radius para o raio em metros escolhido quando se gera o metadado
 -> rodar com: python parse_neural_mapper_metadata.py
 
The input dataset are six statistics from velodyne point cloud. Each statistic is a csv file:
	number of points
	max high
	min high
	mean high
	std high
	bean intensity
Each cell in the csv file, correspond to the value inside the cell with resolution R and Width(W) x High(H) defined in meters by command line parameter
The total resolution is WxHxR

#### Deprecated dataset generation (deactivated)
Running
	create the folder where the dataset will be saved
		mkdir /dados/neural_mapper_dataset/
	compile the code:
		cd $CARMEN_HOME/src/neural_mapper/generate_gt/
		make
	Run any process that publish velodyne data
	Run the neural_mapper_dataset_generator
	./neural_mapper_dataset_generator W H R Skip_clouds
	 
