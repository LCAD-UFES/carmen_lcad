SE JA TIVER FEIITO ISSO ANTES SEMPRE APAGUE OU MOVA O CONTEUDO DAS PASTAS CRIADAS PARA FAZER NOVAMENTE

Se já tem um dataset contendo os arquivos necessários para o Viola-Jones, pule para: *TREINAMENTO*

Modifique o process-volta_da_ufes_playback_viewer_3D.ini para tocar o log que deseja extrair as imagens
	./proccontrol process-volta_da_ufes_playback_viewer_3D.ini

Crie a pasta traffic_light_db/ no bin
mkdir /dados/traffic_light_db_<nome_do_log>

dentro da traffic_light_db/ crie a pasta img
	mkdir /dados/traffic_light_db_<nome_do_log>/img
Rode o extrator do BD
	./traffic_light 3 -generate_database on -database_path /dados/traffic_light_db_<nome_do_log>
Toque o log até o final

Esse modo do traffic light irá gerar três arquivos txt: 
green.txt - Ground Truth com o caminho das imagens com semaforo verde
red.txt - Ground Truth com o caminho das imagens com semaforo vermelho
undetected.txt - Caminho das imagens que contém semáforos mas não foram reconhecidos.
negatives.txt - Caminho das imagens que não contém semáforos.
dentro da pasta /dados/traffic_light_db_<nome_do_log>/img serão salva todas as imagens.

Após separar o dataset e marcar todos os semáforos nas imagens,
 e verificar nas negatives.txt retirando todos os frames que contenham semáforo:

Passos para gerar os dados para treinamento do Viola-Jones
O Viola-Jones utiliza um conjunto de imagens que contém semáforos e 
um conjunto de imagens negativas sem semáforos (normalmente do mesmo trajeto)

Junte os arquivos marcados com semáforos para obter o total de imagens com semafóros marcados (ground_truth);
após gere um novo arquivo retirando o ROI do final da linha para conter apenas o caminho da imagem.
	Sugestão: 
		cat green.txt red.txt yellow.txt off.txt | sort > ground_truth.txt
		cat ground_truth.txt | awk '{print $1}' > positives.txt

*TREINAMENTO*

Agora use o programa create_division para gerar os arquivos de treino, teste e validacao
	cd $CARMEN_HOME/src/traffic_light/tools/create_division_of_train_test_validation
	make
	./create_division <arquivo com a lista das imagens capturadas SEM ground truth (positives)>
	
Rode o script para separar o ground truth referente a cada conjunto
	exemplo: grep -f test.txt ground_truth.txt > gt_test.txt
		 grep -f train.txt ground_truth.txt > gt_train.txt
Mova os arquivos para pasta correta /dados/traffic_light_db_<nome_do_log>/

Crie os arquivos createsamples.txt e training_cascade.txt que irão conter os parâmetros para treinamento [Dissertacao_Tiago]
[http://docs.opencv.org/2.4/doc/user_guide/ug_traincascade.html] -
exemplos desses arquivos estão na pasta $CARMEN_HOME/src/traffic_light/tools/haar_train/

use wc -l train.txt negatives.txt para ver o numero de imagens em cada arquivo

Parâmetros utilizados no create_samples.txt
-w <sample_width> > Width (in pixels) of the output samples.
-h <sample_height> > Height (in pixels) of the output samples.
-num <number_of_samples> > Number of positive samples to generate.

Parâmetros utilizados no training_cascade.txt
-numPos 2077 -numNeg 1101 -w 10 -h 20 -minHitRate 0.999 -maxFalseAlarmRate 0.5 -featureType LBP -precalcValBufSize 2048 -precalcIdxBufSize 2048
-numPos <number_of_positive_samples> - Este número deve ser calculado pelo numero de amostras * minHitRate - Se der erro, reduza mais
-numNeg <number_of_negative_samples>
        Number of positive/negative samples used in training for every classifier stage.
-precalcValBufSize <precalculated_vals_buffer_size_in_Mb>
        Size of buffer for precalculated feature values (in Mb).
-precalcIdxBufSize <precalculated_idxs_buffer_size_in_Mb>
        Size of buffer for precalculated feature indices (in Mb). The more memory you have the faster the training process.
-featureType<{HAAR(default), LBP}>
        Type of features: HAAR - Haar-like features, LBP - local binary patterns.
-w <sampleWidth>
-h <sampleHeight>
	Mesmos do create_samples.txt
-minHitRate <min_hit_rate>
    Minimal desired hit rate for each stage of the classifier. Overall hit rate may be estimated as (min_hit_rate ^ number_of_stages) [Viola2004].
-maxFalseAlarmRate <max_false_alarm_rate>
Maximal desired false alarm rate for each stage of the classifier. Overall false alarm rate may be estimated as (max_false_alarm_rate ^ number_of_stages) [Viola2004].

Haar-like feature parameters:
    -mode <BASIC (default) | CORE | ALL>
----------------------------------------------------------------------------------------------------------------

Inicie o treinamento rodando o programa training
	cd $CARMEN_HOME/src/traffic_light/tools/haar_train
	make
	./training <imagens positivas (train.txt)> <imagens negativas (negatives.txt)> <ground truth (saida do create_gt_filtered> <parâmetros opencv_createsamples> <parâmetros opencv_traincascade>
O programa training gerará um arquivo data_g.xml que contem a arvore de decisão.
	para testar use o programa testing
	./testing <conjunto de teste(test.txt) sem ground truth> <modo de execucao sem visualizacao ou com (0|1)> <arquivo de saída com resultado>
Para verificar o resultado (comparando o arquivo de saída do testing com o ground truth do conjunto de teste)
	./result <arquivo de saída> <ground truth do conjunto de teste (gt_test.txt) <conjunto de teste (test.txt) <resultado gerado pelo training>


