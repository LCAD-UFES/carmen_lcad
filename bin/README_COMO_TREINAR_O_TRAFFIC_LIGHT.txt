SE JA TIVER FEIITO ISSO ANTES SEMPRE APAGUE O CONTEUDO DAS PASTAS CRIADAS PARA FAZER NOVAMENTE

Modifique o process-volta_da_ufes_playback_viewer_3D.ini para tocar o log que deseja extrair as imagens
	./proccontrol process-volta_da_ufes_playback_viewer_3D.ini

Crie a pasta traffic_light_db/ no bin
mkdir $CARMEN_HOME/bin/traffic_light_db

dentro da traffic_light_db/ crie a pasta img
	mkdir $CARMEN_HOME/bin/traffic_light_db/img
Rode o extrator do BD
	./traffic_light 3 -generate_database on -database_path $CARMEN_HOME/bin/traffic_light_db
Esse modo do traffic light irá gerar três arquivos txt: 
green.txt - Ground Truth com o caminho das imagens com semaforo verde
red.txt - Ground Truth com o caminho das imagens com semaforo vermelho
negatives.txt - Caminho das imagens que não contém semáforos.
dentro da pasta traffic_light_db/img serão salva todas as imagens.

Passos para gerar os dados para treinamento do Viola-Jones - NAO ESTA CORRETO AINDA!

Junte os arquivos green e red para obter o total de imagens com semafóro (ground_truth) 
após gere um novo arquivo retirando o ROI do final da linha para conter apenas o caminho da imagem (positivas) 
	cat green.txt red.txt | sort > ground_truth.txt
	cat ground_truth.txt | awk '{print $1}' > positives.txt

Junte as positivas com as negativas para ter a lista de todas as imagens
	cat positives.txt negatives.txt | sort > all_images.txt
Agora use o programa create_division para gerar os arquivos de treino, teste e validacao
	cd $CARMEN_HOME/src/traffic_light/tools/create_division_of_train_test_validation
	make
	./create_division <arquivo com a lista das imagens capturadas (all_images)>
	mv train.txt test.txt validation.txt /home/vinicius/carmen_lcad/bin/traffic_light_db/

Separe o ground truth apenas do arquivo de treino #COM PROBLEMA!
	cd $CARMEN_HOME/src/traffic_light/tools/generate_gt_with_train_images
	make
	./create_gt_filtered <nome do arquivo do subconjunto de imagens (train.txt)> <arquivo de ground truth> <nome do arquivo de saida filtrado>
VERIFICANDO O PASSO ABAIXO! ainda nao esta correto---------------------------------------
Ajuste os parametros de numero de imagens nos arquivos createsamples.txt e traincascade.txt
use wc -l train.txt negatives.txt para ver o numero de imagens em cada arquivo
adicione o numero de positivos(train.txt) em createsamples.txt logo após a flag -num 
adicione o numero de positivos(train.txt) em traincascade.txt logo após a flag -numPos e negativos após -numNeg 
----------------------------------------------------------------------------------------------------------------

Inicie o treinamento rodando o programa training
	cd $CARMEN_HOME/src/traffic_light/tools/haar_train
	make
	./training <imagens positivas (train.txt)> <ground truth (saida do create_gt_filtered> <parâmetros opencv_createsamples> <parâmetros opencv_traincascade>
Quando terminar, mova ou apague os arquivos da pasta traffic_light_db.

