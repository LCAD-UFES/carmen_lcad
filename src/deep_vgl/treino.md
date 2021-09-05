# DeepVGL

Para treinar a rede neural do DeepVGL você precisa ter o python2.7 instalado. Além disso, você precisa:

```bash
pip install scipy
pip install matplotlib
```

## Para treinar a rede

Precisaremos de um log com mapa já criado.
Esse mapa, e seu respectivo log, serão utilizados como base para gerar as poses utilizadas na tabela de images/poses 
aprendidas pela rede neural (Deep Neural Network - DNN) do DeepVGL.

Precisaremos também de logs adicionais que serão utilizados para extrair mais imagens de treino para a DNN.
Pode ser feito com apenas 1 log, mas é recomendado pelo menos 2 logs, um para a tabela de images/poses (que também
pode ser usando para treino) e um para treino.

O log base pode ser, também, uma composição de logs (um trecho de interesse pode ser adicionado ao log base, por exemplo).

(logs do ART: https://drive.google.com/drive/folders/1G3uugLXros_OXDL25HTEyPGzmwRDQyuf?usp=sharing)

## STEP 1

Escolha os logs que serão utilizados. Neste tutorial, utilizaremos três logs para treino:

```bash
/dados/log_volta_da_ufes_art-20210131.txt
/dados/log_volta_da_ufes_art-20210120.txt
/dados/log_saida_lcad3_art-20210212.txt
```


Os dois primeiros logs acima são da volta da Ufes completa, equanto que o terceiro é apenas do Art saindo no Lcad3.
Este terceiro log será juntado ao primeiro, como veremos abaixo, para o DeepVGL permitir a localização dentro do Lcad3 também.

Utilizaremos este outro log para teste/validação 

```bash
/dados/log_volta_da_ufes_art-20210305.txt
```

## STEP 1.1

Gere o arquivo "$CARMEN_HOME/src/deep_vgl/treino_e_teste/logs.txt" contendo linhas como abaixo (apropriadas para os 
logs mencionados no passo anterior):

```bash
/dados/log_volta_da_ufes_art-20210131.txt   /dados/ufes/20210131    1   0     2   640x480     0
/dados/log_volta_da_ufes_art-20210120.txt   /dados/ufes/20210120    1   0     2   640x480     0
/dados/log_saida_lcad3_art-20210212.txt     /dados/ufes/20210212    1   0     2   640x480     0
/dados/log_volta_da_ufes_art-20210305.txt   /dados/ufes/20210305    1   0     2   640x480     0
```

Acima, cada linha possui:

1 - O caminho absoluto para o arquivo de log.

2 - O caminho absoluto para a pasta onde serão salvas as imagens (quando usando lidar, salva imagens também) extraídas do log.

3 - A câmera utilizada no log (quando for lidar, este número não importa, mas é conveniente colocar o número de uma câmera existente).

4 - Se utilizando cameras: o crop height ou o angle left do lidar (Serve para eliminar áreas indesejadas da imagem ou do sweep do lidar. No caso da Bumblebee, este valor deve ser 380 para pegar apenas as 380 primeiras linhas das 480 linhas da imagem. No Art toda a imagem pode ser aproveitada. Se utilizando o lidar: o angulo à esquerda a partir do zero (sempre positivo). Imagine que utilizará um range de -45° até 60° da esquerda para direita, sendo a frente do veículo o zero. Logo esse parâmetro será 45. Use 180 se for usar todo o sweep do lidar).

5 - Log format (0, 1, 2, 3 ou 4; 0 para logs antigos, onde a imagem fica dentro do arquivo .txt de log; 1 para logs novos, onde a imagem é salvada à parte; 2 para logs com imagens de câmeras Intelbras; 3 para lidar em arquivo; e 4 para lidar no log).

6 - As dimensões das imagens que devem ser salvas. Se for lidar, salva nas dimensões informadas também.

7 - Similar ao 4, acima. Se utilizando cameras: o total de linhas a serem ignoradas a partir do topo da imagem (0 nas câmeras do Art e em câmeras bumblebee da Iara. Se utilizando lidar: o angulo à direita a partir do zero (sempre positivo). Imagine que utilizará um range de -45° até 60° da esquerda para direita, sendo a frente do veículo o zero. Logo esse parâmetro será 60. Se for usar todo o sweep, 180).


## STEP 1.2

Execute o seguinte comando para extrair as imagens dos logs selecionados:

```bash
$CARMEN_HOME/src/deep_vgl/treino_e_teste/scripts/generate_images.sh $CARMEN_HOME/src/deep_vgl/treino_e_teste/logs.txt
```
Após isso, as imagens estarão disponíveis nas pastas listadas no arquivo logs.txt

## STEP 1.3

Gere o arquivo que associa as poses dos logs com as imagens capturadas (extraídas no passo anterior). Nesse momento, chamamos 
estes arquivos de "camerapos". São um "preview" do dataset, mas sem tratamento algum.

Para gerar os camerapos, executamos o playback de cada log utilizando o process-ground-truth-generator.ini, com o módulo localize_neural_dataset habilitado.

Edite o arquivo "$CARMEN_HOME/src/deep_vgl/process-ground-truth-generator.ini" para ajustar as seguintes linhas ao seu caso:

```bash
 param_daemon       support         1       0           ./param_daemon ../src/carmen-mercedes-atego.ini
```

Acima, coloque o arquivo de parâmetros de seu robô.
 
```bash
playback           gt_log	    1       0           ./playback /dados/log_volta_da_ufes_art-20210131.txt
```

Acima, troque o log para um dos logs do STEP 1.1.

```bash
 exporter           gt_generator    1       0           ./localize_neural_dataset -camera_id 1 -camera_type 1 -output_dir /dados/ufes/20210131 -output_txt /dados/ufes/camerapos-20210131.txt  -log_filename /dados/log_volta_da_ufes_art-20210131.txt
 
```

Acima, troque a data do log (em dois lugares), o camera_id, e o camera_type (0 - Bumblebee, 1 - Intelbras, 2 - Lidar) e ajuste o path do log no último parâmetro.

```bash
 map_server         support         1       0           ./map_server -map_path ../data/map_volta_da_ufes-20210131-art2 -map_x 7757721.8 -map_y -363569.5 -block_map on 
```

Acima, troque o mapa para seu mapa de referência.

```bash
 navigator_gui      monitors        1       0           ./navigator_gui2 -map_path /dados/maps/map_volta_da_ufes-20210131-art2 
```

Acima, troque o mapa para seu mapa de referência.

```bash
 bumblebee_3view    monitors        0       0           ./bumblebee_basic_view 3
 Camera1            monitors        1       0           ./camera_viewer 1
```

Acima, ative o monitor de imagens da sua câmera.

## STEP 1.4

Copie process-ground-truth-generator.ini para $CARMEN_HOME/bin com o seguinte comando:

```bash
cp $CARMEN_HOME/src/deep_vgl/process-ground-truth-generator.ini $CARMEN_HOME/bin
```

E rode o playback do log:

```bash
cd $CARMEN_HOME/bin
{abra um novo terminal e execute} ./central
{abra um novo terminal e execute} ./proccontrol process-ground-truth-generator.ini

```

Certifique-se de que o robô está corretamente localizado no início do log. Se não, localize-o manualmente logo no inicio do log.

Quando terminar o processo, edite o arquivo de saida (/dados/ufes/camerapos-20210131.txt) removendo linhas iniciais se necessário
(linhas onde a localização ainda não estava Ok, por exemplo).


### Repita os steps 1.3 e 1.4 para cada log selecionado.

## STEP 2

Agora que temos os camerapos e as imagens de cada log, precisamos gerar os arquivos de dataset propriamente ditos e ajustar as imagens para o padrão utilizado pela darknet (DNN utilizada no DeepVGL).
Para isso executaremos um script que precisa de alguns parâmetros relativos ao conjunto de dados e espaçamento escolhidos.

Configure os seguintes parâmetros em $CARMEN_HOME/src/deep_vgl/treino_e_teste/scripts/config.txt (estes parâmetros abaixo são bons):

```bash
- image_path="/dados/ufes/" # images target directory from previous steps
- output_path="/dados/ufes_gt/" # output directory 
- base_offset=5 # spacing between base poses (poses no log de referência que serão associadas a labels pela DNN)
- live_offset=1 # space ignored between base poses (usado para evitar imagens que podem estar em 2 poses - zona cinza)
```

### Juntando logs em um mesmo log base, ou de referência.

Caso você não vá juntar logs, pule esta etapa.

Para juntar logs temos que, na verdade, juntar arquivos camerapos. Para isso, basta concatena-los com cat. No exemplo
abaixo, vamos juntar os logs log_volta_da_ufes_art-20210131.txt e log_saida_lcad3_art-20210212.txt.


```bash
cd /dados/ufes
cat camerapos-20210212.txt camerapos-20210131.txt > temp.txt
mv temp.txt camerapos-20210131.txt
```

A ordem é importante: o fim de um log deve encontrar-se, aproximadamente, com o início de outro.
Lembre-se de remover os cabeçalhos repetidos no arquivo final.

A passo a seguir usa o arquivo logs.txt do STEP 1.1, que contém os três logs. Como juntamos dois dos três logs,
precisamos remover um deles de logs.txt. Neste exemplo, o log log_saida_lcad3_art-20210212.txt. Assim, remova
a linha com o log log_saida_lcad3_art-20210212.txt de logs.txt.

### Execução do processo de geração de bases.

Uma vez configurados os parâmetros e, eventualmente, juntados os logs, podemos executar o seguinte comando para 
gerar o dataset de fato:

```bash
$CARMEN_HOME/src/deep_vgl/treino_e_teste/scripts/dataset.sh
```

Caso precise repetir o comando acima, execute:

```bash
rm -r /dados/ufes/train
rm -r /dados/ufes/train.list
rm -r /dados/ufes_gt/*
```

O comando dataset.sh usa o arquivo logs.txt do STEP 1.1 para identificar quais logs vai empregar. A ordem é importante: o primeiro log
(ou o primeiro conjunto de logs se o passo "Juntando logs em um mesmo log base, ou de referência", acima, for empregado) será 
usado como referência pela DNN (suas imagens, espaçadas de base_offset (m), estarão associadas a labels, um
por imagem). 

Durante a execução serão mostrados gráficos dos caminhos percurridos nos logs e a relação de cada pose de cada log com uma pose do log
de referência (o primeiro gráfico é do log de referência com ele próprio).

Após terminar de executar, serão geradas algumas saídas:

Em output_path (/dados/ufes_gt nesse exemplo):
* Arquivos basepos-<log_base>-<log_treino>-<base_offset>-<live_offset>.txt: utilizado para teste e produção.
* Arquivos livepos-<log_base>-<log_treino>-<base_offset>-<live_offset>.txt: utilizados para gerar o dataset de treino.
* Arquivos livepos-<log_base>-TRAIN-<base_offset>-<live_offset>.txt: utilizado para gerar os arquivos necessários ao treinamento da rede.

Pasta train (/dados/ufes/train nesse exemplo):
* As imagens selecionadas de acordo com o espaçamento (base_offset) já no formato utilizado pela darknet.
* A lista de imagens utilizadas para treinamento (no formato da darknet).


## Enumeração do total de classes e configuração da rede para treinamento.

Em nosso exemplo, salvamos os arquivos em "/dados/ufes_gt". O comando a seguir gera a lista de labels utilizadas pela 
darknet no treinamento e teste.

```bash
cat `ls /dados/ufes_gt/basepos-*-5.0m-1.0m.txt | awk '{print $1}'| tail -n 1` | grep -v label | awk '{print "B"$2"E"}' > $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/labels.txt 
```

Agora temos um arquivo com a lista de labels num formato que a darknet possa interpretar corretamente.
Como nossas labels são inteiros e as imagens não nomeadas com timestamp, foi necessário adicionar essas TAGS (B e E) antes e depois do label, para que a darknet conseguisse identificar corretamente a label no nome da imagem. Evitamos assim de alterar o código da darknet.

```bash
# conteúdo do arquivo labels.txt
B0E
B1E
.
.
.
B132E
.
.
.
B657E
```

Precisamos saber o total de labels criadas, para isso execute o seguinte commando:

```bash
cat $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/labels.txt | wc -l
```

O número exibido será utilizado para configurar a saída da darknet.
Neste exemplo 658 será o total de saídas da rede. Em seu caso, dependendo dos logs escolhidos, esse valor pode ser diferente.

## Obtenção do pré-treino da darknet.

O DeepVGL usa parte da darknet pré-treinada para facilitar o trainamento (camadas iniciais). Baixe-as para
o local apropriado no carmen_lcad: 

```bash
cd $CARMEN_HOME/sharedlib/darknet4/
wget https://pjreddie.com/media/files/darknet19_448.conv.23
```

Caso necessário, siga o passo a passo no $CARMEN_HOME/sharedlib/darknet4/README.md da darknet para compila-la para seu hardware.

## Configurações da darknet para iniciar o treinamento da rede.

Precisamos agora editar os arquivos deepvgl.cfg e deepvgl.data para as configurações correspondentes ao novo dataset.

O arquivo "treino_e_teste/darknet_cfg/deepvgl.cfg" é o arquivo de configuração da DNN e deve ser alterado para que a última camada corresponda ao total de labels geradas nos passos anteriores. É importante também garantir que o total de iterações seja no mínimo igual ao total de imagens do dataset. Nesse exemplo editaremos o total de iterações para 8000 (suficiente para o total de imagens) e o número de neurônios na última camada convolucional para 658 (total de labels). 

Desta forma, editaremos a linha "max_batches=" e a última linha "filters=" do deepvgl.cfg:

```bash
gedit $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/deepvgl.cfg
```

Agora, precisamos alterar o arquivo deepvgl.data, que contém a lista de tudo que a darknet precisa para executar o treino corretamente.
Para isso, execute o comando abaixo e edite as linhas conforme o exemplo mostrado:

```bash
gedit $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/deepvgl.data
```

O conteúdo deve ficar parecido com isso:

```bash
classes=658                         # total de labels do dataset (classes para a darknet)
train  = /dados/ufes/train.list     # caminho para o arquivo com a lista das imagens de treino
valid  = /dados/ufes/train.list     # repetimos a arquivo anterior pois não utilizaremos a validação da darknet
labels = /dados/ufes/labels.txt     # caminho para o arquivo com as labels
backup = backup/                    # local onde a darknet salvará os pesos da rede durante o treino
top=1                               # quando executando com o parâmetro "-topk", o inteiro representa o top: 1, 2 ... na métrica da ImageNet 
```

Por último, copie esses arquivos para "/dados/ufes/" com o seguinte comando (não se esqueça de salvar os arquivos modificados acima!):

```bash
cp $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/{labels\.txt,deepvgl\.cfg,deepvgl\.data} /dados/ufes/
```

Agora basta iniciar o treinamento com o comando abaixo:

```bash
cd $CARMEN_HOME/sharedlib/darknet4
make
./darknet classifier train /dados/ufes/deepvgl.data /dados/ufes/deepvgl.cfg darknet19_448.conv.23 
```

A darknet vai começar a treinar e mostrar um gráfico de desempenho. 

Nas linhas impressas pela darknet, as colunas são:
XXXXXXXXXX

Ao final, teremos os pesos da rede salvos na pasta $CARMEN_HOME/sharedlib/darknet4/backup com o nome de deepvgl_final.weights
