# DeepVGL

## Para treinar a rede (precisa do python2.7 instalado)

Precisaremos de um log com mapa já criado.
Esse mapa, e seu respectivo log, serão utilizados como base para gerar as poses utilizadas na tabela de poses/images do DeepVGL.

Presicaremos também de logs que serão utilizados para extrair as imagens do treino da DNN.

Nesse exemplo utilizaremos 2 logs.

Pode ser feito com apenas 1 log, mas é recomendado pelo menos 2 logs, um para base e um para treino.

## STEP 1
Escolha os logs que serão utilizados.

Geralmente ficam salvos em "/dados"

```bash

ls /dados
log_volta_da_ufes-20160825.txt
log_volta_da_ufes-20160825.txt_bumblebee
log_volta_da_ufes-20160825.txt_velodyne
log_volta_da_ufes-20191003.txt
log_volta_da_ufes-20191003.txt_bumblebee
log_volta_da_ufes-20191003.txt_velodyne

```

## STEP 1.1

Precisamos gerar o arquivo "$CARMEN_HOME/src/deep_vgl/treino_e_teste/logs.txt" contendo linhas como no exemplo abaixo:

ex.:

```bash

/dados/log_volta_da_ufes-20191003.txt   /dados/ufes/20191003    3   380     1   640x480     0
/dados/log_volta_da_ufes-20160825.txt   /dados/ufes/20160825    3   380     1   640x480     0

```

Nesse exemplo, cada linha possui:

1 - o caminho absoluto para o arquivo de log.

2 - o caminho absoluto para a pasta onde serão salvas as imagens extraídas do log.

3 - a câmera utilizada no log.

4 - o crop height (para eliminar áreas indesejadas da imagem).

5 - log format (1 or 2).

6 - as dimensões da imagem de saída

7 - o total de linhas a serem ignoradas o topo da imagem (inteiro)


## STEP 1.2

Gerar as imagens:

execute o seguinte comando para extrair as imagens dos logs selecionados.

```bash

$CARMEN_HOME/src/deep_vgl/treino_e_teste/scripts/generate_images.sh

```
Após isso as imagens estarão disponíveis nas pastas listadas no arquivo logs.txt

## STEP 1.3

Precisamo gerar o arquivo que associa as poses dos logs com as imagens capturadas (extraídas no passo anterior). Nesse momento chamamos de "camerapos". São um "preview" do dataset, mas sem tratamento algum.

Para gerar os camerapos, executamos o playback de cada log utilizando o process-ground-truth-generator.ini, com o módulo localize_neural_dataset habilitado.

Edite o arquivo "$CARMEN_HOME/src/deep_vgl/treino_e_teste/datasets/process-ground-truth-generator.ini" para ajustar as seguintes linhas ao seu caso:

```bash

playback    gt_log          1   0   ./playback /dados/log_voltadaufes-20160825.txt
exporter    gt_generator    1   0   ./localize_neural_dataset -camera_id 3 -output_dir /dados/ufes/20160825 -output_txt /dados/ufes/camerapos-20160825.txt 
 
```

## STEP 1.4

copie process-ground-truth-generator.ini para $CARMEN_HOME/bin com o seguinte comando:

```bash
 
cp $CARMEN_HOME/src/deep_vgl/treino_e_teste/datasets/process-ground-truth-generator.ini $CARMEN_HOME/bin

```

e rode o playback do log:

```bash

cd $CARMEN_HOME/bin
./central &
./proccontrol process-ground-truth-generator.ini

```

### Repita os steps 1.3 e 1.4 para cada log selecionado.

## STEP 2

Agora que temos os camerapos e as imagens de cada log, precisamos geras os arquivos de dataset propriamente ditos e ajustar as imagens para o padrão utilizado pela darknet (DNN utilizada no DeepVGL).
Para isso executaremos um script que precisa de alguns parâmetros relativos ao conjunto de dados e espaçamento escolhidos.

Configure os seguintes parâmetros em \'$CARMEN_HOME/src/deep_vgl/treino_e_teste/scripts/config.txt\':

```bash

- image_path="/dados/ufes/" # images target directory from previous steps
- output_path="/dados/ufes_gt/" # outpu directory 
- base_offset=5 # spacing between base poses
- live_offset=1 # spacing between live poses

```
Uma vez configurados os parâmetros podemos executar o seguinte comando para gerar o dataset de fato:

```bash

$CARMEN_HOME/src/deep_vgl/scripts/dataset.sh

```

## Após terminar de executar, serão geradas algumas saídas

Em output_path (/dados/ufes_gt nesse exemplo):
* Arquivos basepos-<log_base>-<log_treino>-<base_offset>-<live_offset>.txt: utilizado para teste e produção.
* Arquivos livepos-<log_base>-<log_treino>-<base_offset>-<live_offset>.txt: utilizados para gerar o dataset de treino.
* Arquivos livepos-<log_base>-TRAIN-<base_offset>-<live_offset>.txt: utilizado para gerar os arquivos necessários ao treinamento da rede.

Pasta train (/dados/ufes/train nesse exemplo):
* As imagens selecionadas de acordo com o espaçamento (base_offset) já no formato utilizado pela darknet.
* A lista de imagens utilizadas para treinamento (no formato da darknet).


## Precisamos enumerar o total de classes e configurar a rede para treinamento.

Em nosso exemplo, salvamos os arquivos em "/dados/ufes_gt", desta forma o comando a seguir gera a lista de labels utilizadas pela darknet durante o treinamento.

```bash

cat `ls /dados/ufes_gt/basepos-*-5m-1m.txt | awk '{print $1}'| tail -n 1` | grep -v label | awk '{print "B"$2"E"}' > $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/labels.txt 

``` 

Precisamos saber o total de labels criadas, para isso execute o seguinte commando:

```bash

cat $CARMEN_HOME/src/deep_vgl/treino_e_teste/darknet_cfg/labels.txt | wc -l

``` 

O número exibido será utilizado para configurar a saída da darknet.

Neste exemplo 651 será o total de saídas da rede. Em seu caso, dependendo dos logs escolhidos, esse valor pode ser diferente.

## É necessário ter a Darknet apropriadamente instalada.

Baixe a darknet e compile de acordo com seu hardware. Recomendo utilizar GPU (CUDnn) e OpenCV.

o comando abaixo baixa a darknet em /dados/:

```bash

cd /dados/
git clone https://github.com/AlexeyAB/darknet
cd darknet
wget https://pjreddie.com/media/files/darknet19_448.conv.23
cd $CARMEN_HOME/src/deep_vgl/

``` 

Siga o passo a passo no README da darknet e compile para seu hardware.

## Agora temos que criar as configurações da darknet para iniciar o treinamento da rede

(Detalhar esta etapa melhor, anotei assim para nao perder o raciocínio)

Precisamos editar os arquivos deepvgl.cfg e deepvgl.data para as configurações correspondentes ao novo dataset

Temos também que copiar os arquivos:
* labels.txt
* deepvgl.cfg
* deepvgl.data
para a pasta CFG da Darknet

Agora basta iniciar o treinamento com o comando abaixo:

```bash

cd /dados/darknet
./darknet classifier train cfg/deepvgl.data cfg/deepvgl.cfg darknet19_448.conv.23

```
Ao final, teremos os pesos da rede salvos na pasta /dados/darknet/backup