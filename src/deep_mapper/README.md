# DeepMapper

DeepMapper é um sistema que estima mapas de profundidade a partir de imagens de câmera monocular utilizando redes neurais profundas.

## Modelos Pré-Treinados

Os modelos pré-treinados com "AdaBins_nyu.pt" e "AdaBins_kitti.pt" estão disponíveis [aqui](https://1drv.ms/u/s!AuWRnPR26byUmfRxBQ327hc8eXse2Q?e=AQuYZw).
* Baixe e salve-os na pasta "$CARMEN_HOME/src/deep_mapper/pretrained/", pois serão necessários para rodar a rede.
* Também é possível baixar através do comando make download através do gdown.pl.
```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make download
```

## Preparando o Ambiente

```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```
Caso possuir o CUDA 10.0 e CUDNN compatíveis instalados, basta rodar o comando:
```shell
    ./instalar_dependencias.sh
```


* Para utilizar serão necessários o CUDA_10.0 (e CUDNN compatível), Python 3.5 (ou superior), pip e virtualenv, e são automaticamente instalados pelo comando acima.

### Caso não tenha CUDA E CUDNN
* Certifique-se de baixar as dependências no [link](https://1drv.ms/u/s!AuWRnPR26byUmfRbqEF7468fDdHM1g?e=KoabLc)
* Salve-as na pasta do projeto!
* Execute o comando e todas as dependências serão instaladas:
```shell
    ./instalar_dependencias.sh 1
```
O parametro 1 é para instalar CUDA e CUDNN.


## Para executar o módulo no Carmen_LCAD

 Uma vez que os pesos estejam salvos na pasta pretrained e o projeto CARMEN_LCAD compilado e com todas as dependências instaladas,<br/>
 execute os comandos abaixo para habilitar a utilização do DeepMapper no Carmen.
 
 Os comandos abaixo configuram as variáveis de ambiente para que os scripts em pythons sejam corretamente carregados.
### Inclua os diretórios no PYTHONPATH
#### Opção 1
```shell
 echo "export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/:$CARMEN_HOME/src/deep_mapper/models/:$PYTHONPATH" >> ~/.bashrc
 source ~/.bashrc
```
#### Opção 2 - Através do editor de texto:
```shell
    gedit ~/.bashrc
```
E inclua os seguintes dados no final do arquivo:
```shell
    #Deep mapper
    export PYTHONPATH=$CARMEN_HOME/src/deep_mapper:$CARMEN_HOME/src/deep_mapper/models:$PYTHONPATH
```
Salve e feche o editor de texto. Para recarregar as variáveis de ambiente, execute:
```shell
    source ~/.bashrc
```

### Caso queira o módulo dentro de um process.ini (opcional)
É necessário ajustar algum process.ini de sua preferência para carregar o DeepMapper. Para isso edite o process escolhido e adicione a sequinte linha:
```
deep_map_ART    depth_map   0   0   ./deep_mapper  -camera_id 1  # para utilização com o ART e Intelbras
deep_map_IARA   depth_map   0   0   ./deep_mapper  -camera_id 3  # para utilização com a IARA e Bumblebee 
```

### Execução
Inicie a central como de costume, o proccontrol com o process escolhido e na tela do PROCCONTROL GUI, inicie o DeepMapper relativo à câmera e veículo escolhidos.
```shell
cd $CARMEN_HOME/bin/
./central
```
```shell
cd $CARMEN_HOME/bin/
./proccontrol process-playback-fovea.ini
```
```shell
cd $CARMEN_HOME/bin/
source $CARMEN_HOME/src/deep_mapper/venv/bin/activate
(venv) lcad@lcad:~/carmen_lcad/bin$ ./deep_mapper -camera_id 3
```
Após carregar os pesos, basta dar play e começar a rodar o log.


## Testando a rede com um vídeo

## Visualizar imagens lado a lado - original/profundidade
* Rode o seguinte comando para utilizar um vídeo e gerar imagens lado a lado comparando o frame original e a profundidade estimada:
```
source $CARMEN_HOME/src/deep_mapper/venv/bin/activate
python infer_video.py --model kitti --input test_video.mp4
deactivate
```
Obs.: test_video.mp4 é algum vídeo de sua escolha.

Observação: Consome bastante memória de vídeo e é recomendado o uso da GPU TitanV com 11GB. Caso contrário pode ficar muito lento e prejudicar a experiência.

# Artigo original
[AdaBins](https://arxiv.org/abs/2011.14141)
