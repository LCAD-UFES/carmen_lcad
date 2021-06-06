# DeepMapper

DeepMapper é um sistema que estima mapas de profundidade a partir de imagens de câmera monocular utilizando redes neurais profundas.

## Modelos Pré-Treinados
* Os modelos pré-treinados com "NYU depth v2" e "kitti" estão disponíveis [aqui](https://1drv.ms/u/s!AuWRnPR26byUmfRxBQ327hc8eXse2Q?e=AQuYZw).
* Baixe e salve-os na pasta "$CARMEN_HOME/src/deep_mapper/pretrained/", pois serão necessários para os teste.

## Preparando o Ambiente
* Para utilizar é necessário ter o CUDA_10.0 (e Cudnn compatível) , Python 3.5 (ou superior), pip e virtualenv (para evitar problemas)
* Certifique-se de baixar as dependencias no [link](https://1drv.ms/u/s!AuWRnPR26byUmfRbqEF7468fDdHM1g?e=KoabLc)
* Salve-as na pasta do projeto!
* Execute o comando e todas as dependências serão instaladas (processo já testado):
```
./instalar_dependencias.sh 1
```
O parametro 1 é para instalar CUDA e CUNN corretamente, se você já possuir o CUDA 10.0 e Cudnn compatível instalados então rode o seguinte comando:
```
./instalar_dependencias.sh
```
## Testando a rede

## Visualizar imagens lado a lado - original/profundidade
* Rode o seguinte comando para utilizar um vídeo e gerar imagens lado a lado comparando o frame original e a profundidade estimada:
```
source $CARMEN_HOME/src/deep_mapper/venv/bin/activate
python infer_video.py --model kitti --input test_video.mp4
deactivate
```
Obs.: test_video.mp4 é algum vídeo de sua escolha.

 ## Para executar o módulo no Carmen

 Uma vez que os pesos estejam salvos na pasta pretrained e o projeto CARMEN_LCAD compilado e com todas as dependencias instaladas,<br/>
 execute os comandos abaixo para habilitar a utilização do DeepMapper no Carmen.
 ```
 echo "export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/DeepMapper:$CARMEN_HOME/src/deep_mapper/DeepMapper/models/:$PYTHONPATH" >> ~/.bashrc
 source ~/.bashrc

 ```
 Os comandos acima configuram as variáveis de ambiente para que os scripts em pythons sejam corretamente carregados.

 É necessário ajustar algum process.ini de sua preferência para carregar o DeepMapper. Para isso edite o process escolhido e adicione a sequinte linha:
```
deep_map_ART    depth_map   0		0			./deep_mapper  -camera_id 1  # para utilização com o ART e Intelbras
deep_map_IARA   depth_map  	0		0			./deep_mapper  -camera_id 3  # para utilização com a IARA e Bumblebee 
```

Inicie a central como de costume, o proccontrol com o process escolhido e na tela do PROCCONTROL GUI, inicie o DeepMapper relativo à câmera e veículo escolhidos.

Observação: Consome bastante memória de vídeo e é recomendado o uso da GPU TitanV com 11GB. Caso contrário pode ficar muito lento e prejudicar a experiência.

# Artigo original
[AdaBins](https://arxiv.org/abs/2011.14141)
