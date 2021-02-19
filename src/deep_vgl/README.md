# Para testar o módulo

Para testar o módulo você precisa gerar a base de teste e executar o módulo em modo teste.

## Executar o módulo em modo teste (uso em produção)

Para rodar o módulo (dnn_visual_gl) no modo de produção é necessário um arquivo com uma lista de arquivos de imagem (imagens
aprendidas pela rede neural), poses e labels: poses_and_labels.txt. Para gerar este arquivo precisamos de um outro 
arquivo que contém linhas como a abaixo:

```bash
/dados/ufes/20191003/1570117938.166961.bb3.l.png 13 7757785.56312 -363523.757782 0.0 0.011196 -0.04909 0.252019 1570117938.166961
```

Nesta linha, temos uma imagem usada para treino ou teste, o label desta pose (saída da rede, 13), a pose 6D (x, y, z, roll, pitch e yaw),
e o timestamp.

Para gerar o arquivo com as linhas acima, precisamos rodar o comando abaixo:

```bash
python2.7 $CARMEN_HOME/src/localize_neural2/gerar_dataset/scripts/dataset.py -i /dados/ufes/ -o /dados/ufes_gt/ -b 5 -l 1 -I $CARMEN_HOME/src/localize_neural2/gerar_dataset/scripts/logs.txt
```

O comando abaixo gera o arquivo poses_and_labels.txt a partir do arquivo XXXXX.

```bash
 cat /dados/ufes_gt/basepos-20191003-20191003-5m-1m.txt |grep -v label| awk '{print $3 " " $4 " " $8 " " $1}' > config/poses_and_labels.txt
```

Depois disso é só rodar um process que publique images de câmera e, em seguida, o módulo com seus parâmetros:

```bash
 ./deep_vgl config/config.cfg config/classifier.weights config/poses_and_labels.txt 2 -camera_id 3
```

O módulo dnn_visual_gl vai publicar mensagens gps_xyz e gps gphdt que permitirão localização global.
Use bin/gps_xyz-test se quiser examinar (só as informações essenciais à localização global são incluídas nas mensagens).


# link com as imagens dos logs da IARA - Volta da UFES

https://drive.google.com/drive/folders/1tqRKGO3DtW1yreoxYeD9Ssc3Ip8fxaXC?usp=sharing

20160825<br>
20160825-01<br>
20160825-02<br>
20160830<br>
20160902<br>
20161021<br>
20170119<br>
20171205<br>
20180112<br>
20180112-02<br>
20191003<br>
<br>
escolha um dos logs e salve em um local acessível em seu computador.<br>
ex.:  /dados/ufes/20161021<br>
<br>
!!!! lembre-se de executar o comando "git pull" para ter sempre a versão mais atual do carmen e submódulos !!!!!<br>
<br>
execute os seguintes comandos para gerar a lista de imagens: (altere o caminho de acordo com seu caso)<br>
cd /dados/ufes/<br>
find \`pwd\`/20161021/ -name \*.l.png > imagens.txt<br>
mv imagens.txt $CARMEN_HOME/src/localize_neural2/config/train.txt<br>
<br>
Para compilar basta executar o comando make da seguinte forma:<br>
cd $CARMEN_HOME/src/localize_neural2/<br>
make<br>
<br>
para testar execute:<br>
./localize_neural2<br>
<br>
OBS.: <br>
O arquivo <br>
config/basepos-20191003-20191003-5m-1m.txt, referente ao log_volta_da_ufes-20191003.txt (disponível no google drive de logs do LCAD)<br>
contém a lista de labels e respectivas imagens e poses, no seguinte formato<br>
<sub>image label x y z rx ry rz timestamp<br>
/dados/ufes/20191003/1570117889.210752.bb3.l.png 0 7757730.3562 -363561.284062 0.0 0.037827 0.023097 0.674785 1570117889.210752<br>
/dados/ufes/20191003/1570117928.424578.bb3.l.png 1 7757734.3402 -363558.144286 0.0 0.035305 -0.016079 0.677648 1570117928.424578<br>
/dados/ufes/20191003/1570117929.611112.bb3.l.png 2 7757738.44882 -363555.113682 0.0 0.037699 -0.030118 0.662518 1570117929.611112<br>
/dados/ufes/20191003/1570117930.610327.bb3.l.png 3 7757742.55146 -363552.010419 0.0 0.038893 -0.043297 0.661246 1570117930.610327<br>
/dados/ufes/20191003/1570117931.547136.bb3.l.png 4 7757746.7374 -363548.810127 0.0 0.046705 -0.050046 0.669928 1570117931.547136<br>
/dados/ufes/20191003/1570117932.421384.bb3.l.png 5 7757751.0108 -363545.509366 0.0 0.059163 -0.051509 0.672911 1570117932.421384<br>
/dados/ufes/20191003/1570117933.233292.bb3.l.png 6 7757755.21166 -363542.282578 0.0 0.0505 -0.050288 0.667493 1570117933.233292<br>
/dados/ufes/20191003/1570117933.982701.bb3.l.png 7 7757759.3385 -363539.104213 0.0 0.045324 -0.050641 0.660624 1570117933.982701<br>
/dados/ufes/20191003/1570117934.732098.bb3.l.png 8 7757763.63225 -363535.829442 0.0 0.041511 -0.050475 0.652415 1570117934.732098<br>
</sub>

## Gerar base de teste

Você vai precisar de um log e do poses_opt deste log. Vamos usar o log log_volta_da_ufes-20191003.txt. Seu poses_opt pode
ser econtrado no local padrão: data/graphslam/poses_opt-log_volta_da_ufes-20191003.txt

Edite o arquivo src/localize_neural/log2png.sh e inclua o log de interesse e o diretório onde serão
gravadas as imagens da base. A seguir rode os comandos:

```bash
 cd src/localize_neural
 log2png.sh
```

Rode o comando abaixo no diretório bin para gerar as poses de cada imagem. Se você for empregar outro log, modifique o 
process abaixo de acordo. Lembre-se de criar o diretório de saída de localize_neural_dataset (/dados/ufes e /dados/ufes/20191003 no 
../src/localize_neural2/process-ground-truth-generator.ini).

```bash
 ./proccontrol ../src/localize_neural2/process-ground-truth-generator.ini
```

Sua base de teste é o diretório /dados/ufes/20191003 e o arquivo /dados/ufes/camerapos-20191003.txt especificados em 
../src/localize_neural2/process-ground-truth-generator.ini, mais o arquivo basepos-20191003-20191003-5m-1m.txt.
O nome deste arquivo representa:

```bash
 basepos-<data set de referencia>-<data set de teste>-<espacamento entre os key frames do data set de referencia>-<espacamento estre as imagens no data set de teste>.txt.
```

Para gerar o arquivo basepos-20191003-20191003-5m-1m.txt você vai precisar rodar os comandos abaixo:

```bash
 cd src/localize_neural2
 python gerar_dataset/scripts/dataset.py -i /dados/ufes/ -o /dados/ufes_gt/ -b 5 -l 1
```

O arquivo será gerado em /dados/ufes_gt/basepos-20191003-20191003-5m-1m.txt. O afastamento entre as imagens neste
aqrquivo é de 5m.

Este processo também gera o arquivo /dados/ufes_gt/livepos-20191003-20191003-5m-1m.txt, que é usado no treino.
Este arquivo tem o mesmo formato de basepos-20191003-20191003-5m-1m.txt, mas o afastamento entre as imagens
é de 1m.


xxxxxxxxxxxxxxxxxxx

O procedimento acima conclui o processo de geração da base de teste.


```bash
 readlink -f /dados/ufes/20191003/*r.png > ~/carmen_lcad/src/localize_neural2/config/test.txt
```

É necessário também a lista de poses e labels associadas às saídas da rede treinada. Em nosso exemplo, basta executar o 
seguinte comando:

```bash
 cat /dados/ufes_gt/basepos-20191003-20191003-5m-1m.txt |grep -v label| awk '{print $3 " " $4 " " $8 " " $1}' > config/poses_and_labels.txt
```


