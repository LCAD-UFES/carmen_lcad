# DeepVGL

## Para testar a rede (precisa do python2.7 instalado)

Para avaliar o quanto a rede está acertando precisaremos dos seguintes arquivos (gerados na etapa de treino):

```bash
deepvgl.cfg               # as configurações da rede
labels.txt                # os labels gerados para o dataset de treino
deepvgl_final.weights     # os pesos da rede, gerados durante a execução do treino da darknet (vários serão gerados e podem ser avaliados)
```

Precisamos também de uma lista de imagens para validação, que devem ter o mesmo formado da lista utilizada para treino.
Podemos utilizar a mesma lista de imagens usada para treinamento da rede (/dados/ufes/train.list). 

O comando abaixo gera como saída o percentual de acertos da rede com MAE 1, 2 e 3.

```bash
$CARMEN_HOME/src/deep_vgl/deep_vgl-eval --labels /dados/ufes/labels.txt --weights_file $CARMEN_HOME/sharedlib/darknet4/backup/deepvgl_final.weights --config_file /dados/ufes/deepvgl.cfg --images_list /dados/ufes/train.list 
```
Podemos utilizar qualquer um dos pesos gerados (presentes na pasta "/dados/darknet/backup") e gerar uma lista de imagens de validação, não presentes no treino.

## Podemos ver o que a rede está vendo!

Uma vez escolhido o peso com melhores predições, podemos verificar o funcionamento antes de usar o módulo integrado ao CARMEN.

Para visualizar as predições, e gerar os arquivos necessários para integração precisaremos executar algums passos.

Novamente precisaremos dos pesos da rede ("/dados/darknet/backup/deepvgl_final.weights") e das configurações da rede ("/dados/ufes/deepvgl.cfg").

Porém, agora utilizaremos uma lista diferente de imagens. Para gerar essa lista executamos o comando abaixo, lembre-se de ajustá-lo 
para a pasta que deseja utilizar como teste. Em nosso caso "/dados/ufes/20210120":

```bash
readlink -f /dados/ufes/20210120/*.png > $CARMEN_HOME/src/deep_vgl/config/test-20210120.txt
```

Também é necessário um arquivo com uma lista de arquivos de imagem (imagens aprendidas pela rede neural), poses e labels: poses_and_labels.txt. 

Para gerar este arquivo precisamos de um outro arquivo que contém linhas como a abaixo:

```bash
/dados/ufes/20191003/1570117938.166961.bb3.l.png 13 7757785.56312 -363523.757782 0.0 0.011196 -0.04909 0.252019 1570117938.166961
```

Nesta linha, temos uma imagem usada para treino ou teste, o label desta pose (saída da rede, 13), a pose 6D (x, y, z, roll, pitch e yaw),
e o timestamp.

Esse arquivo foi gerado na etapa de treinamento e está salvo (em nosso exemplo) na pasta "/dados/ufes_gt" com o nome:

```bash
/dados/ufes_gt/basepos-20210131-20210120-5.0m-1.0m.txt
```

O comando abaixo gera o arquivo poses_and_labels.txt a partir do arquivo "/dados/ufes_gt/basepos-20210131-20210120-5m-1m.txt".

```bash
cd $CARMEN_HOME/src/deep_vgl
cat /dados/ufes_gt/basepos-20210131-20210120-5.0m-1.0m.txt |grep -v label| awk '{print $3 " " $4 " " $8 " " $1}' > config/poses_and_labels.txt
```
Agora é só executar o comando abaixo para visualizar exatamente o que o deep_vgl está vendo:

```bash
cd $CARMEN_HOME/src/deep_vgl
./deep_vgl-test --poses_and_labels config/poses_and_labels.txt --weights_file $CARMEN_HOME/sharedlib/darknet4/backup/deepvgl_final.weights --config_file treino_e_teste/darknet_cfg/deepvgl.cfg  --images_list $CARMEN_HOME/src/deep_vgl/config/test-20210120.txt 

```
