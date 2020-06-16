# Índice:
  - Overview
    - Parser
    - HypergraphSCLAM
  - Tutorial
    - Como usar o Parser
    - Como usar o HypergraphSCLAM
  - Arquivo de Configuração do Parser
  - Arquivo de Configuração do HypergraphSCLAM

# Overview

Temos 2 módulos principais: o Parser e o Hypergraphsclam

## Parser

Responsável por ler o arquivo de LOG da IARA, realizar filtragens nos dados, construir o grafo correspondente ao LOG e salvar o grafo em um arquivo desejado.
    
## HypergraphSCLAM
    
Responsável por ler o arquivo contendo o grafo gerado pelo Parser, remontar o grafo no G2O e fazer a otimização das poses e calibração da odometria.

# Sumário dos Comandos para Criação de Mapas

cd bin
mkdir log_do_dia
gedit VoltaUFESAmbiental.txt&
{colocar como conteúdo:
/dados/log_volta_da_ufes-201903025.txt                          volta_ufes_normal_parser_config.txt  carmen-ford-escape.ini  
/dados/log_volta_da_ufes-20190625-estacionamento-ambiental.txt  ambiental_parser_config.txt          carmen-ford-escape.ini  

Editar ../../data/hypergraphsclam/config/volta_ufes_normal_parser_config.txt e ../../data/hypergraphsclam/config/ambiental_parser_config.txt
}
mkdir sync poses
cd sync
../../hypergraphsclam_parser ../VoltaUFESAmbiental.txt
{
Para visualizar o resultado do hypergraphsclam_parser:
gnuplot
>> plot '1_gps.txt' u 1:2 w l
>> replot '1_raw_odometry.txt' u 1:2 w l
}
cd ../poses
../../hypergraphsclam ../sync/sync.txt poses_opt ../../../data/hypergraphsclam/config/volta_ufes_e_ambiental_optimization_config.txt

{
É conveniente pegar os valores de 
}

# Tutorial

Usaremos dois logs de exemplo.

 - Log da volta da UFES
 - Log do estacionamento da Ambiental

 O processo consiste em:
 
 1. Fazer o parse dos logs pra construir grafos. O executável que faz esse procedimento é o *parser*.
 2. Otimizar as poses dos veículos usando o executável *hypergraphsclam*.
 3. Construir e mesclar mapas usando os arquivos de saída do *hypergraphsclam*.

O executavel *parser* gera arquivos **.txt** que são consumidos pelo executável *hypergraphsclam*. O executável *hypergraphsclam* gera  
arquivos contendo as poses dos veículos e esses arquivos podem ser usados para fazer mapas conforme as instruções no módulo MAPPER. Bastando que  
esses arquivos estejam nas pasta esperada pelo MAPPER (**$CARMEN_HOME/bin/tmp/**)
 
## Como usar o Parser

O Parser recebe como entrada 1 arquivo de texto contendo 3 colunas:

1. O caminho do LOG da IARA
2. O arquivo de configuração para o Parser dentro da pasta **$CARMEN_LCAD/src/hypergraphsclam/config/**
3. O arquivo do *.ini* do veículo usado para fazer o LOG.
    
#### Exemplo com 1 LOG:

>**VoltaUFES.txt**  
> ____  
> /dados/log_volta_da_ufes-201903025.txt  volta_ufes_normal_parser_config.txt  carmen-ford-escape.ini  
> ___

&nbsp;&nbsp;&nbsp;&nbsp;A opção por colocar esses argumentos em um arquivo é para permitir a otimização conjunta de vários logs.

#### Exemplo com 2 LOGs:

> **VoltaUFESAmbiental.txt**  
> ____  
> /dados/log_volta_da_ufes-201903025.txt                          volta_ufes_normal_parser_config.txt  carmen-ford-escape.ini  
> /dados/log_volta_da_ufes-20190625-estacionamento-ambiental.txt  ambiental_parser_config.txt          carmen-ford-escape.ini  
> ____  

&nbsp;&nbsp;&nbsp;&nbsp;Ao fornecer mais de 1 LOG no arquivo, o sistema pode tentar encontar fechamento de LOOPs entre os todos os LOGS indicados com o intuito de conectar os subgrafos em um grafo global. Esse recurso é utilizado se e somente se o parâmetro **DISABLE_VELODYNE_LOOP** estiver **COMENTADO** no arquivo de configuração do Parser. Mais instruções abaixo.

*ATENÇÃO: o arquivo de CONFIGURAÇÃO DO PARSER é essencial para gerar bons mapas. Há uma quantidade boa de exemplos de arquivos na pasta **$CARMEN_HOME/src/hypergraphsclam/config**:

    config/ambiental_parser_config.txt
    config/caixa_parser_config.txt
    config/fusion_parser_config.txt
    config/guarapari_parser_config.txt
    config/log_ufes_aeroporto_parser_config.txt
    config/log_vale_parser_config.txt
    config/parser_config.txt
    config/reitoria_parser_config.txt
    config/volta_ufes_contrario_parser_config.txt
    config/volta_ufes_normal_parser_config.txt
    config/volta_ufes_parser_config.txt

Como os veículos podem ser diferentes, as condições do LOG também podem ser bem diferentes, daí a necessidade de configurações para cada LOG. 

No final desse README, temos uma tabela descrevendo cada parâmetro utilizado nos arquivos de configuração do *parser*.

#### Exemplo de utilização do Parser

&nbsp;&nbsp;&nbsp;&nbsp;Por conveniência, vá para uma pasta desejada para salvar os arquivos intermediários: 

```console
$ cd ~/Documents/log_do_dia/
```

&nbsp;&nbsp;&nbsp;&nbsp;Uma vez nessa pasta, vamos criar um arquivo para definir quais logs serão utilizados, especificar o arquivo de configuração e qual *.ini* do veículo.

O conteúdo do arquivo:

> **~Documents/log_do_dia/VoltaUFESAmbiental.txt**  
> ____  
> /dados/log_volta_da_ufes-201903025.txt                          volta_ufes_normal_parser_config.txt  carmen-ford-escape.ini  
> /dados/log_volta_da_ufes-20190625-estacionamento-ambiental.txt  ambiental_parser_config.txt          carmen-ford-escape.ini  
> ____  

Novamente, no exemplo acima, o log da UFES é indicado, com o arquivo *volta_ufes_normal_parser_config.txt* e o *.ini* do Ford Escape. Na linha 2,   
estamos fazendo o mesmo para o log do estacionamento. **OS DOIS ARQUIVOS DE CONFIGURAÇÃO JÁ ESTÃO NA PASTA CONFIG!**. Achei por bem deixar esses  
arquivos nessa pasta e commitá-los, para servir de exemplo e também serem reutilizados.  

Se necessário, crie novos arquivos na pasta config para configurar o parser em futuros logs.

Agora, crie duas subpastas. Uma para guardar os arquivos de sync e outro para guardar as poses otimizadas. Em seguida vá para a pasta de sync, por ser a primeira etapa a ser realizada no processo:

```
mkdir sync poses
cd sync
```

Vamos executar o parse:

```
$CARMEN_HOME/src/hypergraphsclam/parser ../VoltaUFESAmbiental.txt
```

Após o término desse processo, a pasta estará cheia de arquivos temporários:

```
1_gps_minus_error.txt
1_gps_original.txt
1_gps_plus_error.txt
1_gps.txt
1_odometry.txt
1_raw_gps.txt
1_raw_odometry.txt
1_sync.txt
2_gps_minus_error.txt
2_gps_original.txt
2_gps_plus_error.txt
2_gps.txt
2_odometry.txt
2_raw_gps.txt
2_raw_odometry.txt
2_sync.txt
sync.txt
```

Os arquivos que se iniciam com 1_ são do primeiro log e os arquivos que se iniciam com o prefixo 2_ são do segundo LOG.  
Você pode plotar no **gnuplot** esses arquivos para entender as condições de cada LOG. 

Por exemplo, para visualizar o comportamento do GPS e da odometria no primeiro LOG:

```
$ gnuplot
>> plot '1_gps.txt' u 1:2 w l
>> replot '1_raw_odometry.txt' u 1:2 w l
```

O termo *raw* indica o dead reckoning com a odometria sem calibração. O arquivo de configuração do parser pode receber uma calibração prévia da   
odometria. Se houver essa calibração prévia, o resultado do dead reckoning estará no arquivo 1_odometry.txt, por exemplo.  
Esses arquivos estão diponíveis para visualizarmos o resultado da calibração da odometria.

**O arquivo mais importante** nesta pasta é o **sync.txt**. Esse arquivo contém o grafo que deve ser consumido pelo executável *hypergraphsclam*.

### Fechamento de LOOP

Nesse exemplo anterior, o *parser* gerou o grafo de cada LOG e os juntou no arquivo **sync.txt**. Porém, para exemplificar rapidamente o uso, temos que o *fechamento de loops* pode estar desligado!

Para ligar o fechamento de LOOP, vá nos dois arquivos na pasta config:
 - volta_ufes_normal_parser_config.txt
 - ambiental_parser_config.txt

Nesses **DOIS** arquivos comentem a linha: DISABLE_VELODYNE_LOOP

Antes:
```
...
DISABLE_VELODYNE_LOOP
...
```

Depois:

```
...
-- DISABLE_VELODYNE_LOOP
...
```

Com isso, o sistema fará uso das mensagens do Velodyne para fechar os loops entre os LOGS e também dentro de cada LOG.

Se um dos dois LOGS estava com a flag habilidade, execute o *parser* novamente:

```
$CARMEN_HOME/src/hypergraphslam/parser ../VoltaUFESAmbiental.txt
```

O processo vai demorar um pouco mais, tome um café rapidinho. Mas não demore tanto! Será menos de 2 minutos em um HD convencional.
Há dois motivos principais para a demora:
 - O método de ICP está com muitas iterações para garantir maior qualidade e estimar melhor a transformação entre as nuvens de pontos.
 - O sistema não assume qualquer conhecimento prévio sobre o LOG e portanto a busca é exaustiva.

Agora, podemos partir para a otimização das poses.

## Como usar o HypergraphSCLAM

O executável *hypergraphsclam* recebe como *argumentos*:

1. O arquivo de sync gerado na etapa anterior: **sync.txt**.
2. Um prefixo para gerar os arquivos contendo as poses do veículo: como **poses_opt**, por exemplo.
3. O caminho do arquivo de configuração da otimização: $CARMEN_HOME/src/hypergraphsclam/config/volta_ufes_e_ambiental_optimization_config.txt

#### Exemplo de utilização do HypergraphSCLAM

Vamos agora para pasta reservada para o resultado da otimização:

```
$ cd ../poses/
```

Podemos executar o otimizador:

```
$ $CARMEN_HOME/src/hypergraphsclam/hypergraphsclam \
    ../sync/sync.txt \
    poses_opt \
    $CARMEN_HOME/src/hypergraphsclam/config/volta_ufes_e_ambiental_optimization_config.txt
```

No exemplo acima, o primeiro argumento é a referência para o arquivo gerado pelo executável *parser*, o segundo argumento é o  
prefixo para os arquivos que são gerados pelo processo de otimização, o terceiro argumento é o caminho para um arquivo de  
configuração do otimizador. Na pasta *config/* dentro do módulo *hypergraphsclam* há alguns exemplos de configuração para consultas.

Tenha em mente que **pode ser necessário** criar novos arquivos para resolver problemas específicos em um dado log.

Após a execução da otimização, o executável gerará um conjunto de arquivos para visualização no gnuplot e também as poses otimizadas:

```
fake_gps.txt
poses_opt_1.txt
poses_opt_2.txt
poses_opt_bumblebee.txt
poses_opt_external_loops.txt
poses_opt_gps_minus_error.txt
poses_opt_gps_plus_error.txt
poses_opt_sick.txt
poses_opt.txt
poses_opt_velodyne.txt
```

O aquivo **poses_opt_1.txt** contém as poses no primeiro log (volta da UFES) e o arquivo **poses_opt_2.txt** contém as poses do segundo log (ambiental). Esses dois arquivos podem ser utilizados para confeccionar mapas com o módulo **MAPPER**.

## Mapeamento

Para fazer o mapeamento, inicie pelo mapa da volta da UFES. As instruções abaixo devem funcionar em quaisquer outros também:

1. Como é o primeiro mapa, vá no carmen ini correspondente e **DESATIVE** a flag **mapper_mapping_mode_on_use_merge_between_maps** se estiver na versão mais nova ou a flag **mapper_use_remission_threshold** na versão anterior.

2. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation.ini (**ou outro equivalente ajustado para o seu caso**). 
   Lembre-se de colocar nele o seu log, o rddf desejado, e carmen ini em todos os lugares que precisa.

3. Ligue e desligue, no PROCCONTROL GUI, o botão ClTmpDir (clique o botão ClTmpDir e ecolha Start Program, e depois Stop Program) para inicializar o diretório $CARMEN_HOME/bin/tmp.

4. Copie o primeiro arquivo para a pasta requerida pelo **MAPPER**:

    ```
    $ cp poses_opt_1.txt $CARMEN_HOME/bin/tmp/poses_opt.txt
    ```
5. Limpe o diretório de mapas temporários (../data/mapper_teste2) clicando no botão CleanMap. Ele roda muito rápido. Assim, basta escolher Start Program e depois Stop Program. **APENAS NESSE PRIMEIRO MAPA**, no próximo a gente quer fazer o merge! Então não deve apagar!

6. Para construir seu mapa no diretório ../data/mapper_teste2, rode o PubPoses e reinicie seu log no início do trecho de intesse. Rode o log por todo o trecho de interesse em velocidade (Speed do playback control) compatível com o hardware onde você estiver trabalhando. Pronto. Seu mapa estará disponível em ../data/mapper_teste2. Mate o proccontrol e examine seu mapa com um proccontrol de playback_viewer_3D.ini apropriado.

Se desejar parar por aqui e o mapa estiver em condições de produção

7. Mova para o data usando o padrão de nome de pasta de mapas, e copie os arquivos da pasta tmp para a pasta $CARMEN_HOME/data/graphslam/ assim as poses originais do mapa serão preservadas

        ```
        cp $CARMEN_HOME/bin/tmp/poses_opt.txt  $CARMEN_HOME/data/graphslam/poses_opt_<log_usado>
        ```
Se desejar fazer o merge do mapa atual em ../data/mapper_teste2 com um próximo LOG (por exemplo o da Ambiental):

8. Vá no carmen ini correspondente e **ATIVE** a flag **mapper_mapping_mode_on_use_merge_between_maps** se estiver na versão mais nova ou a flag **mapper_use_remission_threshold** na versão anterior.
        
9. Volte no passo 2 ajustando o process para o novo LOG.


## Arquivo de Configuração de Parser

| PARÂMETRO | DESCRIÇÂO | HINT |
| --------- | --------- | ------------- |
| MAXIMUM_VEL_SCANS | Quantidade máxima de leituras do Velodyne a serem consideradas no parser. Zero significa que todas as leituras serão utilizadas. | 0 |
| LOOP_REQUIRED_TIME | Tempo **mínimo** em **segundos** para considerar fechamento de Loops entre duas nuvens de pontos.   | 300 s |
| LOOP_REQUIRED_DISTANCE | Distância **máxima** em metros para considerar fechamento de Loops entre duas nuvens de pontos. | 2.0 m |
| USE_RESTRICTED_LOOPS | Se ativado (não comentado), então só se considera fechamento de Loops se o valor de *YAW* for menor que PI/2 | Tem que experimentar. |
| ICP_THREADS_POOL_SIZE | Quantidade de threads para fazer a odometria com o LiDAR e câmera. | Coloque o valor da CPU disponível |
| ICP_THREAD_BLOCK_SIZE | Cada thread vai disputar por blocos de nuvens de pontos. Esse valor define o tamanho do bloco. | 300 |
| LIDAR_ODOMETRY_MIN_DISTANCE | Distância mínima entre as nuvens de pontos para computar a odometria com Lidar. Nuvens intermediárias são descartadas. | 0.3 m |
| VISUAL_ODOMETRY_MIN_DISTANCE | Distância mínima entre as imagens da câmera para computar a odometria visual. Imagens intermediárias são descartadas. | 0.1 m |
| ICP_TRANSLATION_CONFIDENCE_FACTOR | Penaliza divergência entre a odometria do carro e a odometria do LiDAR. Se a diferença entre elas for maior que esse valor, descartamos a odometria do lidar. | 1.00 m |
| CURVATURE_REQUIRED_TIME | Não utilizado mais no código. Pode desconsiderar e remover de novos arquivos | --- |
| MIN_SPEED | Velocidade mínima para considerar as medições dos sensores. Speed filtering. | 0.01 m/s |
| DISTANCE_BETWEEN_AXLES | Distância entre os eixos do veículo. | 2.625 m |
| MAX_STEERING_ANGLE | Ângulo máximo da roda do veículo. | 0.5337 |
| UNDERSTEER | Utilizado para computar o movimento Ackerman no dead reckoning. | 0.0015 |
| KMAX | Curvatura máxima do veículo. Usado para | 0.17857142857 |
| ODOMETRY_BIAS | Valores de correção do bias da odometria. Um valor prévio obtido com o PSO | 1.0 1.0 0.0 |
| INITIAL_GUESS_POSE | Pose inicial. Apenas para plotar no paper e comparar a odometria com o GPS | 0.0 0.0 0.0 |
| DISABLE_VELODYNE_ODOMETRY | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza a odometria com o Velodyne | Deixe ativado, se precisar comente |
| DISABLE_VELODYNE_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza os fechamentos de LOOP com o Velodyne | Deixe desativado == comentado |
| DISABLE_SICK_ODOMETRY | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza a odometria com o Sick | Deixe ativado, se precisar comente |
| DISABLE_SICK_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza os fechamentos de LOOP com Sick | Deixe desativado == comentado |
| DISABLE_BUMBLEBEE_ODOMETRY | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza a odometria com a câmera Bumblebee | Deixe ativado, se precisar comente |
| DISABLE_BUMBLEBEE_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *parser* **NÃO** utiliza os fechamentos de LOOP com a câmera Bumblebee | Deixe desativado == comentado |
| SAVE_ACCUMULATED_POINT_CLOUDS | Salva o acúmulo de núvens da odometria com o LiDAR em arquivos na pasta /dados/tmp/. Gasta muito espaço. | Deixe desativado == comentado |
| SAVE_LOOP_CLOSURE_POINT_CLOUDS | Salva as nuvens de pontos utilizadas em fechamento de Loops. Apenas para Debug no desenvolvimento. | Deixe desativado == comentado |
| GPS_IDENTIFIER | Identifica o ID do gps no LOG e o respectivo delay. Pode-se usar múltiplas linhas para identificar os delays de mais de um GPS no LOG. | 1 0.0 |
| USE_FAKE_GPS | Se ativado, ou **NÃO COMENTADO**, então as orientações do GPS (**NMEAHDT**) são descartadas. As orientações são recomputadas usando geometria simples. | Veja o estado do GPS no Log com o gnuplot |
| USE_GPS_ORIENTATION | Se ativado, ou **NÃO COMENTADO**, então a pose inicial da odometria é feita com a orientação do GPS. Apenas para PLOT no paper. | Deixe comentado. |


## Arquivo de Configuração do HypergraphSCLAM

Covariâncias: quanto menor mais confiança no sensor.

> XX -> Variância de x em relacao a x  
YY -> Variância de y em relacao a y  
HH -> Variância do heading em relacao ao heading 

Unidade da variancia nos deslocamentos: metros ao quadrado - area  
Unidade da variancia na orientação: radianos ao quadrado


| PARÂMETRO | DESCRIÇÂO | HINT |
| --------- | --------- | ------------- |
| ODOMETRY_XX_VAR | Variância da odometria, X em relação a X | 0.05 |
| ODOMETRY_YY_VAR | Variância da odometria, Y em relação a Y | 0.05 |
| ODOMETRY_HH_VAR | Variância da odometria, H em relação a H | 0.09 |
| SICK_ICP_XX_VAR | Variância do Sick, X em relação a X | 0.8 |
| SICK_ICP_YY_VAR | Variância do Sick, Y em relação a Y | 0.8 |
| SICK_ICP_HH_VAR | Variância do Sick, H em relação a H | 0.4 |
| SICK_LOOP_ICP_XX_VAR | Variância do fechamento de Loop com o Sick, X em relação a X | 1.75 |
| SICK_LOOP_ICP_YY_VAR | Variância do fechamento de Loop com o Sick, Y em relação a Y | 1.70 |
| SICK_LOOP_ICP_HH_VAR | Variância do fechamento de Loop com o Sick, H em relação a H | 0.75 |
| VELODYNE_ICP_XX_VAR | Variância do Sick, X em relação a X | 0.85 |
| VELODYNE_ICP_YY_VAR | Variância do Sick, Y em relação a Y | 0.85 |
| VELODYNE_ICP_HH_VAR | Variância do Sick, H em relação a H | 0.85 |
| VELODYNE_LOOP_ICP_XX_VAR | Variância do fechamento de Loop com o Velodyne, X em relação a X | 3.0 |
| VELODYNE_LOOP_ICP_YY_VAR | Variância do fechamento de Loop com o Velodyne, Y em relação a Y | 3.0 |
| VELODYNE_LOOP_ICP_HH_VAR | Variância do fechamento de Loop com o Velodyne, H em relação a H | 2.5 |
|VISUAL_XX_VAR | Variância do odometria visual, X em relação a X | 0.250 |
|VISUAL_YY_VAR | Variância do odometria visual, Y em relação a Y | 0.250 |
|VISUAL_HH_VAR | Variância do odometria visual, H em relação a H | 0.125 |
| XSENS_CONSTRAINT_VAR | Variância do Xsens, mesmo valor para todos os eixos. Pode desconsiderar. Uso do Xsens desabilitado na configuração padrão | 0.785398163 |
| GPS_POSE_STD_MULTIPLIER | O GPS tem um desvio padrão definido, aqui temos um multiplier para controlar a confiança na posição. Hack. | 4.0 |
| GPS_POSE_HH_STD | O GPS tem um desvio padrão definido, aqui temos um multiplicador para controlar a confiança na orientação. | 20.0 |
| SPECIAL_ODOMETRY_INFORMATION | 2 mensagens com o mesmo timestamp (**NMEAHDT** e **NMEAGGA**) deveriam estar no mesmo lugar. Penaliza quando o otimizador tenta separar os dois nós correspondentes.  | 100 |
| ODOM_ACKERMAN_PARAMS_VERTICES | Quantos nós de calibração da odometria no grafo.  | 1 |
| OPTIMIZER_OUTER_ITERATIONS | Quantidade de iterações do otimizador. Loop mais externo e global (quantas vezes roda Step 1 e Step 2 no paper).  | 2 |
| OPTIMIZER_INNER_POSE_ITERATIONS | Quantidade de iterações na otimização das poses dos veículos (Step 1) | 20 |
| OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS | Quantidade de iterações na calibração da odometria (Step 2). | 10 |
| FAKE_GPS_CLUSTERING_DISTANCE | Distância usada para a clusterização das mensagens do GPS. Zero desativa esse recurso. | 1.2 m |
|  GPS_SPARSITY_THRESHOLD | Distância minima para considerar leituras de GPS. Leituras intermediárias são desconsideradas. Zero desativa esse recurso.  | 0.20 m |
| USE_VELODYNE_SEQ | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza arestas de odometria do Velodyne | Deixe desativado, ative se precisar |
| USE_VELODYNE_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza aresta de fechamento de Loop com o Velodyne | Deixe ativado, desative se não houver loop. |
| USE_SICK_SEQ | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza arestas de odometria do Sick | Deixe desativado. Ative se quiser experimentar. |
| USE_SICK_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza aresta de fechamento de Loop com o Sick | Deixe desativado. Ative se quiser experimentar. |
| USE_BUMBLEBEE_SEQ | e ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza arestas de odometria visual com a Bumblebee. | Deixe desativado. Ative se quiser experimentar. |
| USE_BUMBLEBEE_LOOP | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza aresta de fechamento de Loop com a Bumblebee. | Deixe desativado. Ative se quiser experimentar. |
| USE_GPS | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza arestas de GPS | Deixe sempre ativado. Desative se quiser experimentar. |
| USE_ODOMETRY | Se ativado, ou **NÃO COMENTADO**, então o *hypergraphsclam* utiliza arestas de odometria. | Deixe sempre ativado. |

