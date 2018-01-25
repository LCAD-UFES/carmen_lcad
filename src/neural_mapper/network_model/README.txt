Codigos em lua para treinamento da rede neural de detecao de estradas a partir do LIDAR, proposta no artigo:
        https://arxiv.org/pdf/1703.03613.pdf

Recomenda-se utilizacao do itorch notebook para vizualizar os dados.

Instalacao do Torch: 
    Seguir tutorial descrito em <http://torch.ch/docs/getting-started.html>

Instalar o itorch notebook:
    Pre-requisitos do iTorch: Seguir tutorial em <https://github.com/facebookarchive/iTorch#requirements>
    Executar no terminal:
        luarocks install itorch
        luarocks install mnist

---------------------------------------------------------
DADOS DE ENTRADA/GROUND TRUTH:

A pasta 'Dataset/'  e composta por arquivos no formato csv de entrada/ground truth da rede, geradas a partir do banco de dados de
marcacao de ruas da KITTI.
5 estatisticas sao geradas para cada instante a partir da nuvem de pontos do LIDAR e servem como dados de entrada. 
Os ground truths foram gerados a partir das imagens de camera com rua marcada na altura do carro,
sendo tranformadas para uma visao de cima pelo metodo 'bird's eye view'.

Dessa maneira, o seguinte padrao de nomenclatura dos arquivos e seguido:
        <indice>_<tipo>_<dado>.csv

indice => indica o instante da captura. Ex: 000001 (segundo instante)
tipo => Categoria de marcacao da kitti (tem variacao se e urbano ou nao e se marca a rua inteira ou faixa). Tipos: 'um', 'umm' ou 'uu'        
dado => Indica qual estatistica/ground truth. Dados: Estatisticas:'max', 'min', 'std', 'mean', 'median';Ground truth: 'gt'

---------------------------------------------------------
SCRIPTS DO MODULO:

model.lua:
abre o modelo de rede neural, e coloca na variavel model

test.lua:
Abre os 5 arquivos de estatisticas de nome 'um_000000' printa a imagem deles na tela, faz um forward no modelo e printa a imagem resultado
(se rede nao estiver treinada, a imagem de saida nao faz sentido)

train.lua:
Treina a rede com dados da kitti, usando os parametros passados por:
        -batchSize => tamanho do batch
        -learningRate => default 0.01 (usado no artigo)
        -trsize => tamanho do banco de dados de treino (se trsize = 10, abre do 'um_000000' ate o 'um_000010'
        -optimization => funcao de otimizacao, default ADAM (e o usado no artigo)
        -kittiType => tipo de banco de dado do Kitti, default 'um' (pode ser 'um', 'uu' ou 'umm')

A loss function utilizada e diretamente setada no codigo como um SpacialCrossEntropyCriterion (linha 47)

No estado atual de desenvolvimento, o treino nao funciona e retorna o erro: 
        cuda runtime error (59) : device-side assert triggered at 
        /tmp/luarocks_cutorch-scm-1-1420/cutorch/lib/THC/generic/THCStorage.c:32

----------------------------------------------------------
FUNCIONAMENTO:

Para rodar o programa, pegar o banco de dados no drive do lcad: (My Drive/Databases/Database NEURAL_MAPPER_KITTI) e descompacta-lo dentro
do diretorio do modulo ('$CARMEN_LCAD/src/neural_mapper_lua_model/network_model')

Abrir itorch notebook:
    No terminal:
       cd $CARMEN_LCAD/src/neural_mapper/network_model
       export CUDA_VISIBLE_DEVICES=0
       itorch notebook

Teste com itorch notebook:
    No iTorch:
        dofile('model.lua')
        dofile('test.lua')
Treino no itorch notebook:
    No iTorch:
        dofile('model.lua') -- Se nao tiver feito em uma linha anterior
        dofile('train.lua')

---------------------------------------------------------



