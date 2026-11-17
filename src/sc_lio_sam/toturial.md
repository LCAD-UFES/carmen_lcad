# Tutorial: SC-LIO-SAM integrado ao CARMEN


---

## 0. Instalação

Este tutorial descreve os passos necessários para configurar e compilar o SC-LIO-SAM utilizando o ROS Noetic no Ubuntu 20.04.

Antes de começar, certifique-se de ter o ROS 1 Noetic instalado. Em seguida, instale os pacotes adicionais necessários executando o comando abaixo:

```
sudo apt-get update
sudo apt-get install  \
    ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher
```

Instalar a Biblioteca GTSAM (>= 4.1 — nesta máquina, **4.2a9** em `/usr/local`)

> O PPA `gtsam-release-4.0` **não serve mais**: o código usa
> `gtsam::Pose3::Identity()` (maiúscula), que só existe a partir do 4.1. Confira
> o que está instalado com
> `grep GTSAM_VERSION_STRING /usr/local/include/gtsam/config.h`.

Execute os comandos abaixo para remover uma instalação anterior:

```
sudo apt-get purge libgtsam-dev ros-noetic-gtsam

# Remove os cabeçalhos (headers) do GTSAM
sudo rm -rf /usr/local/include/gtsam
sudo rm -rf /usr/local/include/gtsam_unstable
sudo rm -rf /usr/include/gtsam

# Remove as bibliotecas dinâmicas e estáticas (.so, .a)
sudo rm -f /usr/local/lib/libgtsam*
sudo rm -f /usr/lib/libgtsam*

# Remove os arquivos de configuração do CMake
sudo rm -rf /usr/local/lib/cmake/GTSAM*
sudo rm -rf /usr/lib/cmake/GTSAM*
```
Depois compile e instale o GTSAM >= 4.1 a partir da fonte (não há pacote do
4.2 para o Ubuntu 26.04):

```
git clone --branch 4.2a9 https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && sudo make install && sudo ldconfig
```

Criar a pasta para salvar a saída:

```
mkdir -p /dados/sc_lio_sam_output/
sudo chmod 777 /dados/sc_lio_sam_output/
```

Para compilar corretamente use o comando abaixo (pode substituir por -j2 ou -j4 caso tenha um computador forte, mas pode travar)

```
source /opt/ros/noetic/setup.bash
cd $CARMEN_HOME/src/sc_lio_sam/
catkin_make -j$(nproc) -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

> `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` é obrigatório no Ubuntu 26.04: o CMake 4.x
> removeu a compatibilidade com `cmake_minimum_required(VERSION < 3.5)`, que é o
> que o `toplevel.cmake` do catkin e quase todo pacote Noetic ainda declaram.
>
> O `$CARMEN_HOME` precisa estar definido e o `make` do `$CARMEN_HOME/src` já
> ter rodado: a bridge linka contra as libs do CARMEN em `$CARMEN_HOME/lib`.


---

## 2. O SC-LIO-SAM
É o responsável por gerar a **odometria e o mapa bruto de cada sessão**. É o LIO-SAM (odometria LiDAR-inercial via smoothing/mapping) com o add-on Scan Context, que salva, para cada keyframe, a nuvem de pontos, a pose e um descritor global. Esse conjunto (pose-graph + keyframes) é o que o LT-SLAM vai consumir depois. No seu processo do CARMEN, ele roda via `roslaunch lio_sam run.launch`, salvando os `.pcd` na pasta definida em `SC_LIO_SAM_MAP_PATH`.

---

## 3. A modificação: alinhar mapas com origem oposta (A→B e B→A)

### 3.1 O problema no LT-SLAM original
No LT-SLAM original, o **anchor node** da sessão "query" (o nó que representa o offset entre o referencial da sessão nova e o da sessão central) começa com um valor inicial "neutro" (perto da identidade) e uma covariância grande, esperando que a detecção de loop via Scan Context, sozinha, convirja para o offset correto durante a otimização.

Isso funciona bem quando as duas sessões percorrem o **mesmo caminho no mesmo sentido**. Mas quando a sessão "query" percorre o **mesmo trajeto físico só que ao contrário** — por exemplo, o mapa 1 foi gravado indo de um ponto A até um ponto B, e o mapa 2 foi gravado voltando de B até A — o ponto de partida da sessão 2 não coincide com o ponto de partida da sessão 1: ele coincide com o **fim** dela, e com um heading (yaw) invertido em torno de 180°. Partindo de um anchor "neutro", o otimizador tende a não convergir (ou converge errado) nesse cenário, principalmente em trajetos longos.

### 3.2 A modificação que você fez no código
Você alterou o código-fonte original do LT-SLAM para que o anchor node da sessão query **aceite um valor inicial explícito** (x, y, z, yaw), em vez do valor neutro padrão. Ou seja, em vez de deixar o otimizador "adivinhar" o offset do zero, você injeta uma estimativa inicial coerente com a geometria real do encontro entre as duas sessões — e é a partir dela que a otimização (via anchor node + loops SC/ICP) refina o alinhamento fino.

Isso é o que permite o cenário A→B / B→A:

```
Mapa 1 (central):  A ────────────────────────► B
Mapa 2 (query):    B ◄──────────────────────── A     (gravado no sentido contrário)
```

Fisicamente, a **origem do mapa 2** (ponto B) coincide com o **fim do mapa 1** (ponto B também) — só que a sessão 2 chega ali "de costas" em relação à sessão 1, com o heading girado ~180°. A receita para calcular o chute inicial é:

1. Pegue a pose do **último nó** da sessão central (mapa 1): `x, y, z, yaw` no ponto B.
2. Use esses mesmos `x, y, z` como posição inicial do anchor da sessão query (mapa 2), já que fisicamente é o mesmo ponto.
3. Some **~180°** ao `yaw`, porque a sessão 2 passa por ali no sentido inverso.

No `params_ltslam.yaml` isso fica:

```yaml
# Preencha com uma estimativa grosseira de onde a sessão "02" começa, no referencial
# da sessão central "01". Se a 02 é o caminho de volta (começa perto de onde a 01
# terminou), pegue a pose do último nó de "01" (x,y,z,yaw) e some ~180 graus no yaw.
query_anchor_init_x: -107.463
query_anchor_init_y: -52.144
query_anchor_init_z: 2.035
query_anchor_init_yaw_deg: 7.007
```

Importante: o anchor continua sendo uma variável otimizável (com covariância grande) — ele **não fica travado** nesse valor. A diferença é que agora ele **começa perto do valor certo**, em vez de começar do zero, o que evita que a otimização convirja para um mínimo local errado quando os pontos de partida/chegada das duas sessões são opostos.

Esse mesmo raciocínio generaliza para qualquer par de sessões: o que importa não é "sempre somar 180°", e sim **identificar o ponto físico onde as duas trajetórias se encontram** (início-com-início, início-com-fim, ou qualquer outro trecho comum) e calcular o `yaw` inicial a partir da diferença de heading real entre as sessões naquele ponto.

### 3.3 Ajustes complementares para esse cenário
Dois parâmetros do Scan Context ajudam quando as sessões são longas e/ou de sentido oposto:
- `scDistThres` (comece testando entre 0.4 e 0.5) — sensibilidade para aceitar um loop entre sessões;
- `scNumCandidates` (comece testando entre 10 e 20) — quantos candidatos de loop são avaliados.

E o `rsSearchRadius` (busca por proximidade entre sessões) só deve ser aumentado **depois** que o anchor já estiver numa posição razoável — aumentá-lo sem corrigir o chute inicial só aumenta falso positivo.

---

## 4. Parâmetros principais de cada arquivo

### 4.1 `params_argos.yaml` (SC-LIO-SAM)
| Grupo | Parâmetros | Para que serve |
|---|---|---|
| Tópicos/Frames | `pointCloudTopic`, `imuTopic`, `lidarFrame`, `baselinkFrame` | Define de onde vêm os dados e os frames do TF |
| Exportação | `savePCD`, `savePCDDirectory`, `localizationMode` | Onde salvar os keyframes/poses da sessão; `localizationMode: true` reusa um mapa existente só para localizar, sem sobrescrever |
| Sensor | `N_SCAN`, `Horizon_SCAN`, `lidarMinRange`/`MaxRange` | Características do LiDAR usado |
| IMU | `imuAccNoise`, `imuGyrNoise`, `imuAccBiasN`, `imuGyrBiasN`, `imuGravity` | Ruído/calibração do IMU (valores de allan variance) |
| Features LOAM | `edgeThreshold`, `surfThreshold`, `edgeFeatureMinValidNum` | Controla o que é aceito como "quina" (edge) vs "superfície plana" — valores mais altos = mais exigente, evita features falsas em ambientes estreitos (corredores) |
| Loop closure | `loopClosureEnableFlag`, `historyKeyframeSearchRadius`, `historyKeyframeFitnessScore` | Detecção de loop **dentro** da própria sessão |

### 4.2 `params_ltslam.yaml`
| Parâmetro | Para que serve |
|---|---|
| `sessions_dir`, `central_sess_name`, `query_sess_name` | Onde estão as sessões e qual é a central/query |
| `save_directory` | Onde salvar as poses alinhadas |
| `loopFitnessScoreThreshold` | Score máximo de ICP para aceitar um loop entre sessões |
| `kNumSCLoopsUpperBound` / `kNumRSLoopsUpperBound` | Limite de loops considerados (Scan Context / busca por raio) |
| `query_anchor_init_x/y/z/yaw_deg` | Chute inicial da posição da sessão query (ver seção 3) |
| `scDistThres`, `scNumCandidates` | Sensibilidade do Scan Context para achar loops entre sessões |
| `rsSearchRadius` | Raio de busca por proximidade entre sessões (só relevante com anchor já razoável) |

### 4.3 `params_ltremovert.yaml` (LT-remover)
| Grupo | Parâmetros | Para que serve |
|---|---|---|
| Saída | `saveMapPCD`, `saveCleanScansPCD`, `save_pcd_directory` | O que salvar e onde |
| Entradas | `central_sess_scan_dir`, `central_sess_pose_path`, `query_sess_scan_dir`, `query_sess_pose_path` | Apontam para os scans e para os `*_aft_intersession_loops.txt` gerados pelo LT-SLAM |
| Sensor | `sequence_vfov`, `sequence_hfov` | Campo de visão vertical/horizontal do LiDAR usado |
| Amostragem | `use_keyframe_gap`, `keyframe_gap` | De quantos em quantos frames processar (aumentar reduz uso de RAM) |
| Lote | `clean_for_all_scan`, `batch_size`, `valid_ratio_to_save` | Processa a sequência inteira em lotes |
| Resolução | `remove_resolution_list`, `revert_resolution_list` | Resolução das imagens de range usadas para detectar/reverter mudanças |
| Sensibilidade estática | `downsample_voxel_size`, `num_nn_points_within`, `dist_nn_points_within` | Quão "rigoroso" é o critério para considerar um ponto estático |
| Performance | `num_omp_cores` | Threads usadas para projeção dos pontos |

---

## 5. Ordem de execução (passo a passo)

### Passo 1 — Gerar o mapa de cada sessão (CARMEN + SC-LIO-SAM)
Rode o proccontrol apontando para o `.ini` de processo, ajustando `LOG_NAME` e `SC_LIO_SAM_MAP_PATH` para cada sessão que for gerar (uma vez para o mapa 1, outra para o mapa 2):

```bash
proccontrol argos/process-argos-sc-lio-sam.ini
```

Esse `.ini` sobe, entre outras coisas, o `playback` do log, o `mapper` do CARMEN e o SC-LIO-SAM (`roslaunch lio_sam run.launch`), salvando os keyframes/poses da sessão na pasta configurada em `SC_LIO_SAM_MAP_PATH`.

### Passo 2 — Alinhar as sessões com o LT-SLAM

```bash
roslaunch ltslam run.launch
```

Configure o `params_ltslam.yaml` com `sessions_dir`, `central_sess_name`, `query_sess_name` e o chute inicial do anchor (seção 3). A saída são os arquivos de pose alinhada, com nome no padrão:

```
nome-do-mapa1_aft_intersession_loops.txt
nome-do-mapa2_aft_intersession_loops.txt
```

### Passo 3 — Verificar o alinhamento no CloudCompare

Antes de seguir para a limpeza, confira visualmente se as trajetórias ficaram alinhadas:

1. Abra os dois arquivos `*_aft_intersession_loops.txt` no CloudCompare como arquivo ASCII;
2. Na configuração de importação, mapeie:
   - **Coluna 3 → X**
   - **Coluna 8 → Y**
   - **Coluna 12 → Z**
3. Compare visualmente as duas trajetórias — elas devem se sobrepor de forma coerente.

Lembre-se da limitação da seção 2.2: isso só funciona bem se as duas sessões percorrem o **mesmo caminho**. Trajetórias diferentes vão aparecer distorcidas mesmo com o LT-SLAM rodando "sem erro".

### Passo 4 — Limpeza do mapa com LT-remover + LT-map

```bash
roslaunch removert run_ltmapper.launch
```

Ajuste o `params_ltremovert.yaml` apontando `central_sess_pose_path` e `query_sess_pose_path` para os arquivos gerados no Passo 2. O LT-remover e o LT-map rodam juntos nesse launch: o remover limpa (HD/LD) e o LT-map aplica a atualização no mapa central.

---

## 6. Recomendações práticas

- **RAM/Swap:** o processo consome muita memória por causa das nuvens de pontos — dependendo do tamanho do mapa, pode consumir toda a RAM disponível. Recomendo **aumentar a swap** antes de rodar. Como referência, no teste com o mapa Honofre ↔ LCAD foi necessário dividir os frames por 2 (`keyframe_gap`) e, mesmo assim, o processo consumiu os 16 GB de RAM + 20 GB de swap disponíveis, por causa do volume de nuvens de pontos.
- **Como saber que terminou:** quando a CPU se estabiliza (uso baixo) mas a memória continua ocupada, geralmente significa que ele já salvou o mapa e só falta encerrar o processo manualmente (Ctrl+C). Depois disso, ele ainda solta outputs adicionais, como os scans e o mapa atualizado.
- **Visualização final:** recomendo usar o CloudCompare para inspecionar o mapa resultante.

---

## 7. Instalando o CloudCompare corretamente

Não instale via `apt` — o pacote do repositório não carrega as bibliotecas necessárias para abrir arquivos `.pcd` (formato PCL) corretamente. Instale via Flatpak:

```bash
flatpak install flathub org.cloudcompare.CloudCompare
```

Mantenha o Flatpak atualizado para garantir que está baixando a versão mais recente do CloudCompare:

```bash
flatpak update
```