# SC-LIO-SAM

> Technical Responsible: Eduardo Piassaroli de Abreu, Miguel Victoria

*******
**Tables of contents**

- [Functional Specification](#functional-specification)
- [Additional Installation](#additional-installation)
- [How to Use](#how-to-use)
- [Technical Specification](#technical-specification)

*******

> Module Classification:
<mark style="background-color: grey;color: white;">Driver</mark>
<mark style="background-color: blue;color: white;">Filter</mark>
<mark style="background-color: green;color: white;">Perception</mark>


## Functional Specification

O SC-LIO-SAM é um algoritmo de SLAM. Ele utiliza de vários dados sensor, como LiDAR, IMU, odometria e GPS para gerar o mapa bruto e o conjunto pose-graph + keyframes (nuvens, poses e descritores)

## Additional Installation

Módulo portado de um fork do CARMEN (ver *Notas de portabilidade*, no fim deste
arquivo). Roda sobre **ROS 1 Noetic**; nesta máquina o Noetic foi compilado
da fonte para o Ubuntu 26.04 (não há pacote `apt` do Noetic para o 26.04) — a
receita está em `~/ros_src/NOTAS.md`.

Pacotes ROS adicionais:

```bash
sudo apt-get update
sudo apt-get install  \
    ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher
```

**GTSAM.** O código usa a API `gtsam::Pose3::Identity()` (maiúscula), que é do
GTSAM ≥ 4.1 — o `identity()` minúsculo do 4.0.3 do PPA antigo **não** compila
mais. Nesta máquina está o **4.2a9** instalado em `/usr/local`:

```bash
ls /usr/local/lib/cmake/GTSAM     # deve existir
grep GTSAM_VERSION_STRING /usr/local/include/gtsam/config.h
```

Se precisar reinstalar, remova a instalação anterior antes:

```bash
sudo apt-get purge libgtsam-dev libgtsam-unstable-dev ros-noetic-gtsam
sudo rm -rf /usr/local/include/gtsam /usr/local/include/gtsam_unstable /usr/include/gtsam
sudo rm -f  /usr/local/lib/libgtsam* /usr/lib/libgtsam*
sudo rm -rf /usr/local/lib/cmake/GTSAM* /usr/lib/cmake/GTSAM*
```

Crie a pasta de saída:

```bash
mkdir -p /dados/sc_lio_sam_output/
sudo chmod 777 /dados/sc_lio_sam_output/
```

### Compilando

O build é próprio (`catkin_make`), **fora** do `make` do `$CARMEN_HOME/src`. Ele
linka contra as libs do CARMEN já compiladas, então rode o `make` do `src/`
antes — o `CMakeLists.txt` da bridge lê `$CARMEN_HOME` e aborta se não estiver
definida.

```bash
source /opt/ros/noetic/setup.bash
cd $CARMEN_HOME/src/sc_lio_sam/
catkin_make -j$(nproc) -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

> `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` é obrigatório: o CMake 4.x do Ubuntu 26.04
> removeu a compatibilidade com `cmake_minimum_required(VERSION < 3.5)`, que é o
> que quase todo pacote do Noetic (inclusive o `toplevel.cmake` do catkin) ainda
> declara. Sem a flag o `cmake` nem configura. Mesma flag usada no build do
> Noetic em `~/ros_src/NOTAS.md`.

Se a máquina for fraca, troque por `-j2`/`-j4`: o `mapOptmization.cpp` sozinho
come vários GB de RAM ao compilar.

Saem 12 executáveis em `devel/lib/`:

| Pacote | Binários |
|---|---|
| `lio_sam` | `imageProjection`, `featureExtraction`, `imuPreintegration`, `mapOptmization` |
| `carmen_ipc_bridge` | `ipc_bridge_node`, `pointcloud_node`, `gps_localization_node`, `fused_odometry_from_slam_node` |
| `ltslam` / `removert` | `ltslam_ltslam`, `removert_removert`, `removert_reconstruct_feature_map` |

`build/`, `devel/` e o symlink `src/CMakeLists.txt` são gerados pelo `catkin_make`
e estão no `.gitignore` do módulo.


## How to Use

Detalhes de como configurar e executar o módulo.

### Mandatory Inputs

Parâmetros principais do arquivo `params.yaml` (SC-LIO-SAM):

| Grupo | Parâmetros | Para que serve |
|---|---|---|
| Tópicos/Frames | `pointCloudTopic`, `imuTopic`, `lidarFrame`, `baselinkFrame` | Define de onde vêm os dados e os frames do TF |
| Exportação | `savePCD`, `savePCDDirectory`, `localizationMode` | Onde salvar os keyframes/poses da sessão; `localizationMode: true` reusa um mapa existente só para localizar, sem sobrescrever |
| Sensor | `N_SCAN`, `Horizon_SCAN`, `lidarMinRange`/`MaxRange` | Características do LiDAR usado |
| IMU | `imuAccNoise`, `imuGyrNoise`, `imuAccBiasN`, `imuGyrBiasN`, `imuGravity` | Ruído/calibração do IMU (valores de allan variance) |
| Features LOAM | `edgeThreshold`, `surfThreshold`, `edgeFeatureMinValidNum` | Controla o que é aceito como "quina" (edge) vs "superfície plana" — valores mais altos = mais exigente, evita features falsas em ambientes estreitos (corredores) |
| Loop closure | `loopClosureEnableFlag`, `historyKeyframeSearchRadius`, `historyKeyframeFitnessScore` | Detecção de loop **dentro** da própria sessão |

> WIP: há mais parâmetros a serem adicionados e detalhados.

Dependências de outros módulos:
- Executar o `central`, o `param_daemon` e os módulos que publicam os sensores.

Executar o programa. O `run.launch` tem dois `arg` **obrigatórios** (`params_file`
e `savePCDDirectory`) — sem eles o `roslaunch` aborta. É preciso dar `source` no
`devel/setup.bash` deste workspace antes:

```bash
source $CARMEN_HOME/src/sc_lio_sam/devel/setup.bash
roslaunch lio_sam run.launch \
    params_file:=$CARMEN_HOME/bin/argos/params_argos.yaml \
    savePCDDirectory:=/dados/sc_lio_sam_output/minha_sessao/
```

Modo de localização (reusa um mapa existente sem sobrescrever), com a pose
inicial dentro do mapa:

```bash
roslaunch lio_sam run.launch localizationMode:=true \
    params_file:=$CARMEN_HOME/bin/argos/params_argos.yaml \
    savePCDDirectory:=/dados/sc_lio_sam_output/meu_mapa/ \
    initialPoseX:=... initialPoseY:=... initialPoseYaw:=...
```

E a bridge com o IPC do CARMEN, em outro terminal:

```bash
source $CARMEN_HOME/src/sc_lio_sam/devel/setup.bash
roslaunch carmen_ipc_bridge carmen_ipc_bridge.launch
```

Na prática nada disso é digitado à mão: o `proccontrol` (abaixo) sobe os dois com
os argumentos certos.

### Outputs

O módulo gera a odometria e o mapa bruto da sessão: nuvens de pontos (`.pcd`), poses e o pose-graph com os descritores globais (Scan Context) de cada keyframe. Esses arquivos são salvos na pasta definida em `SC_LIO_SAM_MAP_PATH` (por padrão `/dados/sc_lio_sam_output/`).

Para gerar o mapa (CARMEN + SC-LIO-SAM), rode o proccontrol com o .ini correto, ajustando `LOG_NAME` e `SC_LIO_SAM_MAP_PATH` para cada sessão que for gerar.

```bash
proccontrol argos/process-argos-sc-lio-sam.ini
```

## Technical Specification

O SC-LIO-SAM é baseado no LIO-SAM, que faz odometria LiDAR-inercial acoplada via smoothing and mapping (fatoração de grafo de poses), acrescido do add-on Scan Context. A cada keyframe, o sistema salva a nuvem de pontos, a pose estimada e um descritor global (Scan Context) usado para reconhecimento de lugar e detecção de loop.


### Known Limitations or Issues

Pontos em aberto (WIP) a serem endereçados:
- Reduzir o gasto de memória.
- Detalhar os parâmetros do `params.yaml`.
- Avaliar a migração dos parâmetros do `.yaml` para o `.ini`.
- Atualizar o README com instalação e execução.


## Notas de portabilidade

Este módulo foi trazido de um fork do CARMEN, onde o pacote da bridge se chamava
de outro jeito. Além do rename para **`carmen_ipc_bridge`** (e do header
`carmen_time.hpp`), quatro pontos precisaram de adaptação, porque dependiam de
extensões de API que o fork tinha e o CARMEN não tem. Nenhuma delas mexeu em
código do CARMEN — tudo ficou dentro deste módulo, atrás de uma chave, para
poder ser religado se a API aparecer.

### 1. IMU embutido do LiDAR — **DESLIGADO**

`src/carmen_ipc_bridge/src/ipc_bridge_node.cpp`, chave `CARMEN_HAS_IMU_LIDAR` (0).

A versão original usava `carmen_imu_{publish,subscribe}_imu_lidar_message(...,
sensor_id)`, alimentada por um driver de LiDAR que republicava o IMU embutido
nessa mensagem. Servia de **fallback** quando o XSens sumia ou publicava zerado.
O CARMEN só tem `carmen_imu_subscribe_imu_message()` (sem `sensor_id`) e nenhum
driver de LiDAR publicando nela — não há IMU de LiDAR para ouvir.

**Consequência prática:** o `/imu_raw` passa a depender **só** do XSens. Se o
XSens vier zerado, o nó emite um `ROS_WARN` dizendo que não há fallback, em vez
de trocar de fonte silenciosamente. O `lidar_imu_sensor_id` do
`carmen_ipc_bridge.launch` ficou comentado pelo mesmo motivo.

### 2. `horizontal_angles_deltas` do LiDAR — **DESLIGADO**

`src/carmen_ipc_bridge/src/pointcloud_node.cpp`, chave
`CARMEN_LIDAR_CONFIG_HAS_HORIZONTAL_DELTAS` (0).

A versão original lia `double *horizontal_angles_deltas` do `carmen_lidar_config`
(offset de azimute por canal, vindo da chave `lidar<N>_horizontal_angles` do
`param_daemon`). O struct do CARMEN (`velodyne_messages.h`) não tem esse campo.

**Consequência prática:** nenhuma regressão — o caminho já era opcional.
`ring_cos_az_off_`/`ring_sin_az_off_` ficam na identidade e o azimute sai do
índice do shot, como em qualquer LiDAR sem correção horizontal.

### 3. `theta`/`valid` do `carmen_gps_xyz_message` — **contornado**

`src/carmen_ipc_bridge/src/ipc_bridge_node.cpp`, `gps_xyz_handler()`.

A versão original tinha `theta` e `valid` no struct do GPS (rumo de duas
antenas). O CARMEN não tem. A posição (`x`, `y`, `z`, `nr`, `gps_quality`) é
idêntica e continua sendo lida normalmente; o **rumo** já chegava por um caminho
separado (`carmen_gps_gphdt_message` → `gps_hdt_handler` → `/gps/heading_raw`),
então `frame.theta` sai 0 e quem precisa de rumo usa o tópico do GPHDT.

**Consequência prática:** nenhuma — o `gps_localization_node` já combinava as
duas fontes.

### 4. `..._initialize_gaussian_command_host()` — implementado localmente

`src/carmen_ipc_bridge/src/fused_odometry_from_slam_node.cpp`.

O nó assina o `carmen_localize_ackerman_initialize_message` (clique de
"posicionar robô" do `navigator_gui`/`viewer_3D`) e precisa distinguir o clique
de verdade do **eco do seu próprio snap** — o filtro é pelo campo `host`. A
versão original tinha uma variante `_host` do `initialize_gaussian_command` que
deixava o chamador escolher esse campo; o CARMEN só tem a versão que carimba
`carmen_get_host()`.

Em vez de mexer no `libcarmen` (a interface é compartilhada com o resto do
sistema), a variante foi reimplementada como função `static` dentro do próprio
nó: mesma mensagem IPC, mesmos `NAME`/`FMT`, só com o `host` vindo por parâmetro.

### 5. `zlib` no link

`src/carmen_ipc_bridge/CMakeLists.txt`.

O `libmap_io` do CARMEN lê/grava mapa comprimido e não arrasta a `zlib` sozinho:
sem `-lz` os quatro nós quebram o link em `undefined reference to 'gzopen'`.
