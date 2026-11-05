# SC-LIO-SAM ROS 2 (CARMEN)

O SC-LIO-SAM gera mapas 3D e localiza o veículo usando LiDAR, IMU, odometria
Ackermann e GPS opcional do CARMEN. O LT-mapper alinha sessões e remove pontos
dinâmicos dos mapas.

Responsáveis: Eduardo Piassaroli de Abreu e Miguel Victoria.

## Requisitos

| Dependência | Ubuntu 20.04 / ROS 2 Foxy | Ubuntu 26.04 / ROS 2 Lyrical |
|---|---|---|
| GTSAM | **4.2.0** — PPA BorgLab | **4.2.0** — Ubuntu `universe` |
| PCL | 1.10.0 | 1.15.1 |
| OpenCV | 4.2.0 | 4.10.0 |
| Boost | 1.71 | 1.90 |
| Eigen | 3.3.7 | 3.4.0 |
| CMake | 3.16.3 | 4.2.3 |
| Compilador / Python | GCC 9 / Python 3.8 | GCC 15 / Python 3.14 |

Estas são as versões usadas na validação. O projeto usa **C++17** e aceita
**CMake ≥ 3.16**. Use GTSAM **4.2.0**; GTSAM 4.3 ou posterior não é suportada.
As demais bibliotecas podem ser instaladas pelo APT de cada Ubuntu.

Compile o **CARMEN** antes deste workspace. São necessários
os headers de `include/carmen`, as bibliotecas de `lib`, `sharedlib/libtf`
e `sharedlib/prob_models`.

## Instalação

Configure primeiro o repositório ROS 2 e o locale UTF-8 conforme a instalação
oficial: [Foxy / Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
ou [Lyrical / Ubuntu 26.04](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html).

### Ubuntu 20.04 / Foxy

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install build-essential cmake git libgtsam-dev \
  python3-colcon-common-extensions python3-numpy python3-yaml \
  libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev \
  libomp-dev libjsoncpp-dev libbullet-dev zlib1g-dev \
  ros-foxy-desktop ros-foxy-pcl-conversions ros-foxy-cv-bridge \
  ros-foxy-image-transport ros-foxy-robot-state-publisher \
  ros-foxy-tf2-geometry-msgs ros-foxy-rosidl-default-generators
```

### Ubuntu 26.04 / Lyrical

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install build-essential cmake git libgtsam-dev \
  python3-colcon-common-extensions python3-numpy python3-yaml \
  libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev \
  libomp-dev libjsoncpp-dev libbullet-dev zlib1g-dev \
  ros-lyrical-desktop ros-lyrical-pcl-conversions ros-lyrical-cv-bridge \
  ros-lyrical-image-transport ros-lyrical-robot-state-publisher \
  ros-lyrical-tf2-geometry-msgs ros-lyrical-rosidl-default-generators
```

Os pacotes GTSAM usados são `4.2.0-2ubuntu2` no Focal e `4.2.0+dfsg-6` no
Resolute. Confira com `apt-cache policy libgtsam-dev`.
Não é necessário instalar `libgtsam-unstable-dev`.

## Compilação

Os quatro pacotes usam `Release` por padrão.

No Ubuntu 20.04,
troque `lyrical` por `foxy` no comando abaixo:

```bash
source /opt/ros/lyrical/setup.bash
cd "$CARMEN_HOME/src/sc_lio_sam_ros2"
MAKEFLAGS=-j2 colcon build --parallel-workers 1
```

**CUDA é opcional.** O build a detecta automaticamente e usa CPU quando
não há toolkit. Para forçar CPU, acrescente
`--cmake-args -DLIO_SAM_ENABLE_CUDA=OFF` ao comando de compilação.
Para usar a busca de mapa na GPU, passe `gpu:=true`
no launch. 

## Executar com proccontrol

Antes de iniciar, ajuste no `.ini` os caminhos do log, do YAML, do
`setup.bash` e de `SC_LIO_SAM_MAP_PATH`. Execute um processo por vez:

```bash
cd ~/carmen_lcad/bin

# IARA
./proccontrol ../data/iara/process/process-playback_iara-sc.ini

# Argos
./proccontrol argos/process-argos-playback-lio-sam.ini
```

Os processos iniciam os serviços CARMEN, o playback e os nós ROS 2.
**Em mapeamento, a pasta de saída é apagada na inicialização.** Use uma
pasta exclusiva e aguarde a gravação do mapa ao encerrar.

## Executar manualmente

Mantenha `central`, `param_daemon` e os sensores/playback em execução.
Os YAMLs usados são:

| Veículo | YAML | LiDAR | Relógio dos processos de playback |
|---|---|---|---|
| IARA | `~/carmen_lcad/data/iara/parameters/param.yaml` | ID 0 | `use_sim_time:=true` |
| Argos | `~/carmen_lcad/bin/argos/params_argos.yaml` | ID 5 | `use_sim_time:=false` |

Os exemplos abaixo usam a IARA. Em um terminal, inicie a ponte:

```bash
source ~/carmen_lcad/src/sc_lio_sam_ros2/setup.bash
ros2 launch carmen_ros_bridge carmen_ros_bridge.launch.py \
  params_file:=$HOME/carmen_lcad/data/iara/parameters/param.yaml \
  lidar_sensor_id:=0 use_sim_time:=true
```

Em outro terminal, inicie o mapeamento:

```bash
source ~/carmen_lcad/src/sc_lio_sam_ros2/setup.bash
mkdir -p "$HOME/sc_lio_sam_output"
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/carmen_lcad/data/iara/parameters/param.yaml \
  savePCDDirectory:=$HOME/sc_lio_sam_output/sessao_nova \
  localizationMode:=false use_sim_time:=true
```

Para localizar em um mapa salvo, substitua o comando de mapeamento por:

```bash
ros2 launch lio_sam run.launch.py \
  params_file:=$HOME/carmen_lcad/data/iara/parameters/param.yaml \
  savePCDDirectory:=$HOME/sc_lio_sam_output/mapa_existente \
  localizationMode:=true use_sim_time:=true \
  initialPoseX:=0.0 initialPoseY:=0.0 initialPoseZ:=0.0 \
  initialPoseRoll:=0.0 initialPosePitch:=0.0 initialPoseYaw:=0.0
```

Ajuste a pose inicial no referencial do mapa: metros e radianos.
Também é possível usar **2D Pose Estimate** no RViz.

| Opção | Uso |
|---|---|
| `use_sim_time` | Use o mesmo valor na ponte e no SLAM |
| `rviz:=false` | Executar sem interface gráfica |
| `rviz_software_rendering:=true` | Alternativa quando houver falha no renderizador do RViz |
| `gpu:=true` | Habilitar busca de mapa CUDA, se compilada |

Os YAMLs precisam das seções `lio_sam` e `carmen_ros_bridge`, com calibração
do LiDAR/IMU e geometria de colisão do veículo. Para listar os argumentos:

```bash
ros2 launch lio_sam run.launch.py --show-args
ros2 launch carmen_ros_bridge carmen_ros_bridge.launch.py --show-args
```

## LT-mapper

Fluxo: **duas sessões com sobreposição → LT-SLAM → Removert → reconstrução**.
Não é necessário executar sensores ou IPC nesta etapa.

### 1. Preparar sessões e configurações

Exemplo em `/dados/ltmapper`; ajuste todos os caminhos se usar outra pasta:

```bash
source ~/carmen_lcad/src/sc_lio_sam_ros2/setup.bash
export LT_ROOT=/dados/ltmapper
mkdir -p "$LT_ROOT/config" "$LT_ROOT/sessoes"
cd ~/carmen_lcad/src/sc_lio_sam_ros2
cp src/ltslam/config/params.yaml "$LT_ROOT/config/ltslam.yaml"
cp src/removert/config/params_ltmapper.yaml "$LT_ROOT/config/removert.yaml"
cp src/removert/config/params_reconstruct.yaml "$LT_ROOT/config/reconstruct.yaml"
```

Coloque as sessões gravadas com `savePCD:true` em `sessoes/central` e
`sessoes/query`. Cada sessão deve conter `Scans/*.pcd`, `SCDs/*.scd`
e `singlesession_posegraph.g2o`.

### 2. Alinhar com LT-SLAM

Edite estes campos em `config/ltslam.yaml`, mantendo os demais parâmetros:

```yaml
ltslam:
  sessions_dir: /dados/ltmapper/sessoes
  central_sess_name: central
  query_sess_name: query
```

Se necessário, ajuste `query_anchor_init_x/y/z` e `query_anchor_init_yaw_deg`
(o yaw desse parâmetro é em graus). Execute:

```bash
ros2 launch ltslam run.launch.py \
  params_file:=$LT_ROOT/config/ltslam.yaml \
  save_directory:=$LT_ROOT/alinhado keep_alive:=false
```

**O LT-SLAM apaga a pasta de saída ao iniciar.** As poses para a próxima
etapa são os arquivos `*_central_aft_intersession_loops.txt`.

### 3. Remover pontos dinâmicos

Edite estes campos em `config/removert.yaml`:

```yaml
removert:
  isScanFileKITTIFormat: false
  central_sess_scan_dir: /dados/ltmapper/sessoes/central/Scans/
  central_sess_pose_path: /dados/ltmapper/alinhado/central_central_aft_intersession_loops.txt
  query_sess_scan_dir: /dados/ltmapper/sessoes/query/Scans/
  query_sess_pose_path: /dados/ltmapper/alinhado/query_central_aft_intersession_loops.txt
```

Confira `sequence_vfov`, `sequence_hfov` e `ExtrinsicLiDARtoPoseBase` para o
sensor. A matriz identidade corresponde a poses do próprio LiDAR.

```bash
ros2 launch removert run_ltmapper.launch.py \
  params_file:=$LT_ROOT/config/removert.yaml \
  save_pcd_directory:=$LT_ROOT/limpo rviz:=false keep_alive:=false
```

### 4. Reconstruir para localização

Em `config/reconstruct.yaml`, copie os quatro caminhos de scans/poses da
etapa anterior para a seção `removert`. Na seção `map_reconstruct`, ajuste:

```yaml
map_reconstruct:
  use_precomputed_global_map: true
  precomputed_global_map_path: /dados/ltmapper/limpo/updated_map.pcd
```

```bash
ros2 launch removert reconstruct_feature_map.launch.py \
  params_file:=$LT_ROOT/config/reconstruct.yaml \
  output_dir:=$LT_ROOT/localizacao
```

Use `$LT_ROOT/localizacao` como `savePCDDirectory` com
`localizationMode:=true`. Use pastas novas nas etapas 3 e 4 para evitar
arquivos antigos misturados aos resultados.

Neste fluxo, `updated_map.pcd` fornece a nuvem global; as features são
extraídas dos scans configurados. Para usar scans limpos, garanta um scan
por linha da trajetória, na mesma ordem. A reconstrução não gera descritores
Scan Context nem âncora GPS; inicialize a pose manualmente.

## Saídas e verificação

| Saída | Conteúdo |
|---|---|
| `/lio_sam/mapping/odometry` | Pose estimada |
| `/lio_sam/mapping/path` | Trajetória |
| `/odometry/gps` | GPS corrigido no frame `utm` |
| `/lio_sam/mapping/gps_in_map` | GPS no mapa; usado pelo display **Odom GPS** do RViz |
| `Scans/`, `SCDs/`, `singlesession_posegraph.g2o` | Sessão para LT-SLAM |
| `cloudCorner.pcd`, `cloudSurf.pcd`, `cloudGlobal.pcd`, `trajectory.pcd`, `transformations.pcd` | Mapa para localização |

O Argos não possui GPS. Na IARA, **Odom GPS** exige GPS/heading válidos e
âncora do mapa; ter mensagens em `/odometry/gps` não garante esse display.

```bash
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic echo /odometry/gps --once
```

Para verificar o workspace compilado, com o ambiente carregado:

```bash
cd ~/carmen_lcad/src/sc_lio_sam_ros2
python3 tests/ros_compatibility.py
python3 tests/run_scancontext_geometry_test.py
python3 tests/gps_covariance_parameters.py
```
