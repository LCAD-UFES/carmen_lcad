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

O projeto usa ROS 1 Noetic e GTSAM **4.2.0**, instalada como pacote binário
via APT. No Ubuntu 20.04, use o PPA do BorgLab; no Ubuntu 26.04, use o
repositório oficial `universe`. Cada sistema usa os seus próprios binários.
Não instale pacotes de uma distribuição na outra.

O ROS Noetic e as bibliotecas do Carmen precisam estar previamente instalados.
No Ubuntu 26.04, este procedimento pressupõe que o ambiente Noetic já esteja
preparado; os comandos abaixo não instalam o ROS nessa distribuição.

### Ubuntu 20.04 (Focal)

```bash
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt-get update
sudo apt-get install libgtsam-dev
sudo apt-get install ros-noetic-navigation ros-noetic-robot-localization ros-noetic-robot-state-publisher
```

### Ubuntu 26.04 (Resolute)

```bash
sudo add-apt-repository universe
sudo apt-get update
sudo apt-get install libgtsam-dev
```

Se você adicionou anteriormente o PPA antigo do README, remova essa entrada
antes do `apt-get update`:

```bash
sudo add-apt-repository --remove ppa:borglab/gtsam-release-4.0
```

O PPA 4.0 não publica pacotes para Resolute. O pacote oficial usado no Ubuntu
26.04 é `4.2.0+dfsg-6`. Não é necessário instalar `libgtsam-unstable-dev` nem
compilar a GTSAM a partir do código-fonte para este workspace.

O pacote `4.2.0+dfsg-6` exporta no CMake uma biblioteca de testes
`libCppUnitLite.a` que não fornece. O arquivo
`src/SC-LIO-SAM/SC-LIO-SAM/cmake/GtsamCompatibility.cmake` detecta esse caso e
importa a biblioteca compartilhada do sistema com suas dependências, sem
carregar a exportação de testes. Nenhum arquivo de `/usr` é alterado.
Instalações sem esse defeito usam a configuração CMake fornecida pela GTSAM.
Esse ajuste e a remoção de um include não utilizado de `gtsam_unstable` não
alteram o algoritmo de SLAM.

Validação desta adaptação: compilação completa no Ubuntu 26.04 com
`libgtsam-dev 4.2.0+dfsg-6`. A compilação no Ubuntu 20.04 e a comparação de
trajetória/mapa em execução ainda precisam ser verificadas.

Para conferir a origem e a versão instalada:

```bash
apt-cache policy libgtsam-dev
```

Referências: [instalação oficial da GTSAM](https://borglab.github.io/gtsam/install/),
[PPA 4.2 para Focal](https://launchpad.net/~borglab/+archive/ubuntu/gtsam-release-4.2),
[pacote de Resolute](https://packages.ubuntu.com/resolute/libgtsam-dev).

Crie a pasta para salvar a saída:

```bash
mkdir -p /dados/sc_lio_sam_output/
sudo chmod 777 /dados/sc_lio_sam_output/
```

Para compilar o workspace:

```bash
source /opt/ros/noetic/setup.bash
export CARMEN_HOME="$HOME/carmen"
cd "$CARMEN_HOME/src/sc_lio_sam"
catkin_make -j$(nproc) -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

Se houver falta de memória, reduza o paralelismo para `-j2`.
Após trocar uma instalação manual pela versão APT, selecione explicitamente
sua configuração para evitar um `GTSAM_DIR` antigo no cache. No Ubuntu 26.04:

```bash
catkin_make --force-cmake -j2 -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DGTSAM_DIR=/usr/lib/cmake/GTSAM
```

No Ubuntu 20.04, consulte `dpkg -L libgtsam-dev` para localizar
`GTSAMConfig.cmake` e use o diretório correspondente, se necessário.
Instalações manuais antigas em `/usr/local` também podem sobrepor cabeçalhos
ou bibliotecas do APT; remova-as pelo procedimento da instalação original
antes de misturar as duas origens.


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

Executar o programa:

```bash
roslaunch lio_sam run.launch
```

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
