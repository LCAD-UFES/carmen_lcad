# Guia de Instalação: SC-LIO-SAM no Ubuntu 20.04 com ROS Noetic

Este tutorial descreve os passos necessários para configurar e compilar o SC-LIO-SAM utilizando o ROS Noetic no Ubuntu 20.04.

## 1. Instalar Dependências do ROS

Antes de começar, certifique-se de ter o ROS 1 Noetic instalado. Em seguida, instale os pacotes adicionais necessários executando o comando abaixo:

```
sudo apt-get update
sudo apt-get install  \
    ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher
```

## 2. Instalar a Biblioteca GTSAM (Versão 4.0.3)

Execute os comandos abaixo para remover:

```
sudo apt-get purge -y libgtsam-dev ros-noetic-gtsam
sudo apt-get autoremove -y

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
Execute os comandos abaixo para baixar, compilar e instalar:
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

## 3. Compilar o Projeto 

Criar a pasta para salvar a saída:

```
mkdir -p /dados/sc_lio_sam_output/
```

Para evitar erros de referência de diretório em computadores com Carmen, precisamos apontar o caminho correto da biblioteca PCL durante o catkin_make.

```
source /opt/ros/noetic/setup.bash
cd ~/carmen_lcad/src/ros1_sc_lio_sam/
catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl
```

## 4. Testar a Instalação

Com o central rodando, execute o process:

```
cd ~/carmen_lcad/bin
./proccontrol argos/process-argos-playback-lio-sam.ini
```

