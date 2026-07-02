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

## 3. Criar o Workspace e Clonar o SC-LIO-SAM

Agora, vamos criar um workspace dedicado para o projeto e clonar o repositório oficial na pasta src.

```
# Criar o workspace
mkdir -p ~/ros_ws_lio_sam/src
cd ~/ros_ws_lio_sam/src

# Clonar o repositório
git clone https://github.com/gisbi-kim/SC-LIO-SAM.git
```

## 4. Compilar o Projeto (WIP)

Para evitar erros de referência de diretório em computadores com Carmen, precisamos apontar o caminho correto da biblioteca PCL durante o catkin_make.

```
# Voltar para a raiz do workspace
cd ~/ros_ws_lio_sam

# Compilar apontando para o diretório correto do PCL no Ubuntu 20.04
catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl

# Atualizar o ambiente
source devel/setup.bash
```
## 5. Testar a Instalação


