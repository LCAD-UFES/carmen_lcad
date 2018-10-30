

Instalação:
 sudo apt-get update
 sudo apt-get install python3-dev python3-pip
 sudo apt-get install swig
 sudo pip3 install -U virtualenv

 # Cria ambiente virtual para instalar os pacotes python sem bagunçar a instalação de python da máquina
 virtualenv --system-site-packages -p python3 ~/venv

 # Inicia o ambiente virtual
 source ~/venv/bin/activate

 # Instala pacotes necessarios
 pip install --upgrade pip
 pip install tensorflow==1.4.1 gym sacred matplotlib
 pip install opencv-python

 # Compilar
 cd carmen_lcad/src/rl_motion_planner
 make

 # Treinar
 source ~/venv/bin/activate {se ja nao estiver no ambiente virtual}
 python train.py
 deactivate {sai do ambiente virtual}








