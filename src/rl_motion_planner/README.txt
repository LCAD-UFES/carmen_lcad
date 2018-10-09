

Instalação:
 sudo apt-get update
 sudo apt-get install python3-dev python3-pip
 sudo pip3 install -U virtualenv

 # Cria ambiente virtual para instalar os pacotes python sem bagunçar a instalação de python da máquina
 virtualenv --system-site-packages -p python3 ~/venv

 # Inicia o ambiente virtual
 source ~/venv/bin/activate

 # Instala pacotes necessarios
 sudo pip install --upgrade pip
 sudo pip install tensorflow==1.4.1 gym sacred matplotlib









