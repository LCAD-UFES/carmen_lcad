# Instalando a ferramenta virtualenv
sudo python2 -m pip install virtualenv
# Criando o virtualenv e ativando ele
virtualenv -p /usr/bin/python2 deeplab_env
source deeplab_env/bin/activate
# Instalando as dependencias necessarias para rodar o deeplab
python2 -m pip install -r requirements.txt

sudo apt-get install python-tk # Requerido para utilizar Matplotlib, remover no futuro (?)

# Desativando o virtualenv
#deactivate
