# Instalando a ferramenta virtualenv
sudo python2 -m pip install virtualenv

# Criando o virtualenv e ativando ele
virtualenv -p /usr/bin/python2 squeezeseg_env
source squeezeseg_env/bin/activate

# Instalando as dependencias necessarias para rodar o squeezeseg
pip install -r $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/requirements.txt

# Desativando o virtualenv
#deactivate
