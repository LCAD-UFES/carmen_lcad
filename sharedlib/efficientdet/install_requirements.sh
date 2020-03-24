# Instalando a ferramenta virtualenv
sudo python3 -m pip install virtualenv

# Criando o virtualenv e ativando ele
virtualenv -p /usr/bin/python3 venv
source venv/bin/activate

#pip install --ignore-installed --upgrade ~/Downloads/tensorflow-1.13.1-cp35-cp35m-linux_x86_64.whl 
# Instalando as dependencias necessarias para rodar o squeezeseg
python3 -m pip install -r requirements.txt

# Desativando o virtualenv
#deactivate
