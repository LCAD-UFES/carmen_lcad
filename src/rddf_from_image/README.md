# rddf\_from\_image

### Preparação do virtualenv

Esse código precisa ser executado em um virtualenv preparado para utilizar a rede inplace_abn modificada para essa tarefa.

Na pasta de sua preferência, crie um virtualenv que utilize python 3, por exemplo com o comando:

```
virtualenv -p python3.6 <nome_do_ambiente>
```

Em seguida, ative o ambiente (`source <nome_do_ambiente>/bin/activate`) e siga os passos a seguir para instalar nele o que é necessário:

```
pip install torch==1.1
cd $CARMEN_HOME/sharedlib/inplace_abn
python setup.py install
cd scripts
pip install -r requirements.txt
```

### Acertos finais (só são necessários na primeira vez)

Modifique o arquivo bashrc (`gedit ~/.bashrc`) e insira no final:
```
#inplace_abn
export PYTHONPATH=$CARMEN_HOME/sharedlib/inplace_abn/scripts:$PYTHONPATH
```
Depois disso feche e reabra o terminal ou execute o comando `bash` na janela em que o módulo será executado.

É necessário que se possua um arquivo com os pesos de um treinamento prévio da rede neural utilizada pelo módulo. Portanto, antes de sua execução, modifique a linha 41 do arquivo `$CARMEN_HOME/sharedlib/inplace_abn/scripts/run_inplace_abn.py`
```
chk_path = "/home/lcad/carmen_lcad/sharedlib/inplace_abn/BestLoss0601.pt"
```
e insira ali o path do arquivo de pesos que será utilizado.

### Utilização

- Para funcionamento correto não execute o módulo rddf original, pois as mensagens publicadas por ele são do mesmo tipo das desse módulo.

- O melhor desempenho ocorre com a resolução da câmera em 640x480. Mude isso no arquivo carmen-ford-escape.ini.

Este módulo publica RDDF gerado por rede neural em tempo real. Para utilizá-lo execute o código rddf\_from\_image e passe como argumento a id da câmera que fornecerá a entrada da rede, por exemplo:

```
./rddf_from_image 3
```


