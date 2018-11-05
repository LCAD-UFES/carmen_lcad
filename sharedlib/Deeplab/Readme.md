# LibDeepLab

## Semantic segmentation inference lib using Deeplab v3+

## Installing dependencies

instalar o python 2.7 -> ```make install_python2```

Antes de instalar as dependencias, se for utilizar o tensorflow somente na CPU, entre no arquivo `requirements.txt` e troque
`tensorflow-gpu==1.11.0` para `tensorflow==1.11.0`

instalar dependencias -> ```make virtualenv```

baixar modelo -> ```make download```

### Running standalone
Para rodar somente a parte em python, descomente as ultimas linhas presentes no arquivo `bridge.py` e rode `python2 bridge.py`

NAO EH NECESSARIO ATIVAR O VIRTUALENV ANTES DE RODAR DE FORMA STANDALONE, O PROPRIO SCRIPT PYTHON JA FAZ ISSO.

### Useful links

https://stackoverflow.com/questions/8998499/virtual-environments-and-embedding-python
https://virtualenv.pypa.io/en/latest/userguide/#using-virtualenv-without-bin-python