# LibDeepLab

## Semantic segmentation inference lib using Deeplab v3+

## Installing dependencies

instalar o python 2.7 -> ```make install_python2```

Antes de instalar as dependencias, se for utilizar o tensorflow somente na CPU, entre no arquivo `requirements.txt` e troque
`tensorflow-gpu==1.11.0` para `tensorflow==1.11.0`

instalar dependencias -> ```make virtualenv```

baixar modelo -> ```make download```

para funcionar de qualquer lugar -> vai no .bashrc e coloque 

```
# Deeplab
export PYTHONPATH=$CARMEN_HOME/sharedlib/Deeplab:$PYTHONPATH
```

### Running standalone
Para rodar somente a parte em python rode `python2 bridge.py <png_filename>`

NAO EH NECESSARIO ATIVAR O VIRTUALENV ANTES DE RODAR DE FORMA STANDALONE, O PROPRIO SCRIPT PYTHON JA FAZ ISSO.

### Useful links

https://stackoverflow.com/questions/8998499/virtual-environments-and-embedding-python
https://virtualenv.pypa.io/en/latest/userguide/#using-virtualenv-without-bin-python

------------------------------------------------------------------
Generate Semantic Map Images: 
-----------------------------------------------------------

1. Extract camera messages from log (replace log.txt with your log): 
grep BUMB /dados/log.txt > /dados/image_list.txt

2. Give as parameter the path to the image list and an output folder:
./generate_semantic_map_images /dados/image_list.txt /dados/output

* To show images while converting add the flag -show
./generate_semantic_map_images /dados/image_list.txt /dados/output -show

* To generate semantic maps from only one camera side add flag -side and side number (0 left or 1 right)
./generate_semantic_map_images /dados/image_list.txt /dados/output -side 0

* To save color semantic maps add flag -scolor
./generate_semantic_map_images /dados/image_list.txt /dados/output -scolor 

** Multiple flags may be used:
./generate_semantic_map_images /dados/image_list.txt /dados/output -show -side 0 -scolor

** If the output folder does not exist it will be created