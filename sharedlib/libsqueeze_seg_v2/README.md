# LibSqueezeSegV2

## Semantic segmentation inference lib using SqueezeSegV2

#Make
:~/carmen_lcad/src/mapper_datmo$ make


## Installing dependencies

Install python 2.7 -> ```make install_python2```

Before installing the dependencies, if use without GPU, go to `requirements.txt` and change
`tensorflow-gpu==1.11.0` to `tensorflow==1.11.0`

For working from any place, go to your .bashrc and insert the line at the end of file:

```
# SqueezeSegV2
export PYTHONPATH=$CARMEN_HOME/sharedlib/libsqueeze_seg_v2/src:$PYTHONPATH
```
Save and then refresh your bash with:
```
bash
```

###Installing virtualenv
:~/carmen_lcad/bin$ /$CARMEN_HOME/sharedlib/libsqueeze_seg_v2/create_env.sh

#Activating virtualenv
:~/carmen_lcad/bin$ source $CARMEN_HOME/sharedlib/libsqueeze_seg_v2/squeezeseg_env/bin/activate

### Testing
For testing porpouses, start with SqueezeSegV2 example:
~/carmen_lcad/sharedlib/libsqueeze_seg_v2$ python ./src/demo.py

#Using SqueezeSegV2 with carmen:

Set variable mapper_use_remission = on in your carmen.ini file.

Run Central
```
:~/carmen_lcad/bin$ ./central
```

Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-mapper-datmo-map-generation.ini
```

Run mapper_datmo:
```
:~/carmen_lcad/bin$ ./mapper_datmo -map_path ../data/mapper_teste2 -camera5 right -verbose 2 -file_warnings off
```

Push Playback button. The results from segmentation will be at samples_out:
$CARMEN_HOME/sharedlib/libsqueeze_seg_v2/data/samples_out/

```
:~/carmen_lcad/bin$ ./mapper_datmo -map_path ../data/mapper_teste2 -camera5 right -verbose 2 -file_warnings off
```

### Caso tenho o problema: 

Your CPU supports instructions that this TensorFlow binary was not compiled to use: AVX2 FMA

Baixe o tensorflow obdecendo aos requisitos da tabela no seguinte link:

https://github.com/lakshayg/tensorflow-build

Execute o comando, substituindo o caminho pelo caminho do arquivo baixado:
pip install --ignore-installed --upgrade /path/to/binary.whl

Para instalar o opencv no python, com virtualenv:
python -m pip install opencv-python

### Useful links

https://stackoverflow.com/questions/8998499/virtual-environments-and-embedding-python
https://virtualenv.pypa.io/en/latest/userguide/#using-virtualenv-without-bin-python
