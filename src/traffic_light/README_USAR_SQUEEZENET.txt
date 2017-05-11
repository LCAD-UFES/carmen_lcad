## Requisitos

- Caffe - http://caffe.berkeleyvision.org/

  [Installation instructions](http://caffe.berkeleyvision.org/installation.html)
Preferencialmente compile com CUDA
Coloque as váriaveis de sistema:

#Caffe
export CAFFE_HOME=/home/vinicius/caffe
export PATH=$CAFFE_HOME/build/tools:$PATH
export PATH=$CAFFE_HOME/build/lib:$PATH
export LD_LIBRARY_PATH=$CAFFE_HOME/build/lib:$LD_LIBRARY_PATH
#Se compilar pycaffe
export PYTHONPATH=$CAFFE_HOME/python:$PYTHONPATH

Mude no trafffic_light_main.cpp USE_SQUEEZEENET para 1;

Se quiser usar a CPU em vez da GPU, mude GPU_MODE para 0
Se o houver mais de uma GPU, escolha o device mudando DEVICE_ID para o id da placa que quer utilizar

