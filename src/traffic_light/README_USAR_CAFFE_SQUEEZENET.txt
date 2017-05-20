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
#Para suprimir a saida de texto da caffe
export GLOG_minloglevel=1

Mude as variaveis para usar a squeezenet no $CARMEN_HOME/carmen-ford-escape.ini
#--------- Traffic light ------------
traffic_light_use_squeezenet 		on
traffic_light_gpu_mode 			on
traffic_light_gpu_device_id 		0	

Se quiser usar a CPU em vez da GPU, mude traffic_light_gpu_mode para off
Se o houver mais de uma GPU, escolha o device mudando traffic_light_gpu_device_id para o id da placa que quer utilizar

