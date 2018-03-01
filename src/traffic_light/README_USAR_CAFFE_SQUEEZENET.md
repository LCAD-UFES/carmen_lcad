## Requisitos

Caffe - preferencialmente utilize a Caffe-ENet ja presente em `carmen_lcad/sharedlib/ENet/caffe-enet`.

Siga as [instrucoes de instalacao](https://github.com/LCAD-UFES/carmen_lcad/blob/master/src/road_mapper/readme.md#inst1allation) ate `(end of installation)`.

Preferencialmente compile com CUDA.

Coloque as váriaveis de sistema:

```
 export CAFFE_HOME=$CAFFE_ENET_HOME
 #Para suprimir a saida de texto da Caffe
 export GLOG_minloglevel=1
```

Mude as variaveis para usar a squeezenet no `$CARMEN_HOME/src/carmen-ford-escape.ini`:

```
 #--------- Traffic light ------------
 traffic_light_use_squeezenet 		on
 traffic_light_gpu_mode 			    on
 traffic_light_gpu_device_id 		  0	
```

Se quiser usar a CPU em vez da GPU, mude `traffic_light_gpu_mode` para `off`. 

Se houver mais de uma GPU, escolha o device mudando `traffic_light_gpu_device_id` para o id da placa que quer utilizar. 
