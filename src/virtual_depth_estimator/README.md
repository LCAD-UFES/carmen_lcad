Virtual Depth Estimator

Objetivo do módulo:
Mapa online a ser produzido pelo Virtual DPT da mesma forma que seria pelo LiDAR, diminuindo custos para produção de um mapa preciso.

Como?
Através das câmeras, identificando objetos móveis e criando uma mensagem virtual para que o sistema possa tratar.

Surgiu após verificar a distância dos objetos móveis através da câmera, viu-se que o erro médio é de 8% após análise de um grande conjunto de amostras.


### Setup - Instruções para instalação

1) Download dos pesos da YoloV4
```cd $(CARMEN_HOME)/sharedlib/darknet4
```
```make download
```

2) Download dos pesos da DPT
``` cd $(CARMEN_HOME)/src/virtual_depth_estimator/DPT
```
``` make download
```

3) Criando a virtualenv para instalar as dependências:
``` cd $(CARMEN_HOME)/src/virtual_depth_estimator/DPT
```
``` ./create_env.sh
```

4) Compilando
``` cd $(CARMEN_HOME)/src/virtual_depth_estimator/
```
``` make
```

4) Exemplo de execução
Terminal 1 - 
```./central
```
Terminal 2 
```./proccontrol process-playback-fovea.ini 
```

Terminal 3:
4.1 . Ative a venv
``` source $(CARMEN_HOME)/src/virtual_depth_estimator/DPT/venv/bin/activate
```
4.2 . Executando o código
```(venv) > ./virtual_depth -camera3 1
```


Vídeo com a execução:
https://www.youtube.com/watch?v=0iY1kU2bWpI&t=761s

