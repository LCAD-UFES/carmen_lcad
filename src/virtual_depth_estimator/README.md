Virtual Depth Estimator

Objetivo do módulo:
Mapa online a ser produzido pelo Virtual DPT da mesma forma que seria pelo LiDAR, diminuindo custos para produção de um mapa preciso.

Como?
Através das câmeras, identificando objetos móveis e criando uma mensagem virtual para que o sistema possa tratar.

Surgiu após verificar a distância dos objetos móveis através da câmera, viu-se que o erro médio é de 8% após análise de um grande conjunto de amostras.


### Setup - Instruções para instalação

1. Download dos pesos da YoloV4
    ```shell
    cd $CARMEN_HOME/sharedlib/darknet4
    ```
    ```shell
    make download
    ```

1. Download dos pesos da DPT
    ```shell
    cd $CARMEN_HOME/src/virtual_depth_estimator/DPT
    ```
    ```shell
    make download
    ```

1. Criando a virtualenv para instalar as dependências:
    ```shell
    cd $CARMEN_HOME/src/virtual_depth_estimator/DPT
    ```
    ```shell
    ./create_env.sh
    ```

1. Compilando:
    ```shell
    cd $CARMEN_HOME/src/virtual_depth_estimator/
    ```
    ```shell
    make
    ```

1. Exemplo de execução:
    ```shell
    ./central
    ```
    ```shell
    ./proccontrol process-playback-fovea.ini 
    ```
1. Ative a venv e rode o modulo stereo_velodyne:
    ```shell
    source $CARMEN_HOME/src/virtual_depth_estimator/DPT/venv/bin/activate; ./stereo_velodyne_dpt 3
    ```
1.  Para visualizar SOMENTE a rede DPT, execute:
    ```shell
    (venv) > ./virtual_dpt 3
    ```

1.  Para visualizar a Yolo com a rede DPT, execute:
    ```shell
    (venv) > ./virtual_depth -camera3 1
    ```


**Vídeo com a execução:**
https://www.youtube.com/watch?v=0iY1kU2bWpI&t=761s

