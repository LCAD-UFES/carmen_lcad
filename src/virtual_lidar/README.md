Módulo: Virtual LiDAR

Objetivo do módulo:
Mapa online a ser produzido pelo Virtual LiDAR da mesma forma que seria pelo LiDAR, diminuindo custos para produção de um mapa preciso.

Como?
Através das câmeras, identificando objetos móveis e criando uma mensagem virtual para que o sistema possa tratar no behavior selector.

Surgiu após verificar a distância dos objetos móveis através da câmera, viu-se que o erro médio é de 8% após análise de um grande conjunto de amostras.

Exemplo de uso:
./central
./proccontrol process-playback-fovea.ini 
./virtual_lidar -camera3 1 -croph 780
