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


https://github.com/microsoft/AirSim/issues/1907
If you are using the DepthPlanner image, the depth value at each pixel is the depth of the object in the camera frame. For every pixel (u,v) in the DepthPlanner image, the true point (x, y, z) in the world in the camera frame of reference can be calculated as
x = (u - cx) * d / fx
y = (v - cy) * d / fy
z = d

where
d is the value at the pixel (u,v) in the DepthPlanner image,
(cx, cy) is the principal point,
fx, fy are the focal lengths.

For the values (cx, cy, fx, fy) refer to your settings.json file.
cx = width / 2
cy = height / 2
fx = cx / (tan(FOV / 2))
fy = fx
