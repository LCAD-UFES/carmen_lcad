# IPC_WATCHER

Esse módulo foi criado para servir como um visualizador da comunicação entre os diferentes módulos do carmen, inspirado no rqt do ROS. Para utilizar, execute em um terminal o central, no segundo execute esse módulo executando:

```
cd ~/carmen_lcad/src/ipc_watcher
./ipc_watcher
```

e execute o process desejado (pode testar com o navigate). O módulo registra as mensagens e os subscribers no arquivo log.txt, localizado dentro do módulo.

## INSTALAÇÂO

sudo apt install libgtkmm-3.0-dev
sudo apt install g++ cmake libsdl2-dev libglew-dev libgl1-mesa-dev libx11-dev