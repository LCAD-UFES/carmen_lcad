Para rodar o can_playback:

 ./can_playback 1 vcan0

Onde o 1 é o numero do can que está no log que se deseja dar play (escritas da can1 vistas durante a gravacao do log), 
e vcan0 é a interface can corrente onde se deseja publicar o log.

Com o log tocando, voce pode usar as ferramentas can usuais (candump, cansniffer, etc.).
Exemplo de process de playback de log: process-can_playback.ini
Exemplo de process para gravar log: process-can_log.ini

Para criar barramentos can virtuais:

 sudo modprobe vcan
 sudo ip link add dev vcan0 type vcan
 sudo ip link add dev vcan1 type vcan
 sudo ip link set up vcan0
 sudo ip link set up vcan1

Para remover:

 sudo ip link del vcan0
 sudo ip link del vcan1

