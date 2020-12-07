Para fazer log de um bypass

1- Inicie os CANables colocando um numa porta USB e:
 sudo ip link set can0 up type can bitrate 500000
2- Depois o outro e:
 sudo ip link set can1 up type can bitrate 500000

3- Rode o central em um terminal
 ./central

4- Rode o process-can_log.ini (lembre-se de mudar o nome do arquivo de log no process):
 ./proccontrol process-can_log.ini

Quando o CANable recebe ele pisca azul, quando manda, verde.

Para tocar o log (lembre-se de mudar o nome do arquivo de log no process)

 ./proccontrol process-can_playback.ini
