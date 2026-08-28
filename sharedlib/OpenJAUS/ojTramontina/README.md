# ojEcoTech4
e.coTech4 Driver

Para rodar o sistema de freio ou volante, levante o barramento can (precisa do CANable):

 sudo ip link set can0 up type can bitrate 500000

Depois rode em terminais separados:

 ../ojNodeManager/bin/ojNodeManager ../ojNodeManager/nodeManager_EcoTech4.conf a
 ../ojTorc/bin/ojTorc {tecle w para entrar em Wrench Efforts Mode}
 bin/ojEcoTech4-2 can0 can0

Para compilar o codigo do ESP32:
 cd breaks
 . $HOME/esp/esp-idf/export.sh {o ponto eh importante}
 make
 make flash {para programar o ESP32}
 make monitor {para visualisar a saida do ESP32 via a sua USB}
