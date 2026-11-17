subir a interface CAN

sudo ip link set can0 up type can bitrate 500000

cd ../../ojNodeManager
bin/ojNodeManager nodeManager_Tramontina.conf a

cd ../ojTramontina/
bin/ojTramontina can0 can0

cd ../ojTorc
bin/ojTorc


. ../esp-idf-arduino/esp-idf/export.sh

Para compilar
 idf.py build

Para flash
 idf.py -p /dev/ttyUSB0 flash

Para monitor
 idf.py -p /dev/ttyUSB0 monitor

to stop the program ctrl+]


