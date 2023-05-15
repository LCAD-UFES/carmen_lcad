## Compile sua aplicação

No diretório principal dela (acima do diretório main), execute o comando:
 
. ../esp-idf-arduino/esp-idf/export.sh
 idf.py build

Para flash:

 idf.py -p /dev/ttyUSB0 flash

Para monitor:

 idf.py -p /dev/ttyUSB0 monitor


