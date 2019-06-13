= Instalações
sudo apt-get install can-utils

= Ativar para uso
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ifconfig can0 up

= Teste
candump can0

cansend can0 999#DEADBEEF   # Send a frame to 0x999 with payload 0xdeadbeef
candump can0                # Show all traffic received by can0
canbusload can0 500000      # Calculate bus loading percentage on can0 
cangen can0 -D 11223344DEADBEEF -L 8    # Generate fixed-data CAN messages

cansniffer -c can0          # Display top-style view of can traffic

The filter of cansniffer FILTER can be a single CAN-ID or a CAN-ID/Bitmask:
+1F5<ENTER>    - add CAN-ID 0x1F5
-42E<ENTER>    - remove CAN-ID 0x42E
-42E7FF<ENTER> - remove CAN-ID 0x42E (using Bitmask)
-500700<ENTER> - remove CAN-IDs 0x500 - 0x5FF
+400600<ENTER> - add CAN-IDs 0x400 - 0x5FF
+000000<ENTER> - add all CAN-IDs
-000000<ENTER> - remove all CAN-IDs

= In order to record this type of received CAN data to file (including timestamp), use:

$ candump -l vcan0

The resulting file will be named like: candump-2015-03-20_123001.log

= In order to print logfiles in a user friendly format:

$ log2asc -I candump-2015-03-20_123001.log vcan0

= Recorded CAN log files can also be re-played back to the same or another CAN interface:

$ canplayer -I candump-2015-03-20_123001.log

= If you need to use another can interface than defined in the logfile, use the expression CANinterfaceToUse=CANinterfaceInFile. This example also prints the frames:

$ canplayer vcan0=can1 -v -I candump-2015-03-20_123001.log

https://sgframework.readthedocs.io/en/latest/cantutorial.html





### Resultados Ford Fusion

216 - odometro das rodas (ja no codigo)
76  - angulo do volante (ja no codigo)
230 - cambio: 
  can0  230   [8]  00 04 00 00 8E 00 00 00 - neutro
  can0  230   [8]  10 06 00 00 8E 00 00 00 - drive
  can0  230   [8]  E0 02 00 00 8E 00 00 00 - reh
  can0  230   [8]  00 00 00 00 8E 00 00 00 - park

171 - cambio: 
  can0  171   [8]  14 0C 00 00 00 00 00 00 - park
  can0  171   [8]  14 2C 00 00 00 00 00 00 - reh
  can0  171   [8]  14 4C 00 00 00 00 00 00 - neutro
  can0  171   [8]  14 6C 00 00 00 00 00 00 - drive
 
4B0 - Freio (esforcco aplicado)
  can0  4B0   [8]  F9 00 80 00 C0 00 00 F3 - zero esforco
  can0  4B0   [8]  F7 00 80 00 30 00 B0 E6 - pouco esforcco
  can0  4B0   [8]  F7 00 80 00 00 0B 58 93 - medio esforcco
  can0  4B0   [8]  F9 00 80 00 D0 29 60 C1 - muito esforcco

82 - volante
  can0  082   [8]  92 08 14 08 97 80 40 00 - zero
  can0  082   [8]  52 08 15 20 84 80 40 00 - 90 graus a direita
  can0  082   [8]  51 08 15 9C 82 80 40 00 - 180 graus a direita
  can0  082   [8]  AE 08 16 20 80 80 40 00 - 90 graus a esquerda
  can0  082   [8]  AE 08 16 28 7F 80 40 00 - 180 graus a esquerda



