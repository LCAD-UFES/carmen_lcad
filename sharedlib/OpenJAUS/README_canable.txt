= Instalações
sudo apt-get install can-utils

= Para usar o firmware candleLight (bem melhor) ver documentação em: carmen_lcad/sharedlib/OpenJAUS/ojEcoTech4-2/canable/README_Alberto.txt

sudo ip link set can0 up type can bitrate 500000

= Ativar para uso (firmware velho; ver acima)
sudo slcand -o -c -s6 /dev/ttyACM0 can0

Hitech ecoTech can funciona a 250kbps usar -s5 (Veja a tabela no final do arquivo caso o bitrate seja diferente de 250 ou 500)
	sudo slcand -o -c -s5 /dev/ttyACM0 can0

sudo ifconfig can0 up

= Teste
candump can0

cansend can0 999#DEADBEEF   # Send a frame to 0x999 with payload 0xdeadbeef
candump can0                # Show all traffic received by can0
canbusload can0 500000      # Calculate bus loading percentage on can0 
cangen can0 -D 11223344DEADBEEF -L 8    # Generate fixed-data CAN messages

cansniffer -c can0          # Display top-style view of can traffic
-m <mask>       (initial FILTER default 0x00000000)
-v <value>      (initial FILTER default 0x00000000)
-q              (quiet - all IDs deactivated)
-c              (color changes)

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




Bring Virtual CAN Interface up
 $ sudo modprobe vcan
 $ sudo ip link add dev vcan0 type vcan
 $ sudo ip link set up vcan0
 $ candump vcan0


cansniffer -c can0 -m 80 81 2B0 111 112

### Hyundai HB20
Steering (Bytes 0 e 1 - Alngle)
ID   data
2B0  98 13 00 07 62 - Tudo para a esquerda
2B0  00 00 00 07 AF - Steering ZERO
2B0  74 EC 00 07 42 - Tudo para a direita

Velocity
ID   data
52A  01 00 00 00 00 00 01 00 (Bytes 0 e 1 - Alngle # also bytes 6 and 7)


### Toyota Etios
ID  data
25  0F C2 0F FE 50 00 00 5B -  90 degrees Right Steering (Bytes 0 e 1 - Alngle of Position) (Bytes 6 e 7 Torck)
25  0F 87 0F FE 90 00 00 60 - 180 degrees Right
25  0F 49 0F FE 30 00 00 C2 - 270 degrees Right
25  0F 0E 0F FE A0 00 00 F7 - 360 degrees Right
25  0F 0E 0F FE A0 00 00 F7 - 450 degrees Right
25  0E 96 0F FE 00 00 00 DE - 540 degrees Right
25  0E 59 0F FE E0 00 00 81 - 630 degrees Right
25  0E 36 0F FE 60 00 00 DE - 675 degrees Right
25  00 00 0F FE 30 00 00 6A - Steering ZERO
25  00 3A 0F FE 00 00 00 74 -  90 degrees Left
25  00 78 0F FE 90 00 00 42 - 180 degrees Left
25  00 B1 0F FE 30 00 00 1B - 270 degrees Left
25  00 EF 0F FE B0 00 00 D9 - 360 degrees Left 
25  01 29 0F FE 90 00 00 F4 - 450 degrees Left 
25  01 29 0F FE 90 00 00 F4 - 540 degrees Left 
25  01 A2 0F FE 50 00 00 2D - 630 degrees Left 
25  01 D9 0F FE 30 00 00 44 - 710 degrees Left

1D0  08 D5 09 00 00 00 00 BF - Gear Park         (Byte 5 indicates the Gerar) (Bytes 6 and 7 indicates aceleration)
1D0  00 00 02 00 00 00 00 DB - Gear Rearward
1D0  0C 58 0C 00 00 00 00 49 - Gear Drive
1D0  00 00 10 00 00 00 00 E9 - Gear Neutral

49F  00 00 00 02 00 00 20 00 - Gear Park         (Byte 4 indicates the Gerar) (Bytes 0 and 1 indicates aceleration)
49F  00 00 00 01 00 00 20 00 - Gear Rearward
49F  00 00 00 0A 00 00 20 00 - Gear Drive
49F  00 00 00 02 00 00 00 96 - Gear Neutral    (Aceleration at 3000 RPM)

AA - Velocity
B4 - Velocity



### Mensagens Cambio SARA Black - Sara branca muda para 00 01 02 respectivamente
               __
 10F8109A [8]  04 00 00 00 00 00 00 5A - Neutro
               __
 10F8109A [8]  05 00 00 00 00 00 00 5A - Drive
               __
 10F8109A [8]  06 00 00 00 00 00 00 5A - Rear

### Mensagens Velocidade SARA - mph
                  __ __
 10F8109A [8]  05 00 00 00 00 00 00 5A - Drive - Velocity
Ex: zero para velocidade
                    __ __
 10F8109A   [8]  06 00 00 00 00 00 00 5A - Rear - Velocity
                    __ __ 
 10F8109A   [8]  06 03 00 00 00 00 00 5A - Rear - Velocity


### Mensagens publicadas direcao FOX
 0C2
 0D0
 3D0
 729
 

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


TABELA BITRATE COMANDO slcand
s0 	10 Kbit/s
s1 	20 Kbit/s
s2 	50 Kbit/s
s3 	100 Kbit/s
s4 	125 Kbit/s
s5 	250 Kbit/s
s6 	500 Kbit/s
s7 	800 Kbit/s
s8 	1000 Kbit/s 
