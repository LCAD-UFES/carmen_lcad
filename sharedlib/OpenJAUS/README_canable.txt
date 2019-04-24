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

cansniffer can0             # Display top-style view of can traffic

The filter of cansniffer FILTER can be a single CAN-ID or a CAN-ID/Bitmask:
+1F5<ENTER>    - add CAN-ID 0x1F5
-42E<ENTER>    - remove CAN-ID 0x42E
-42E7FF<ENTER> - remove CAN-ID 0x42E (using Bitmask)
-500700<ENTER> - remove CAN-IDs 0x500 - 0x5FF
+400600<ENTER> - add CAN-IDs 0x400 - 0x5FF
+000000<ENTER> - add all CAN-IDs
-000000<ENTER> - remove all CAN-IDs