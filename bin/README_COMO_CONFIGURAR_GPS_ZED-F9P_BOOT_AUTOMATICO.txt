= Para linkar o GPS ZEDF9 DUAL ANTENA a uma porta fixa (em vez de ficar tendo que encontrar a /dev/ttyACMX ou etc

  Crie o arquivo /etc/udev/rules.d/99-gps-ZED-F9P.rules com o conteúdo:
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", ATTRS{serial}=="0", SYMLINK+="gps_ZED_F9P_back", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", ATTRS{serial}=="1", SYMLINK+="gps_ZED_F9P_front", MODE="0666"

  O serial é o número de série do gps, muito importante identificar a antena traseira e dianteira para colocar na linha correta do comando acima.
 Para saber qual é, conente o GPS que está com a antena traseira e digite:
 dmesg
  
  Vai aparecer no final dos print do dmesg o conteúdo com esses campos:
[ 4894.968469] usb 1-1: Product: u-blox GNSS receiver
[ 4894.968472] usb 1-1: Manufacturer: u-blox AG - www.u-blox.com
[ 4894.968474] usb 1-1: SerialNumber: 0

  Use o SerialNumber no serial do arquivo que você criou na linha do SYMLINK+="gps_ZED_F9P_back"
Faça o mesmo processo para o GPS com antena dianteira.
APROVEITE PARA VERIFICAR SE OS OUTROS CAMPOS DO CONTEUDO NO ARQUIVO /etc/udev/rules.d/99-gps-ZED-F9P.rules batem com a saída do dmesg.
 
  Para testar, reinicie a máquina.
  * Alternativamente, remova os GPS das USBs e digite o comando:
 sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger

