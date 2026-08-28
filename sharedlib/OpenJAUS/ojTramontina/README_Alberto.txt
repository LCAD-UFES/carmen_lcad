= ODrive (ver: https://docs.odriverobotics.com/developer-guide). Note que fiz um 
  merge (https://stackoverflow.com/questions/54033842/how-to-pull-a-pull-request-from-upstream-in-github) com: https://github.com/madcowswe/ODrive/pull/258

== Para compilar
cd steering_odrive/ODrive/Firmware {pode dar um tanto de warning}

== Para instalar o firmware novo
odrivetool dfu build/ODriveFirmware.hex

= ESP32
Antes de qualquer coisa rode o script (tem que incluir o "."): 
. $HOME/esp/esp-idf/export.sh

== Para compilar e carrega na placa (tem que apertar e seguarar o botao de boot ate comeccar a gravar no dispositivo, senao pode ficar tentando e falhar):
make flash

== Para monitorar a placa
make monitor



