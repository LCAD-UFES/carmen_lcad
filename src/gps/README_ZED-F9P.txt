Driver antigo
colocar a porta que foi conectada no carmen.ini (/dev/USB..)
mudar o comando str2str para velocidade de 36800
Checar se está publicando o GPS1 no driver e colocar para pegar qual nº publicar via parâmetro.

Para rodar o GPS ublox ZED-F9P 
Liste todas as portas USBs:

ls /dev/tty*

conecte o gps na porta USB e liste novamente e encontre a que apareceu nova
ls /dev/tty*
Na minha o GPS foi reconhecido como /dev/ttyACM0

Altere no carmen.ini do seu carro o parametro que recebe a porta do GPS (usa o driver antigo gps_nmea)

gps_nmea_dev  /dev/ttyACM0

Ex:
../src/carmen-ford-escape-sensorbox.ini

Rode o process 
e rode dos drivers:
 ./gps_nmea
Para enviar a correção RTK (lembre-se de mudar a porta nos comandos):
str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyACM0:38400:8:n:1:off

Para checar se esta passando mensagem rode:
./gps_test
Exemplo para adicionar no process:

gps			sensors 	0		0			./gps_nmea
 gps_correction		sensors	 	0		0			str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyACM0:38400:8:n:1:off

