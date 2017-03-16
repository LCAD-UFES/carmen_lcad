== Acesso a placa (RaspberryPi) via car01

- Ajuste a rede de car01 para Carro Network (o carro nao precisa estar ligado para conectar)
- ssh -XC pi@192.168.0.13, senha: 1q2w3e4r
- O -XC acima permite rodar aplicativos graficos instalados na placa remotamente (via terminal na car01)

== Para acessar a Internet da placa
- Ligue um WiFi dongle na placa e acesse a Internet_Lenta_02 (senha produshow2015) ou outra rede WiFi

== O hardware da placa segue o circuito do PICAN2 Duo (http://skpang.co.uk/catalog/pican2-duo-canbus-board-for-raspberry-pi-23-p-1480.html)
- Datasheet: PICAN2DUODSB.pdf
- Diagrama: pican_duo_rev_B2.pdf
- Foi necessario adicionar hardware para corrigir um problema de corrida no driver do Linux (https://github.com/raspberrypi/linux/issues/1490)
-- Ver hardware_adicional.png e 74HC00.pdf

== Trocar os cristais das interfaces CAN para 16MHz

== Ajustar na placa o arquivo /boot/config.txt para conter no fim:

dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
dtoverlay=spi-bcm2835-overlay

-- Ver PICAN2_jessie_2016-05-10.pdf (mas confie nos dados acima para cristais de 16MHz)

== Tudo ajustado como acima (interfaces CAN ligadas na placa e Linux da placa configurado), pode-se acessar a placa. Para isso:
-- Instale o can-utils na placa
--- sudo apt-get install can-utils
-- O can-utils eh descrito em https://fabiobaltieri.com/2013/07/23/hacking-into-a-vehicle-can-bus-toyothack-and-socketcan/,
https://www.kernel.org/doc/Documentation/networking/can.txt e o dump (history) em arquivos txt neste diretorio
-- Para entender filtros a mascaras can: http://www.cse.dmu.ac.uk/~eg/tele/CanbusIDandMask.html (ver tambem wikipedia: https://en.wikipedia.org/wiki/CAN_bus)
