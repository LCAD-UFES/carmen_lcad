= Instalações
sudo apt-get install can-utils

= Ativar para uso
sudo slcand -o -c -s0 /dev/ttyACM0 can0
sudo ifconfig can0 up

= Teste
candump can0
