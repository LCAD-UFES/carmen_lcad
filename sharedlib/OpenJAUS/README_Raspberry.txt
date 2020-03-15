== Para colocar para operar o OpenJaus no Raspberry Pi ==
- Instale uma versao do Raspibian >= 2017-09-07
- Para ativar a rede wired: https://www.modmypi.com/blog/how-to-give-your-raspberry-pi-a-static-ip-address-update
- Para incluir uma rede WiFi (note que o Raspberry 3 tem hardware de wifi e Bluetooth nativos), edite /etc/wpa_supplicant/wpa_supplicant.conf e inclua sua rede
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
- Em seguida, baixe e suba a interface de rede
sudo ifdown wlan1
sudo ifup wlan1

- Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

1- Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo 
  para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
 cd
 ssh-keygen -t rsa

2- Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo
 cd
 ssh pi@192.168.0.14 mkdir -p .ssh
 cat .ssh/id_rsa.pub | ssh pi@192.168.0.14 'cat >> .ssh/authorized_keys'

3- Teste se funcionou com o comando abaixo
 ssh pi@192.168.0.14 'ls'

- Mude o endereco do servidor de pacotes (do apt-get) comentando a linha existente e adicionando a <linha> abaixo no arquivo indicado abaixo:
 <linha> deb http://linorg.usp.br/raspbian/raspbian/ stretch main contrib non-free rpi
 sudo nano /etc/apt/sources.list 

- Instale editores de codigo
 sudo apt-get install eclipse-cdt
 sudo apt-get install gedit

- Instale libs necessarias
 sudo apt-get install libncurses5-dev
 sudo apt-get install nmap
 sudo apt-get install can-utils

- Baixe a OpenJaus (apenas) do github do LCAD (https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)
 sudo apt-get install subversion
 cd
 svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/OpenJAUS

- Depois de baixado o OpenJAUS como acima, todas as mudanças futuras no github podem ser incorporadas com:
 svn up

- Para subir coisas para o git use o commit do svn (que já sobe as mudanças). Exemplo:
 svn commit -m "adicao de ojNodeManager/IARAnodeManager.conf"

- Mude seu .bashrc incluindo as linhas abaixo (depois, reboote ou inicie um novo bash):
#OpenJaus
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/OpenJAUS/libopenJaus/lib:~/OpenJAUS/libjaus/lib:~/OpenJAUS/ojTorc/lib:~/OpenJAUS/ojIARASim/lib

- Compile o OpenJAUS
 cd OpenJAUS
 make

- No Raspberry, ajuste o processo de boot adicionando as linhas abaixo no fim do arquivo /boot/config.txt
# Desempenho - Alberto
disable_splash=1
force_turbo=1

dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=8000000,interrupt=24
dtoverlay=spi-bcm2835-overlay

- No Raspberry 192.168.0.13 ajuste o processo de boot adicionando as linhas abaixo no arquivo /etc/rc.local antes do "exit 0"
/home/pi/OpenJAUS/ojNodeManager/set.bat
su - pi -c "/home/pi/OpenJAUS/ojNodeManager/run_SteeringBypass.bat"

- No Raspberry 192.168.0.14 ajuste o processo de boot adicionando as linhas abaixo no arquivo /etc/rc.local antes do "exit 0"
/home/pi/OpenJAUS/ojNodeManager/set.bat
/home/pi/OpenJAUS/ojNodeManager/wait_for_IARA_driver_network.bat
su - pi -c "/home/pi/OpenJAUS/ojNodeManager/run_IARA_driver.bat"


= Para desligamento dos Raspberry Pi quando do shoutdown de car01

- Se o root da car01 ainda não tem uma chave pública para acessar os Pi, execute os comando abaixo 
  para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
 sudo su
 cd
 ssh-keygen -t rsa

- Copie a chave pública da car01 para os Pi com os comando abaixo
 cd
 ssh pi@192.168.0.14 mkdir -p .ssh
 ssh pi@192.168.0.13 mkdir -p .ssh
 cat .ssh/id_rsa.pub | ssh pi@192.168.0.14 'cat >> .ssh/authorized_keys'
 cat .ssh/id_rsa.pub | ssh pi@192.168.0.13 'cat >> .ssh/authorized_keys'

- Teste se funcionou com o comando abaixo
 ssh pi@192.168.0.14 'ls'

- Crie o arquivo /etc/init.d/shutdown-raspberrys na car01 e acrescente os comandos abaixo dentro dele:
 #! /bin/sh
 ssh pi@192.168.0.14 -t sudo shutdown -h -P now
 ssh pi@192.168.0.13 -t sudo shutdown -h -P now
 exit 0

- Mude as permissoes do arquivo como abaixo:
 sudo chmod +x /etc/init.d/shutdown-raspberrys

- Crie um link para o arquivo /etc/rc6.d/K99shutdown-raspberrys
 sudo ln -s /etc/init.d/shutdown-raspberrys /etc/rc0.d/K99shutdown-raspberrys

