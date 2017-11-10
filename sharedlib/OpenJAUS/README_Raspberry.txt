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

2- Teste se funcionou com o comando abaixo
 ssh pi@192.168.0.14 'ls'

- Mude o endereco do servidor de pacotes (do apt-get) comentando a linha existente e adicionando a <linha> abaixo no arquivo indicado abaixo:
 <linha> deb http://linorg.usp.br/raspbian/raspbian/ stretch main contrib non-free rpi
 sudo nano /etc/apt/sources.list 
	
- Instale a lib curses
 sudo apt-get install libncurses5-dev

- Baixe a OpenJaus (apenas) do github do LCAD (https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)
 sudo apt-get install subversion
 cd
 svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/OpenJAUS

- Depois de baixado o OpenJAUS como acima, todas as mudanças futuras no github podem ser incorporadas com:
 svn up

- Para subir coisas para o git use o commit do svn (que já sobe as mudanças). Exemplo:
 svn commit -m "adicao de ojNodeManager/IARAnodeManager.conf"

- Compile o OpenJAUS
 cd OpenJAUS
 make

- Mude seu .bashrc incluindo as linhas abaixo:
#OpenJaus
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/OpenJAUS/libopenJaus/lib:~/OpenJAUS/libjaus/lib:~/OpenJAUS/ojTorc/lib:~/OpenJAUS/ojIARASim/lib
