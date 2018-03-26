== Para colocar para operar o Voice Kit - Raspberry Pi ==

- Instale uma versao do Raspibian >= 2017-09-07
- Conectar os cabos ligando Raspberry e Voice Hat como mostrado em: https://aiyprojects.withgoogle.com/voice/#assembly-guide-2-assemble-the-hardware
- Use um cartao micro SD para gravar a ISO do Voice Kit. (Download em: magpi.cc/2x7JQfS). (Gravacao da ISO com ETCHER: magpi.cc/2fZkyJD).
- Conecte os perifericos necessarios: Teclado USB, Mouse USB, Cabo HDMI, cabo de energia (Ex.:celular, corrente>=2,0 A).
- Com o boot do Raspberry Pi, o led dentro da caixa se acende. 
	Caso apareca "Openbox Syntax Error", voce tera que reescrever a imagem no cartao SD.
- Clique duas vezes no icone "Check Audio". Voce ouvira "Front, Centre" e uma mensagem na tela. Responda de acordo as instrucoes. 
	Em caso de erro, siga a solucao mostrada na mensagem.


== ATIVANDO A REDE ==

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
	
- Instale libs necessarias
 sudo apt-get install libncurses5-dev
 sudo apt-get install nmap
 sudo apt-get install can-utils

- Baixe a OpenJaus (apenas) do github do LCAD (https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)
 sudo apt-get install subversion
 cd
 svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/voice_kit_aiy



== SUBINDO ARQUIVOS PARA GITHUB/CARMEN == 

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



== CRIANDO PRIMEIRO PROJETO E CREDENCIAIS == 

Se quiser, siga os passos em ingles em: https://aiyprojects.withgoogle.com/voice/#users-guide-1-1--connect-to-google-cloud-platform

*utilize o Chromium*

- Faca um conta no Google Cloud Plataform (GCP).
- Entre no Console do Cloud com sua ID e senha: https://console.cloud.google.com e siga os passos abaixo:
	1- Crie projeto (acima, esquerda);
	2- Clique em "Produtos e Servicos" (linha tripla)
	3- Escolha "APIS e Servicos"
	4- Procure por "Google Assistant API" e clique nele
	5- Clique em "ENABLE"
	6- Ainda em "APIS e Servicos" va em "Credentials" e crie uma credencial. Escolha "OAuth client ID"
	7- Va em " Configure consent screen"
	8- Entre com um nome do "produto" (Ex.: Voice-Assistant) e salve.
	9- Clique em "Outros". Troque de nome que lembre a credencial (Ex.: Voice Recongnizer)
	10- Feche a Pop-up que ira aparecer. Nao precisa guardar os numeros.
- Faca o download da credencial. O nome comecara com "client_secrets..." e estara na pasta Download.
- Renomeie a credencial para 'assistant.json' e mova para a pasta '/home/pi'
	- Abra o terminal:
		cd Downloas
		mv client_secret..... /home/pi/assistant.json
- Ative os controles do dispositivo conectado com sua Google ID em: https://myaccount.google.com/activitycontrols	
	-Deixe ligado os segui
		-Web and app acitivy. Inclua o checkbox de "Incluir historico de busca do Google...."
		-Device Information
		-Voice and audio activity

== TESTANDO DEMOS ==

Tambem em: https://aiyprojects.withgoogle.com/voice/#users-guide-3-1--start-the-assistant-library-demo-app

- Em '/home/pi', tanto em 'AIY-voice-kit-python', quanto 'AIY-projects-python' e 'assistant-sdk-python' terao uma pasta '/src/examples/voice' com suas demos.
- Teste primeiro o "assistant_library_demo.py"
- Se Python 3.5, para nao dar error na demo "assistant_grpc_demo.py", siga os passos:
		- Renomeie uma pasta 'futures' para 'oldfutures' dentro da '/bin':
			1- No terminal:
				cd /usr/local/lib/python3.5/dist-packages/concurrent
				sudo mv futures oldfutures
		- Re-teste a demo.


	

	

-------------------------
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
