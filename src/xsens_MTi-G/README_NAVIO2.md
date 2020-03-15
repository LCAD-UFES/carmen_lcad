Para rodar o módulo:
apenas ligue o raspberry com Navio2 e garanta que seu computador tem o IP configurado no navio para receber a mensagem. (IARA 192.168.1.1)
run:
./navio2_mavlink

#Resumo para entender como a placa e o ardupilot funcionam, se não precisar pode pular para Configuração da Placa:
```
A placa Navio tem vários sensores integrados para Drones. 
Para ler e tratar esses sensores (Filtros como EKF, comandos de controle etc) ele usa disponibiliza uma imagem do raspbian já com o Ardupilot instalado. 
O Ardupilot é um framework (tipo o carmen) que contém uma estrutura de módulos que permitem
a operação de drones, carros em escala, e outras aplicações DIY de forma até autônoma.
O Ardupilot foi feito para ser o mais automático possível, sem a necessidade de codificação, por isso ele se conecta na maioria dos 
Ground Control Stations (GCS) que são softwares para controle
de operação de drones via interface gráfica, é possível por exemplo traçar uma rota pela interface e os módulos do Ardupilot executam essa rota usando os Sensores do Navio2 e demais sensores e motores 
conectados a ele e ao raspberry. (Como rodar o proccontrol e setar uma rota para a IARA no navigator_gui)
Para se comunicar com os GCS o ardupilot usa o midleware Mavlink (tipo um IPC mesmo) ele define mensagens e controla a comunicação.
A comunicação pode ser feita via UDP, TCP ou Serial. Usamos UDP no módulo carmen.
A forma usada aqui foi usando uma biblioteca em c do mavlink (https://github.com/mavlink/c_library_v2) e comunicando via UDP na porta padrão 14550 com o ardupilot.
As definições das mensagens podem ser encontradas em: https://mavlink.io/en/messages/common.html
Os padrões publicados pelo Ardupilot são:https://ardupilot.org/dev/docs/mavlink-requesting-data.html
Um overview desse processo descrito acima pode ser visto em:https://ardupilot.org/dev/docs/mavlink-basics.html
Um detalhe é que o Ardupilot instalado no raspberry com Navio só publica mensagens se for feita uma primeira solicitação (REQUEST)

Após tudo configurado, sempre que o raspberry for ligado ele iniciará automaticamente o ardupilot. Para evitar problemas na comunicação, aguarde ele dar o boot totalmente.
```

#Passos resumidos:
        -Criar cartão SD com a imagem sugerida pela emlid.
        -Plug o NAVIO2 no raspberry (siga as instruções no site)
        -O raspberry alimentará o NAVIO2
        -Configure a rede wifi e/ou USB (copiado do tutorial src/pi_camera/README.md)
        -Use o código em ~/carmen_lcad/src/xsens_MTi-G/navio2_mavlink_main.cpp para receber as mensagens

#Configuração da Placa:

Os passos abaixo estão descritos no tutorial disponível em:
https://doc.emlid.com.br/navio2/

####Download da imagem Raspbian configurada
  http://files.emlid.com/images/emlid-raspbian-20190227.img.xz
  [Link do Drive versão da sensorbox]

####Escrever imagem no cartão SD

Baixe, o aplicativo Etcher extraia e execute no Etcher como administrador.
  https://etcher.io/

Selecione o arquivo com a imagem e a letra da unidade do cartão SD.

Clique em “Flash!”. O processo pode demorar alguns minutos.

Instruções mais detalhadas estão disponíveis [aqui](http://www.raspberrypi.org/documentation/installation/installing-images/).

#### Configure an Static IP to the Raspberry PI on IARA's network (src/pi_camera/README.md)
 
 Start by editing the dhcpcd.conf file
 
```bash
 $ sudo nano /etc/dhcpcd.conf
```

 Add at the end of the file the following configuration:
 eth0 = wired connection
 For the first pi_camera we used the IP adress 192.168.0.15/24
 Replace the 15 with the IP number desired
 Make sure you leave the /24 at the end of the adress
 
```
 interface eth0

 static ip_address=192.168.1.15/24
 static routers=192.168.1.1
 static domain_name_servers=8.8.8.8
```

 To exit the editor, press ctrl+x
 To save your changes press the letter “Y” then hit enter
 
 To disable WiFi and bluetooth edit:
 
 ```bash
 sudo nano /boot/config.txt
```
add the lines:

```
 dtoverlay=pi3-disable-wifi
 dtoverlay=pi3-disable-bt
```
 use the prefix "pi3-" if you are using a raspberry 3
 
 Reboot the Raspberry PI
 
```bash
 $ sudo reboot
```

 You can double check by typing:
 
```bash
 $ ifconfig
```

 The eth0 inet addr must be 192.168.0.15


####Configuração do Ardupilot 
Ligue o raspberry e connect ou via SSH ou usando monitor e teclado

Você pode seguir o tutorial com imagens em [Instalação e Execução](https://doc.emlid.com.br/navio2/common/ardupilot/installation-and-running/)

ou seguir abaixo:

Para usuários avançados faça:
Para selecionar o veículo que seria lançado por padrão. você deve configurá-lo com o emlidtool:

  sudo emlidtool ardupilot
  
Antes da configuração, o emlidtool verifica seu RCIO firmware and will suggest to update it if you have the outdated one:

Ao rodar "sudo emlidtool ardupilot" Foi solicitado atualização do firmware, foi feita com sucesso [ o firmware atual da IARA é o ArduCopter: 3.6.5 ]

Após atualização desligue totalmente o raspberry e Navio da energia para subir as configurações
Ligue e faça novamente:
  sudo emlidtool ardupilot
Vamos usar o arducopter, mas poderia ter sido bem arduplane oo ardurover. Quando o comando estiver em execução selecione:

```
Choose your vehicle:
< copter >

Choose your version:
< 3.6  >

```


em Conectando-se ao GCS usei o QGroundControl
        Instalação em: https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html
Parece ter problemas para calibrar via Wifi então mudei para o Mission Planner
Tentei o Mission Planner 
https://discuss.ardupilot.org/t/running-mission-planner-on-linux/19100
Funciona para calibrar e ver os angulos

Para usa-lo, garanta que o navio esteja rodando o ardupilot, ele conectará automaticamente (se você tiver setado o IP do PC corretamente na etapa de instalação)

para instalar o Mission Planner:
1- you need to install Mono from the instructions on this page. 
This will install the latest version older versions or the one that installs with apt will not work properly.
 http://www.mono-project.com/download/#download-lin I installed mono-complete. 
Tried a couple of the other recommendations but they were already installed with complete.
2 - Get the latest beta version of MP and unzip it into a directory. 
MissionPlannerBeta.zip from here http://firmware.us.ardupilot.org/Tools/MissionPlanner/
I got the MissionPlanner-1.3.68.zip.
3 - command to start up is sudo mono MissionPlanner.exe. 
Takes some extra time first time through.
 Probably building p-code for the emulator. 
Once you run it once it comes up like you are used to seeing.

OBS:. Sobre a calibração:
The calibrations are saved in flash, so there is no need to recalibrate every time. I am not sure whether examples could read them from the .stg files or not, but you can manually extract the values by reading the full parameter list in Mission Planner and then find a way to pass them to the example.
The calibration data is stored in a non-volatile memory (namely /var/APM/<vehicle>.stg) for sure! Otherwise you’d need to calibrate your vehicle on every launch.

Dá para comunicar com o Ardupilot usando MAVlink (protocolo tipo ipc)
https://mavlink.io/en/messages/common.html#ATTITUDE

Para calibrar seguir o tutorial Complete the Calibration Wizards usando MissionPlanner
https://drive.google.com/open?id=1TTaNSh_-00v7FKRkSYc-cY1bS4gaTRXz
Faça: Frame Type (Copter Only), Accelerometer e Compass.

Calibraton compass video
https://youtu.be/DmsueBS0J3E


Não usei esse modo abaixo, apenas para referencia de como o pessoal faz a calibração na mão, os passos anteriores parece suficientes.
Além disso na forma abaixo você terá que sempre atualizar os valores raw direto no seu codigo para cada novo dado recebido.
Matriz de calibração:
https://diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A1676387&amp;commentId=705844%3AComment%3A2063925&amp;xg_source=activity


Para testar os sensores do Navio2 sem usar o Ardupilot:
No navio rode o  AHRS com o ip da maquina que irá receber
No PC rode o Navio2/Utilites/3D.py
https://docs.emlid.com/navio2/common/dev/ahrs/
These Examples don’t use calibration data, though.

