# Istall Raspbian on the Raspberry PI

- Baixe o arquivo imagem do sistema operacional "Raspbian" no site do raspberry (https://www.raspberrypi.org/downloads/raspbian/)
Obs: Nao baixe o arquivo LITE, e sim a versao completa.

- Baixe o Etcher para formatar o cartao de memoria utilizando o arquivo imagem citado na instrucao acima.
Link: (https://etcher.io/)

- Execute o Etcher e selecione o cartao de memoria e o arquivo imagem baixado no site do Raspberry pi

No final do processo o cartao vai estar pronto para ser inserido no Raspberry pi 3, e o mesmo estara pronto para utilizacao.

== Instrucoes iniciais para a utilizacao do Raspberry pi 3 (Usando a camera)

- Conecte seu Raspberry pi 3 em uma rede Wireless ou Wired(A internet sera um recurso necessario para, eventualmente, baixar-
mos pacotes e programas necessarios para a devida utilizacao do Script "rpi_camera" e de outras ocasionais funcionalidades)


# Enable the Camera

1º-  Clique no menu inicar do Raspiberry pi 3(Canto superior esquerdo da tela inicial) e acesse a secao "Preferencias". Selecione
"Raspiberry Pi Configuration"

2º- Na janela que se abrira selecione a aba "Interfaces"

3º- Habilite a Camera e o SSH.

Sua camera esta habilitada e pronta pra uso.

Obs: Sera necessario reiniciar o sistema apos este passo.

Caso queira testar se a camera ja esta pronta pra uso, teste o seguinte comando no terminal:

"raspivid -o teste.h264 -t 10000"

Se tudo estiver funcionando corretamente a camera sera ativada e gravara um video de 10 segundos que sera salvo na pasta:
"home/pi"

Obs: Para consultar os comandos no terminal utilizados pela Pi Camera consulte a documentacao no link:
https://www.raspberrypi.org/documentation/raspbian/applications/camera.md


# Install Dependencies anf Download the pi_camera file from git

```bash
 $ sudo apt-get install cmake
 $ sudo apt-get install libopencv-dev python-opencv
 $ sudo apt-get install subversion
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/pi_camera
```


# Compile and test the pi_camera module on the Raspberry PI

```bash
 $ cd pi_camera/raspicam
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make
 $ sudo make install
 $ cd ../..
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make
 $ ./pi_camera_test
```

 The pi_camera_test program will run the camera and display the image captured using OpenCv


# Configure an Static IP to the Raspberry PI on IARA's network
 
 Start by editing the dhcpcd.conf file
 
```bash
 $ sudo nano /etc/dhcpcd.conf
```

 Add at the end of the file the following configuration:
 eth0 = wired connection
 For the first pi_camera we used the IP adress 192.168.0.15/24
 Replace the 15 with the IP number desired
 Make sure you leave the /24 at the end of the adress
 
```ini
 interface eth0

 static ip_address=192.168.0.15/24
 static routers=192.168.0.1
 static domain_name_servers=192.168.0.1
``` 

 To exit the editor, press ctrl+x
 To save your changes press the letter “Y” then hit enter
 
 Reboot the Raspberry PI
 
```bash
 $ sudo reboot
```

 You can double check by typing:
 
```bash
 $ ifconfig
```

 The eth0 inet addr must be 192.168.0.15
