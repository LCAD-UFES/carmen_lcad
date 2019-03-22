# Install Raspbian on the Raspberry PI

- Baixe a imagem do sistema RASPBIAN STRETCH WITH DESKTOP no site do raspberry (https://www.raspberrypi.org/downloads/raspbian/)
Obs: Nao baixe o arquivo LITE pois este possui apenas interface por linha de comando.

- Será necessário identificar o nome do dispositivo do cartão SD. Para isto, antes de inserir o cartão de memória execute o seguinte comando para ver as unidades de disco presentes.

```bash
 $ sudo fdisk -l
 ou
 $ ls -la /dev/sd*
```
- Insira o cartão e execute novamente o comando de forma a identificar o nome do cartao de SD que irá aparecer. Será algo do tipo /dev/sd...

```bash
 $ sudo fdisk -l
```

- Caso o cartão SD apareça como por exemplo /dev/sde1 ou /dev/sde2, eliminar o numero e considerar apenas /dev/sde 

- Execute o seguinte comando para copiar a imagem do Raspibian para o cartão SD fazendo as alterações de caminho necessárias.

```bash
 $ sudo dd if=/home/usr/Downloads/2018-04-18-raspbian-stretch.img of=/dev/sd...
```

- O cartão esta formatado e pode ser inserido no Raspberry para utilização.


# Enable the Camera and SSH

- Não execute upgrade

```bash
 $ sudo apt-get update
 $ sudo raspi-config
```
 Acesse e habilite a camera:
 
 - Interfacing Options->Camera
 - Interfacing Options->SSH

Teste usando o comando: 

```bash
 $ raspistill -v -o test.jpg
 $ raspivid -o teste.h264 -t 10000
```
raspistill grava uma imgem e raspivid grava um video

# Install Dependencies anf Download the pi_camera file from git

```bash
 $ sudo apt-get install libopencv-dev python-opencv
 $ sudo apt-get install cmake
 $ sudo apt-get install subversion
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/pi_camera
```


# Compile and test the pi_camera module on the Raspberry PI

```bash
 $ cd pi_camera/raspicam && mkdir build && cd build && cmake ..
 $ make
 $ sudo make install
 $ cd ../.. && mkdir build && cd build && cmake ..
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
 
```
 interface eth0

 static ip_address=192.168.1.15/24
 static routers=192.168.1.1
 static domain_name_servers=8.8.8.8
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
 
# Test the pi_camera on IARA
 
 Run the server on the Raspberry PI (You can access the raspbery pi from IARA using ssh to facilitate)
 
 ```bash
 $ ssh pi@192.168.1.15
 $ cd ~/pi_camera/build
 $ ./pi_camera_server_driver
```
 Run the viewer on IARA
 
 ```bash
 $ cd ~/carmen_lcad/bin
 $ ./central
```
 Open another terminal window (on terminal press Ctrl+Shift+t)
 
```bash
 $ ./proccontrol process-pi-camera-viewer.ini
```