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


# Enable the IMU interface e OpenGL

- Não execute upgrade

```bash
 $ sudo raspi-config
```

Entre em Interface Options e ative o I2C e o SPI. Volte e entre em Advanced Options -> GL Driver -> GL (Full KMS) -> Ok. Reboot.

```bash
 $ sudo apt-get install mesa-utils
 $ glxgears {para testar o OpenGL}
```


# How to Enable i2c on the Raspberry Pi

```bash
$ sudo apt-get update
$ sudo apt-get install gedit
$ sudo gedit /etc/modprobe.d/raspi-blacklist.conf
```

Place a hash '#' in front of blacklist i2c-bcm2708
If the above file is blank or doesn't exist, then skip the above step


```bash
$ sudo gedit /etc/modules
```

Add these two lines;

i2c-dev
i2c-bcm2708

```bash
$ sudo gedit /boot/config.txt
```

Add these two lines to the bottom of the file:
dtparam=i2c_arm=on,i2c_arm_baudrate=1000000
dtparam=i2c1=on

{,i2c_arm_baudrate=1000000 eh para a MPU-9250, mas deve funcionar com outras. Se nao funcionar, retirar}

```bash
$ sudo reboot
```

Once your Raspberry Pi reboots, you can check for any components connected to the i2c bus by using i2cdetect;

```bash
$ sudo /usr/sbin/i2cdetect -y 1
```

A table like the table below will be shown and if any divices are connected, thier address will be shown. 
Below you can see that a device is connected to the i2c bus which is using the address of 0x6b.

```bash
0 1 2 3 4 5 6 7 8 9 a b c d e f
00: -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
70: -- -- -- -- -- -- -- --
```

# Install Dependencies

```bash
 $ sudo apt-get update
 $ sudo apt-get install i2c-tools libi2c-dev python-smbus
 $ sudo apt-get install liboctave-dev
 $ sudo apt-get install subversion
 $ sudo apt install setserial
 $ sudo apt-get install freeglut3 freeglut3-dev
 $ sudo apt-get install gedit
 $ sudo apt-get install eclipse eclipse-cdt
 $ sudo apt-get install cmake
 $ sudo apt-get install qt4-dev-tools
 $ sudo apt-get install qtcreator
 $ sudo apt-get install libgtk-3-dev

 ```

# Install carmen_lcad sources

```bash
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/ ~/carmen_lcad/src
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/include/ ~/carmen_lcad/include
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/libcmt/ ~/carmen_lcad/sharedlib/libcmt
```

- Baixe e compile uma versão mais atual do IPC

```bash
 $ cd /usr/local
 $ sudo wget http://www.cs.cmu.edu/afs/cs/project/TCA/ftp/ipc-3.9.1a.tar.gz
 $ sudo tar -xzvf ipc-3.9.1a.tar.gz
 $ cd ipc-3.9.1/src/
 $ sudo cp ~/carmen_lcad/src/xsens_MTi-G/formatters.h .
 $ sudo make
```

- Substitua o arquivo Makefile.rules do src do carmen

```bash
 $ cp ~/carmen_lcad/src/xsens_MTi-G/Makefile.rules ~/carmen_lcad/src/
```

# Configure CARMEN LCAD

```bash
 $ cd ~/carmen_lcad/src
 $ ./configure --nojava --nozlib --nocuda --nographics
 Should the C++ tools be installed for CARMEN: [Y/n] Y
 Should Python Bindings be installed: [y/N] N
 Should the old laser server be used instead of the new one: [y/N] N
 Install path [/usr/local/]: 
 Robot numbers [*]: 1,2
```

# Compile and executing the pi_imu drive module on the Raspberry PI

```bash
 $ 
 $ cd ~/carmen_lcad/src/pi_imu/RTIMULib2/Linux
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make
 $ cd ~/carmen_lcad/src/pi_imu/pi_imu_server
 $ make
 $ ./Output/pi_imu_server_driver 
```

Dependendo da IMU, substitua o arquivo pi_imu_server/RTIMULib.ini por pi_imu_server/RTIMULib-LSM9DS1.ini ou outro
disponivel no diretorio.

# Testing and calibrating the pi_imu drive module on the Raspberry PI

```bash
 $ cd ~/carmen_lcad/src/pi_imu/RTIMULib2
 $ ./Linux/build/RTIMULibDemoGL/RTIMULibDemoGL 
 {or ./Linux/build/RTIMULibDemo/RTIMULibDemo if OpenGL is not working}
 {or ./Linux/build/RTIMULibCal/RTIMULibCal for terminal only calibration}
 ```

For calibration see ~/carmen_lcad/src/pi_imu/RTIMULib2/Calibration.pdf

O arquivo de calibracao, apos rodar o comando acima, ficarah no diretorio corrente 
(~/carmen_lcad/src/pi_imu/RTIMULib2) e se chama RTIMULib.ini. Copie este arquivo para
~/carmen_lcad/src/pi_imu/pi_imu_server quando obtiver uma boa calibracao. Para saber se obteve,
continue lendo a seguir.

O carmen_lcad usa o sistema de coordenadas da mao direita: dedo indicador para frente (x), medio para a direita (y)
e polegar para cima (z).

Para saber se o sistema de coordenada de sua IMU estah correto para o carmen_lcad, rode o programa acima
e verifique se a tela estah igual aa mostrada no arquivo correct_coordinate_system.png quando a IMU
tem seu eixo x apontado para o norte (use uma bussola para saber para onde eh o norte; certique-se de estar
longe de materiais muito magneticos).

Para chegar a uma configuracao correta como a do arquivo correct_coordinate_system.png, calibre o melhor
que puder a IMU seguindo ~/carmen_lcad/src/pi_imu/RTIMULib2/Calibration.pdf e verique se:

1- Quando os eixos x, y e z da IMU apontam para cima (cada um, individualmente), o valor de aceleracao 
(Accelerometers (g): em RTIMULibDemoGL) de cada eixo medido por RTIMULibDemoGL eh proximo de 1.0 (e nao -1.0).

2- Quando olhando para um dos eixos x, y ou z (vire o eixo da IMU na direcao do seu rosto) e girando
a IMU no sentido anti-horario, o valor da velocidade angular (Gyros (radians/s): em RTIMULibDemoGL) deste 
eixo eh positivo.

3- Quando os eixos x, y e z da IMU apontam para o norte (cada um, individualmente), o valor do fluxo 
magnetico (Magnetometers (uT): em RTIMULibDemoGL) de cada eixo medido por RTIMULibDemoGL eh proximo do 
maximo (inverta e deve ficar proximo do minimo - o maximo e o minimo observados nao necessariamente
eh positivo e negativo, respectivamente; podem ser ambos positivos ou negativos).

Se algum dos eixos acima estiver errado no seu sinal (estiver ao contrario), altere as linhas abaixo
do arquivo ~/carmen_lcad/src/pi_imu/RTIMULib2/RTIMULib/IMUDrivers/RTIMULSM9DS1.cpp (ou seu equivalente se 
estiver usando outra IMU; examine o diretorio deste arquivo) para trocar o sinal apropriadamente (elas 
ficam no fim do aquivo):

    //  sort out gyro axes and correct for bias

    m_imuData.gyro.setZ(m_imuData.gyro.z());
    m_imuData.gyro.setY(-m_imuData.gyro.y());

    //  sort out accel data;

    m_imuData.accel.setX(m_imuData.accel.x());
    m_imuData.accel.setY(-m_imuData.accel.y());
    m_imuData.accel.setZ(m_imuData.accel.z());

    //  sort out compass axes

    m_imuData.compass.setX(-m_imuData.compass.x());
    m_imuData.compass.setY(-m_imuData.compass.y());
	//    m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter


Em veiculos como o Art, melhor calibrar usando a interface de texto e apenas maximo e minimo do magnetometro.
Ou seja, coloque para calibrar, retire a sensorbox de sua fixacao e gire-a de ponta cabeca e para a posicao
normal para ajustar o maximo de z. Depois, fize-a e dirija o Art em circulos para obter o maximo e minimo de
x e y.

Quando a calibracao estiver OK, voce pode verifica-la junto ao resto do sistema carmen_lcad rodando:

No Raspberry Pi:

```bash
 $ cd ~/carmen_lcad/src/pi_imu/pi_imu_server
 $ ./Output/pi_imu_server_driver 
```

No PC (um comando abaixo por terminal e dentro do ~/carmen_lcad/bin):

```bash
 $ ./central
 $ ./param_deamon ../src/carmen-ford-escape.ini
 $ ./viewer_3D
 $ ./pi_imu_client_driver <IP do Raspbery pi>
```
(Este teste requer o programa pi_imu_client_driver. Sua compilacao eh descrita mais abaixo.)

No viewer_3D, deligue o GPS Axis e ligue o XSENS Axis. 

Ao apontar o eixo x da IMU para o norte como seu eixo z apontando para cima, o eixo x (vermelho) da IMU 
no viewer_3D deve apontar para a frente do carro, o eixo y (verde) para a esquerda e o eixo z (azul) para cima. 
Use o zoom e mova o carro no viewer_3D se precisar.


# Compile the pi_imu client drive module on your computer

```bash
 $ cd $CARMEN_HOME/src/pi_imu
 $ make
 $ ./pi_imu_client_driver {<ip of machine running pi_imu_server_driver> default = 192.168.1.15}
```

# Visualize the pi_imu module on your computer

```bash
 $ cd $CARMEN_HOME/src/pi_imu_viewer 
 $ make
 $ $CARMEN_HOME/bin/imu_viewer <pi_imu | no_pi_imu>
```

The pi_imu_viewer program will display the image of a box representing the state of the IMU.

Este programa nao usa o mesmo sistema de coordenadas do carmen_lcad, mas pode ser util pois
imprime os angulos da pose da IMU e os dados de seus sensores. 

Ele pode mostrar os dados de uma IMU pi ou da XSENS individualmente e ao mesmo tempo (se voce rodar duas 
instancias dele em terminais diferentes pode ver as duas IMUs ao mesmo tempo). Para isso,
use para a PI IMU:

```bash
 $ $CARMEN_HOME/bin/imu_viewer pi_imu
```

E para a XSENS:

```bash
 $ $CARMEN_HOME/bin/imu_viewer xsens
```

Note que, neste caso, as duas IMUs estarao publicando mensagens de xsens (o sistema espera que
mensagens de apenas uma sejam publicadas).


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

 The eth0 inet addr must be 192.168.1.15

# Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

1- Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo 
  para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
 cd
 ssh-keygen -t rsa

2- Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo
 cd
 ssh pi@192.168.1.15 mkdir -p .ssh
 cat ~/.ssh/id_rsa.pub | ssh pi@192.168.1.15 'cat >> .ssh/authorized_keys'

3- Teste se funcionou com o comando abaixo
 ssh pi@192.168.1.15 'ls'

