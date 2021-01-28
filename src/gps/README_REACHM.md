#Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

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


#Connecting Reach to Raspberry 3

Connect the white wire (RX) of the connector S1 (see reachm_connectors.png) of Reach M+ (or Reach M2) to the TXD pin input of a USB to RS232 TTL adaptor, and the 
black wire (GND) to the GND pin input of USB to TTL adaptor (do not connect the other wires). Connect this USB to TTL adaptor to the Raspberry Pi.

Alternatively, use the UART of Raspberry Pi as described below.

---------------------------------------------------
Configuring the GPIO serial port on Raspberry 3 
(https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/;
https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/)

To enable the GPIO serial port, edit config.txt:

```bash
$ sudo gedit /boot/config.txt &
```

and add the line (at the bottom):

enable_uart=1

To disable Linux's use of console UART
(https://www.raspberrypi.org/documentation/configuration/uart.md)

```bash
sudo raspi-config
```

Select option 5, Interfacing options, then option P6, Serial, and select No. Exit raspi-config.
---------------------------------------------------	

#Configuring Reach to receive correction and send position via serial

To configure Reach M+, follow the instructions in Reach_Configuration.doc (for Reach M2, Reach_M2_Configuration.doc)


#Installing dependencies, downloading src and sharedlib/libcmt directories from git and compile carmen code

Edit .bashrc and insert at its end:

export CARMEN_HOME=/home/pi/carmen_lcad

and run:

```bash
 $ source ~/.bashrc
```

```
 $ sudo apt-get install subversion libncurses5 libncurses5-dev
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/ $CARMEN_HOME/src
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/libcmt/ $CARMEN_HOME/sharedlib/libcmt
```

Replace the $CARMEN_HOME/src/Makefile.rules

```bash
 $ cp $CARMEN_HOME/src/xsens_MTi-G/Makefile.rules $CARMEN_HOME/src/
```

Compile Carmen code:

```bash
 $ cd $CARMEN_HOME/src
 $ ./configure --nojava --nozlib --nocuda --nographics
 Should the C++ tools be installed for CARMEN: [Y/n] Y
 Should Python Bindings be installed: [y/N] N
 Should the old laser server be used instead of the new one: [y/N] N
 Install path [/usr/local/]: 
 Robot numbers [*]: 1,2
```

Compile xsens_MTi-G module on the Raspberry PI (this compiles the gps as well):

```bash
 $ cd ~/carmen_lcad/src/xsens_MTi-G
 $ ./make_pi
```

---------------------------------------------------
#Running Reach M+ Module on Raspberry

Install str2str to receive correction from NTRIP server and send it to Reach via S1 serial port. For that,
download and install RTKLIB
(https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)

```bash
cd $CARMEN_HOME/sharedlib
git clone https://github.com/tomojitakasu/RTKLIB.git
cd $CARMEN_HOME/sharedlib/RTKLIB/app
source makeall.sh
```

Connect Reach M+ (or Reach M2) to the Raspberry Pi with the USB to RS232 TTL adaptor and also using a USB to mini USB cable and
configure low latency serial mode (very important!)

```bash
 $ sudo apt-get update
 $ sudo apt install setserial
 $ sudo usermod -a -G dialout pi
 $ setserial /dev/ttyACM0 low_latency 
```

Then, run:

```bash
sudo $CARMEN_HOME/sharedlib/RTKLIB/app/str2str/gcc/str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyUSB0:115200:8:n:1:off
{or sudo $CARMEN_HOME/sharedlib/RTKLIB/app/str2str/gcc/str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/ESNV0:RTCM3 -out serial://ttyUSB0:115200:8:n:1:off
if CEFE1 is out of order}

sudo $CARMEN_HOME/src/gps/gps_reachm_server /dev/ttyACM0 115200 1 3457
```

For running str2str automatically on system initialization, edit /etc/rc.local and insert at its end:

'''
/home/pi/carmen_lcad/sharedlib/RTKLIB/app/str2str/gcc/str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyUSB0:115200:8:n:1:off > /dev/null 2>&1
'''

---------------------------------------------------
#Running Reach M+ Module on PC

```bash
$CARMEN_HOME/bin/central
$CARMEN_HOME/src/gps/gps_reachm_client 192.168.1.15 3457 
```

--------------------------------------------------
#Running str2str permanently in the Raspberry

Edit /etc/rc.local and add the following line before "exit 0"

/home/pi/carmen_lcad/sharedlib/RTKLIB/app/str2str/gcc/str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyUSB0:115200:8:n:1:off > /dev/null 2>&1 &

--------------------------------------------------
