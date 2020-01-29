#Connecting Reach to Raspberry 3

Connect the white wire output (RXD) of Reach to the TXD pin input of USB to TTL adaptor, and the black wire output (GRD) of Reach to the GRD pin input of USB to TTL adaptor.

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
	

#Configuring Reach to receive correction and send position via serial

To configure Reach, connect the antenna to it, access http://reach.local using a browser, and set the Correction Input and Position Output as defined below.

```
Correction input
Serial
Device: UART
Baud rate: 115200
Format: RTCM3t

Position output
Serial
Device: USB-to-PC
Baud rate: 115200
Format: NMEA
```
---------------------------------------------------
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

Compile xsens_MTi-G module on the Raspberry PI:

```bash
 $ cd ~/carmen_lcad/src/xsens_MTi-G
 $ ./make_pi


---------------------------------------------------
Installing str2str to receive correction from NTRIP server and send it to Reach via S1 serial port

Downloading and installing RTKLIB
(https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)

```bash
cd $CARMEN_HOME/sharedlib
git clone https://github.com/tomojitakasu/RTKLIB.git
cd $CARMEN_HOME/sharedlib/RTKLIB/app
source makeall.sh
```
---------------------------------------------------
#Running Reach M+ Module on Raspberry

```bash
sudo $CARMEN_HOME/sharedlib/RTKLIB/app/str2str/gcc/str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyUSB0:115200:8:n:1:off

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
