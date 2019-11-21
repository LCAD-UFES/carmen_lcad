Configuring Reach to receive correction and send position via serial

Correction input

Serial

Device: UART

Baud rate: 9600

Format: RTCM3

Position output

Serial

Device: USB-to-PC

Baud rate: 115200

Format: NMEA

---------------------------------------------------
Installing and running str2str to receive correction from NTRIP server and send it to Reach via S1 serial port

Downloading and installing RTKLIB:

https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3

git clone https://github.com/tomojitakasu/RTKLIB.git

cd RTKLIB/app

source ./makeall.sh

Running str2str:

https://manpages.debian.org/unstable/rtklib/str2str.1.en.html

https://github.com/tomojitakasu/RTKLIB/issues/99

cd RTKLIB/app/str2str/gcc
<<<<<<< HEAD:src/gps/readme_reach_m+.txt
=======

>>>>>>> 39f8dae54527c45c6736c03b49e01b3d6c5a431c:src/gps/readme_reach_m+.md
sudo ./str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyS0:9600:8:n:1:off

---------------------------------------------------
Configuring the GPIO serial port on Raspberry 3 
<<<<<<< HEAD:src/gps/readme_reach_m+.txt
=======

https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/
>>>>>>> 39f8dae54527c45c6736c03b49e01b3d6c5a431c:src/gps/readme_reach_m+.md

To enable the GPIO serial port, edit config.txt:
https://spellfoundry.com/2016/05/29/configuring-gpio-serial-port-raspbian-jessie-including-pi-3-4/

$ sudo gedit /boot/config.txt &

and add the line (at the bottom):

enable_uart=1

<<<<<<< HEAD:src/gps/readme_reach_m+.txt
To disable Linux's use of console UART:
=======
To change the speed of the serial console, edit /boot/cmdline.txt:

https://www.raspberrypi.org/forums/viewtopic.php?t=217036

>>>>>>> 39f8dae54527c45c6736c03b49e01b3d6c5a431c:src/gps/readme_reach_m+.md
https://www.raspberrypi.org/documentation/configuration/uart.md

sudo raspi-config

Select option 5, Interfacing options, then option P6, Serial, and select No. Exit raspi-config.
	

