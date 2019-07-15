To run the XGV Carmen Control Unit (XGV_CCU - ojTorc/bin/ojTorc):

0- Include in your .bashrc lines equivalent to those below
#OpenJaus
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARMEN_HOME/sharedlib/OpenJAUS/libopenJaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/libjaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojTorc/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojIARASim/lib

1- Install necessary packages
 sudo apt-get install subversion
 sudo apt-get install libncurses5-dev

2- Download the LCAD OpenJAUS code - para ser feito somente no RaspBerry
 svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/OpenJAUS

3- Compile the code by typing in this directory:
 make clean
 make

4- Make sure that the JUDP_IP_Address in the file ojNodeManager/nodeManager.conf is 
the same as that of the machine where you are going to run this code.

5- Create wired connection named Carro_Network. Go to Edit Connections and change IP to Manual with the following information:
IP Adress: 192.168.0.1
Mask: 24
Gateway:192.168.0.1
Make sure Device name to this network is eth0 or enp3s0.


6- Create wired connection named Velodyne_Network. Go to Edit Connections and change IP to Manual with the following information:
IP Adress: 192.168.1.71
Mask: 24
Gateway:192.168.1.201
Make sure Device name to this network is eth1 or enp4s0.

7- Open a Linux terminal, go to the directory ojNodeManager, and type
 bin/ojNodeManager
 
 You should see something like this:

" 
[alberto@cachoeiro ojNodeManager]$ bin/ojNodeManager 

OpenJAUS Node Manager Version 3.3.0b (SEPT 2, 2009)

Subsystem ADDED: OJSubsystem-4
Starting Subsystem Interfaces
Success
Error: Cannot bind multicast socket to 0.0.0.0:3794
Opened Subsystem Interface:	JUDP Interface 0.0.0.0:3794
Starting Component Interfaces
Opened Component Interface:	JAUS ETG/OPC UDP Interface 0.0.0.0:24629
Opened Component Interface:	OpenJAUS UDP Component to Node Manager Interface
Component ADDED: OpenJAUS Node Manager (NodeManager)-1.1
Component ADDED: OpenJAUS Communicator (Communicator)-35.1


OpenJAUS Node Manager Help
   t - Print System Tree
   T - Print Detailed System Tree
   m - Show messages
   M - Do not show messages
   c - Clear console window
   ? - This Help Menu
 ESC - Exit Node Manager
"

8- Start another Linux terminal and type:
 ojTorc/bin/ojTorc
 
  You should see the XGV Carmen Control Unit (XGV_CCU) interface (ascii 
terminal interface implemented with curses). The first lines present
the keyboard functionalities.


