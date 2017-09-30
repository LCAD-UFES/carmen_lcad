To run the XGV Carmen Control Unit (XGV_CCU - ojTorc/bin/ojTorc):

0- Include in your .bashrc lines equivalent to those below
 # OpenJAUS
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/alberto/robotics/code/carmen_lcad/sharedlib/OpenJAUS/libopenJaus/lib:/home/alberto/robotics/code/carmen_lcad/sharedlib/OpenJAUS/libjaus/lib:/home/alberto/robotics/code/carmen_lcad/sharedlib/OpenJAUS/ojTorc/lib


1- Compile the code by typing in this directory:
 make clean
 make

2- Make sure that the JUDP_IP_Address in the file ojNodeManager/nodeManager.conf is 
the same as that of the machine where you are going to run this code.

3- Open a Linux terminal, go to the directory ojNodeManager, and type
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

4- Start another Linux terminal and type:
 ojTorc/bin/ojTorc
 
  You should see the XGV Carmen Control Unit (XGV_CCU) interface (ascii 
terminal interface implemented with curses). The first lines present
the keyboard functionalities.

