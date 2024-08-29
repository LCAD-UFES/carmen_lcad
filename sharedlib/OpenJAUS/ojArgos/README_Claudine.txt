Em um terminal, inicialize a rede vcan:
 sudo modprobe vcan
 sudo ip link add dev vcan0 type vcan
 sudo ip link set up vcan0

Com a rede funcionando para o ROS2, isto é, ros2 topic list funcionando em um terminal, rode neste terminal:
 cd ~/argos/src/joystick_teleop/include/joystick_teleop
 python3.8 can_to_wirelesscontroller.py
 
O programa can_to_wirelesscontroller.py recebe mensagens CAN (comandos de joystick) e envia via ROS2 para o Argos.

Em terminais diferentes, rode (um programa em cada terminal diferente):
 ~/carmen_lcad/sharedlib/OpenJAUS/ojNodeManager/bin/ojNodeManager nodeManager_Argos.conf a
 ~/carmen_lcad/sharedlib/OpenJAUS/ojArgos/bin/ojArgos vcan0 vcan0
 ~/carmen_lcad/sharedlib/OpenJAUS/ojTorc/bin/ojTorc
 
No terminal rodando o ojTorc, aperte a tecla w. Depois, o ojTorc estara pronto para enviar comandos de teclado para o Argos.
 Setas para esquerda e para direita para girar
 Setas para cima e para baixo para acelerar para frente e freiar
 Tecla a para ir para frente
 Tecla f para ir para tras
