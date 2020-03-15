read -p "Please connect the Lume Boxes' can connector and press enter to continue"
sudo ip link set can0 up type can bitrate 500000
read -p "Please connect the Car's can connector and press enter to continue"
sudo ip link set can1 up type can bitrate 250000
