read -p "Please connect the Car's can can connector and press enter to continue"
sudo ip link set can0 up type can bitrate 500000
read -p "Please connect the Lume Boxes' connector and press enter to continue"
sudo ip link set can1 up type can bitrate 250000
