# README_COMO_CONTROLAR_IARA_VIA_LAPTOP


## Network setup
Create wired connection named `IARA`.  
Go to Edit Connections and change IP to Manual with the following information (yes, two IPs on the same network):  

IP Adress: 192.168.0.1  
Mask: 24  
Gateway:192.168.0.1  
IP Adress: 192.168.1.1  
Mask: 24  

Make sure device name to this network is eth0 or enp3s0.



## NAT setup
```
    cd src/scripts_lume
    ./configure_red_car
```

The wlpX and enpX elements are the names of the network interfaces present on your PC, to find out what these are names on your pc follow the script below:

```
    sudo apt-get update
    sudo apt-get install net-Tools

    ifconfig
```

Copy the names and add them to the script execution command, eg `sudo bash config_rede_car.bash enp1s0 wlp2s0`  
To add a new device to the already existing network run the script using `-n`, eg `sudo bash config_rede_car.bash -n enp1s0 wlp2s0`.
To reset all settings already entered, run the script using `-r`, eg `sudo bash config_rede_car.bash -r`.



## Raspberry access setup
Inside `IARA` network, check the conection: `ping 192.168.1.15`.

If you don't already have a public key on the computer that will access the Pi, run the commands below to generate it in `~/.ssh/id_rsa.pub` (make sure you already have the file so you don't generate it again)

```
    cd
    ssh-keygen -t rsa
```

Copy the public key from the computer that will access the Pi to the Pi with the commands below

```
    cd
    ssh pi@192.168.1.15 mkdir -p .ssh
    cat .ssh/id_rsa.pub | ssh pi@192.168.1.15 'cat >> .ssh/authorized_keys'
```

Check
```
    ssh pi@192.168.1.15 'ls'
```



## Issues
### OpenJAUS
Go to `$CARMEN_HOME/sharedlib/OpenJAUS/README_ALBERTO.txt`. Problably you need to install `sudo apt-get install subversion` and `sudo apt-get install libncurses5-dev` and make sure that the `JUDP_IP_Address` in the file `sharedlib/OpenJaus/ojNodeManager/nodeManager.conf` is the same as that of the machine where you are going to run this code.
