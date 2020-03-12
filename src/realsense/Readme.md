# Realsense driver

O driver da camera Intel Realsense para linux eh baseado na [librealsense](https://github.com/IntelRealSense/librealsense). Para utilizar a camera, eh necessario utilizar uma maquina que suporte **USB 3.0**.

## Teste fora do Carmen

 Para realizar um teste rapido:

1. Conecte a camera em uma porta **USB 3.0**

2. Libere o acesso a camera de dentro do docker

```
xhost local:root
```

2. Com o docker instalado, execute o seguinte comando, que ira baixar e rodar um container ja configurado para utilizar a realsense:

```
docker run -it --volume=/tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri:/dev/dri --privileged -v /dev/video/ --env="DISPLAY" speechtesting11/ubuntu16.04_lrs3
```

3. Certifique-se de que a camera foi encontrada rodando o comando:

```
rs-enumerate-devices
```

A saida esperada sera algo do tipo:

```
 Device info: 
    Name                          : 	Intel RealSense D435
    Serial Number                 : 	825412070422
    Firmware Version              : 	05.09.02.00
    Physical Port                 : 	/sys/devices/pci0000:00/0000:00:14.0/usb2/2-2/2-2:1.0/video4linux/video0
    Debug Op Code                 : 	15
    Advanced Mode                 : 	YES
    Product Id                    : 	0B07
    Recommended Firmware Version  : 	05.09.09.02
```

4. Execute o programa para visualizar:

```
realsense-viewer
```

## Instalacao 

Conforme presente no [manual oficial de instalcao](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md), temos como requisitos para utilizar a camera:

* Ubuntu 16 ou 18 com kernel versao 4.4, 4.10, 4.13 ou 4.15. Para versoes do kernel 4.16+ eh necessario [compilar manualmente a librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

Para instalar a librealsense, siga os passos:

- Register the server's public key:  
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
```  

In case the public key still cannot be retrieved, check and specify proxy settings: `export http_proxy="http://<proxy>:<port>"`, and rerun the command. See additional methods in the following [link](https://unix.stackexchange.com/questions/361213/unable-to-add-gpg-key-with-apt-key-behind-a-proxy).  

- Add the server to the list of repositories:  
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```  

- Install the libraries:  
```
  sudo apt-get install librealsense2-dkms
  sudo apt-get install librealsense2-utils
```    
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

- Install the developer and debug packages:  
```
  sudo apt-get install librealsense2-dev
  sudo apt-get install librealsense2-dbg
```  

-  With `dev` package installed, you can compile an application with **librealsense** using `g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

- Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

- Verify that the kernel is updated :    
```
modinfo uvcvideo | grep "version:"
``` 
should include `realsense` string

