# RASPBERRY SENSORBOX

 - To rum the xsens from the Raspberry PI:

# Istall Raspbian on the Raspberry PI

- Baixe a imagem do sistema RASPBIAN STRETCH WITH DESKTOP no site do raspberry (https://www.raspberrypi.org/downloads/raspbian/)
Obs: Nao baixe o arquivo LITE pois este possui apenas interface por linha de comando.

- Será necessário identificar o nome do dispositivo do cartão SD. Para isto, antes de inserir o cartão de memória execute o seguinte comando para ver as unidades de disco presentes.

```bash
 $ sudo fdisk -l
 ou
 $ ls -la /dev/sd*
```
- Insira o cartão e execute novamente o comando de forma a identificar o nome do cartao de SD que irá aparecer. Será algo do tipo /dev/sd...

```bash
 $ sudo fdisk -l
```

- Caso o cartão SD apareça como por exemplo /dev/sde1 ou /dev/sde2, eliminar o numero e considerar apenas /dev/sde 

- Execute o seguinte comando para copiar a imagem do Raspibian para o cartão SD fazendo as alterações de caminho necessárias.

```bash
 $ sudo dd if=/home/usr/Downloads/2018-04-18-raspbian-stretch.img of=/dev/sd...
```

- O cartão esta formatado e pode ser inserido no Raspberry para utilização.


# Enable the Camera and SSH

- Não execute upgrade

```bash
 $ sudo apt-get update
 $ sudo raspi-config
```
 Acesse e habilite a camera:
 
 - Interfacing Options->Camera
 - Interfacing Options->SSH

Teste usando o comando: 

```bash
 $ raspistill -v -o test.jpg
 $ raspivid -o teste.h264 -t 10000
```
raspistill grava uma imgem e raspivid grava um video

# Install Dependencies anf Download the pi_camera file from git

```bash
 $ sudo apt-get install subversion
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/
```

