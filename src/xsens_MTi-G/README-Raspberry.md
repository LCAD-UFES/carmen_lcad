# RASPBERRY SENSORBOX

 - To run the xsens from the Raspberry PI:

# Install Raspbian on the Raspberry PI

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

# Download and Install

```bash
 $ sudo apt-get install subversion libncurses5 libncurses5-dev
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/ ~/carmen_lcad/src
 $ svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/libcmt/ ~/carmen_lcad/sharedlib/libcmt
```

- Baixe e compile uma versão mais atual do IPC

```bash
 $ cd /usr/local
 $ sudo wget http://www.cs.cmu.edu/afs/cs/project/TCA/ftp/ipc-3.9.1a.tar.gz
 $ sudo tar -xzvf ipc-3.9.1a.tar.gz
 $ cd ipc-3.9.1/src/
 $ sudo cp ~/carmen_lcad/src/xsens_MTi-G/formatters.h .
 $ make
```

- Substitua o arquivo Makefile.rules do src do carmen

```bash
 $ cp ~/carmen_lcad/src/xsens_MTi-G/Makefile.rules ~/carmen_lcad/src/
```

# Configure CARMEN LCAD

```bash
 $ cd ~/carmen_lcad/src
 $ ./configure --nojava --nozlib --nocuda
 Should the C++ tools be installed for CARMEN: [Y/n] Y
 Should Python Bindings be installed: [y/N] N
 Should the old laser server be used instead of the new one: [y/N] N
 Install path [/usr/local/]: 
 Robot numbers [*]: 1,2
```

# Compile xsens_MTi-G module on the Raspberry PI

```bash
 $ cd ~/carmen_lcad/src/xsens_MTi-G
 $ ./make_pi
```

- O make_pi assume que a variavel CENTRALHOST possui o valor 192.168.1.1. 
Caso a rede que ira rodar o central tenha outro IP, altere o arquivo ~/.bashrc 

# Make /dev/ttyUSB0 OK for read/write by xsens_MTi-G

```bash
 $ sudo apt-get update
 $ sudo apt install setserial
```

Com o xsens conectado, execute:
```bash
 $ sudo usermod -a -G dialout pi
 $ setserial /dev/ttyUSB0 low_latency 
```

Para tornar o comando setserial permanente, adicione ele ao /etc/rc.local .
Para isso, inclua a linha "setserial /dev/ttyUSB0 low_latency" no fim do arquivo /etc/rc.local uma linha antes do "exit 0".
Após um sudo reboot, tudo estará pronto para uso.

# Para executar o xsens_mtig sem senha de um terminal ou a partir de um process

Se o root da car01 ainda não tem uma chave pública para acessar os Pi, execute os comando abaixo para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
```bash
 $ cd
 $ ssh-keygen -t rsa
```

Copie a chave pública da car01 para os Pi com os comando abaixo
```bash
 $ cd
 $ ssh pi@192.168.1.15 mkdir -p .ssh
 $ cat .ssh/id_rsa.pub | ssh pi@192.168.1.15 'cat >> .ssh/authorized_keys'
```

Teste se funcionou com o comando abaixo
```bash
 $ ssh pi@192.168.1.15 'ls'
```

# Install pi_imu (optional)

 [Read the instructions](../pi_imu)

# Install pi_camera (optional)

 [Read the instructions](../pi_camera)
