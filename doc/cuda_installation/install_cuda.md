IMPORTANTE: O instalador da zed requer CUDA 9.1, enquanto o tensorflow requer CUDA 9.0. O tutorial assume que será instalado CUDA 9.0.

### Instalar o driver de placa de vídeo do fabricante
Em System Settings, acesse Software & Updates.

<img src="https://github.com/LCAD-UFES/carmen_lcad/blob/master/doc/cuda_installation/settings_screenshot.png" width="800" title="Imagem de System Settings">

Acesse a aba Additional drivers, selecione a primeira opção de driver NVIDIA, clique em Apply Changes. Aguarde o fim do processo, quando aparecerá a opção de reiniciar o computador, e o reinicie.

![Imagem de Software & Updates](https://github.com/LCAD-UFES/carmen_lcad/blob/master/doc/cuda_installation/additional_drivers_screenshot.png)


### Instalar o CUDA 9.0

Acesse o link abaixo:\
[CUDA 9.0 download runfile](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=runfilelocal)

Faça download do Base Installer e, com o terminal aberto na pasta de Downloads, digite o comando:
```
sudo sh cuda_9.0.176_384.81_linux.run
```
Aperte a espaço para descer pelos termos e compromissos e responda as perguntas seguintes da maneira abaixo:
```
Do you accept the previously read EULA?
accept/decline/quit: accept

Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 384.81?
(y)es/(n)o/(q)uit: n

Install the CUDA 9.0 Toolkit?
(y)es/(n)o/(q)uit: y

Enter Toolkit Location
 [ default is /usr/local/cuda-9.0 ]: 

Do you want to install a symbolic link at /usr/local/cuda?
(y)es/(n)o/(q)uit: y

Install the CUDA 9.0 Samples?
(y)es/(n)o/(q)uit: n
```

Novamente na página de download do CUDA 9.0, baixe e instale os Patches existentes um por um, de forma semelhante à instalação base:
- comando `sudo sh <nome do arquivo>`
- Desça pelos termos pressionando espaço
- escreva accept ao fim

Como forma de testar se a instalação foi bem sucedida, compile e rode dois exemplos disponibilizados:
```
cd /usr/local/cuda-9.0/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```
```
cd /usr/local/cuda-9.0/samples/5_Simulations/particles
sudo make
./particles
```

### Instalar o CuDNN
Acesse [NVIDIA CuDNN](https://developer.nvidia.com/cudnn), e clique em **Download CuDNN**. Na página seguinte, crie uma conta ou acesse a sua, caso já tenha criado.

Na página CuDNN Download, marque a caixa  **I Agree To the Terms of the cuDNN Software License Agreement** e no fim da página clique em **Archived cuDNN releases**. Clique em **Download cuDNN v7.4.1 (Nov 8, 2018), for CUDA 9.0** e baixe os seguintes arquivos:
- cuDNN Runtime Library for Ubuntu16.04 (Deb)
- cuDNN Developer Library for Ubuntu16.04 (Deb)
- cuDNN Code Samples and User Guide for Ubuntu16.04 (Deb)

Abra o terminal na pasta dos downloads e instale os três pacotes:
```
cd ~/Downloads
sudo dpkg -i libcudnn7_7.4.2.24-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.4.2.24-1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-doc_7.4.2.24-1+cuda9.0_amd64.deb
```
Para testar:
```
cp -r /usr/src/cudnn_samples_v7/ $HOME
cd  $HOME/cudnn_samples_v7/mnistCUDNN
make clean && make
./mnistCUDNN
```
Instalar outras dependências:
```
sudo apt-get install libcupti-dev
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
sudo apt-get install openjdk-8-jdk
wget https://github.com/bazelbuild/bazel/releases/download/0.11.1/bazel_0.11.1-linux-x86_64.deb
sudo dpkg -i bazel_0.11.1-linux-x86_64.deb
```

### Correção do bug do display externo
Esse problema ocorreu até o momento somente com notebooks que estavam utilizando o Ubuntu 20.04 LTS, geralmente ele ocorre quando se seleciona o monitor externo como o único display. Para resolver esse problema foi ultlizados os seguintes passos:

<br/>

#### Pesquise o app NVIDIA X Server Settings: 

<img src="solving_the_display_bug_0.png" width="800" title="App NVIDIA X Server Settings">

<br/>

#### Selecione a opção de menu PRIME Profiles e marque a opção NVIDIA(Performance Mode):

<img src="solving_the_display_bug_1.png" width="800" title="Melhor opção">

<br/>

#### Feche o app e reinicie o computador para validar a configuração realizada,: 

<img src="solving_the_display_bug_2.png" width="800" title="Finalizando">

<br/>

***OBS: Caso a opção NVIDIA(Performance Mode) já esteja marcada:***

- Altere para a outra opção disponível, no caso da imagem usada como referência use a opção NVIDIA on-Demand;
- saia do app e reinicie o computador; 
- Após o reboot do computador refaça os passos mudando para a opção NVIDIA(Performance Mode) e reinicie o computador novamente.