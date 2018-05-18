Abaixo estao as instrucoes para a utilizacao do raspberry pi 3 e para compilar e executar o script "rpi_camera".

== Instruções para formatar o SSD card com o Raspbian ==

- Baixe o arquivo imagem do sistema operacional "Raspbian" no site do raspberry (https://www.raspberrypi.org/downloads/raspbian/)
Obs: Nao baixe o arquivo LITE, e sim a versao completa.

-Baixe o Etcher para formatar o cartao de memoria utilizando o arquivo imagem citado na instrucao acima.
Link: (https://etcher.io/)

-Execute o Etcher e selecione o cartao de memoria e o arquivo imagem baixado no site do Raspberry pi

No final do processo o cartao vai estar pronto para ser inserido no Raspberry pi 3, e o mesmo estara pronto para utilizacao.

== Instrucoes iniciais para a utilizaçao do Raspberry pi 3 (Usando a camera)

-Conecte seu Raspberry pi 3 em uma rede Wireless ou Wired(A internet sera um recurso necessario para, eventualmente, baixar-
mos pacotes e programas necessarios para a devida utilizacao do Script "rpi_camera" e de outras ocasionais funcionalidades)

- Habilite a camera seguindo os passos abaixo:

1º-  Clique no menu inicar do Raspiberry pi 3(Canto superior esquerdo da tela inicial) e acesse a secao "Preferencias". Selecione
"Raspiberry Pi Configuration"

2º- Na janela que se abrira selecione a aba "Interfaces"

3º- Habilite a Camera e o SSH.

Sua camera esta habilitada e pronta pra uso.

Obs: Sera necessario reiniciar o sistema apos este passo.

Caso queira testar se a camera ja esta pronta pra uso, teste o seguinte comando no terminal:

"raspivid -o teste.h264 -t 10000"

Se tudo estiver funcionando corretamente a camera sera ativada e gravara um video de 10 segundos que sera salvo na pasta
"home/pi"

Obs: Para consultar os comandos no terminal utilizados pela Pi Camera consulte a documentacao no link: https://www.raspberrypi.org/documentation/raspbian/applications/camera.md

== Instalando os pacotes necessarios para compilar o codigo "rpi_camera" ==

- O primeiro programa a ser baixado sera o "Cmake", necessario para a compilacao dos arquivos do "rpi_camera". Para instalar o Cmake, utilize o comando "sudo apt-get install cmake"
no teminal.

- Apos a intalacao do Cmake, precisaremos dos pacotes do OpenCV. Utilize o comando "sudo apt-get install libopencv-dev python-opencv" para instalar estes pacotes

== Compilando e executando o "rpi_camera" ==

- Baixe o arquivo rpi_camera.zip e siga as linhas de comando abaixo para a compilacao e execucao do arquivo

cd Downloads
unzip rpi_camera.zip
cd rpi_camera
cd raspicam
mkdir build
cd build
cmake ..
make
sudo make install                      Obs:Nao funciona sem o "sudo"
cd ..
cd ..
cd camera
mkdir build
cd build
cmake ..
make
./rpi_camera

Apos o ultimo comando o codigo ja estara compilado e sendo executado.




