Instalação e Execução do Aplicativo str2str para Receber Correção e Enviá-la para o Reach via Porta Serial 

Endereço do GitHub:
https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3

Instalação do Aplicativo:
git clone https://github.com/tomojitakasu/RTKLIB.git
cd RTKLIB/app
source ./makeall.sh

Execução do Aplicativo
cd RTKLIB/app/str2str/gcc
sudo ./str2str -in ntrip://adesouza:76EfSL@170.84.40.52:2101/CEFE1:RTCM3 -out serial://ttyUSB0:9600
