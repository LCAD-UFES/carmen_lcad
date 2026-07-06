Para baixar as tabelas (matriz NHCM - Non-Holonomic Cost Matrix) do offroad_planner Execute:
make download

Para atualizar as tabelas (matriz NHCM - Non-Holonomic Cost Matrix) do offroad_planner Execute:
make update

As tabelas serão baixadas e colocadas na pasta astro/data. Eles ficam na raiz deste diretório:
https://drive.google.com/drive/u/0/folders/1Veu8G1G6V-NP3VoO45EBm_Nw_JerGdKT

Para saber como gerar as tabelas, examine o README.txt do subdiretorio NHCM.


======== Passos necessários para utilizar o offroad park truck & trailer

Para utilizar o planejador Tractor_trailer_trajectory_planning, é necessário a nossa versão do AMPL.

= Para instalar nossa versão de AMPL:

Baixe o arquivo AMPL.tgz de https://drive.google.com/file/d/1Bx9eP9z_s5k5zkHh6o9RuCpNtClslziO/view?usp=sharing
Descompacte no seu diretório raiz:
 cd ~/
 tar xzvf Downloads/AMPL.tgz

É conveniente incluir o caminho para o executável ampl.bin no .bahsrc. Para isso, inclua no seu .bashrc a linha:
 export PATH=$PATH:$MAEHOME/bin:~/AMPL/ampl.linux-intel64

Para usar o ampl.bin da linha de comando:
 ~/AMPL/ampl.linux-intel64/ampl.bin

ou, se você tiver mudado seu .bashrc e aberto outro terminal
 ampl.bin

Depois, faça:
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libipoptamplinterface.so.1.10.10 /usr/lib/x86_64-linux-gnu/libipoptamplinterface.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libipopt.so.1 /usr/lib/x86_64-linux-gnu/libipopt.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libcoinasl.so.1 /usr/lib/x86_64-linux-gnu/libcoinasl.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libcoinmumps.so.1 /usr/lib/x86_64-linux-gnu/libcoinmumps.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libcoinmetis.so.1 /usr/lib/x86_64-linux-gnu/libcoinmetis.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libcoinlapack.so.1 /usr/lib/x86_64-linux-gnu/libcoinlapack.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libcoinblas.so.1 /usr/lib/x86_64-linux-gnu/libcoinblas.so.1
sudo cp ~/AMPL/ampl.linux-intel64/octeract-runtime/libgfortran.so.3 /usr/lib/x86_64-linux-gnu/libgfortran.so.3

sudo apt-get install libgfortran-7-dev 
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get upgrade libstdc++6
sudo apt install gfortran-7
sudo apt-get install --reinstall libgfortran4

====================================================================

