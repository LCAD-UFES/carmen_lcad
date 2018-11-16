Para estabelecer uma hotword, usei o Porcupine, da Picovoice: https://github.com/Picovoice/Porcupine
Criei uma hotword com a sequencia de comandos:
 cd ~
 git clone https://github.com/Picovoice/Porcupine.git
 cd Porcupine
 tools/optimizer/linux/x86_64/pv_porcupine_optimizer -r resources/ -p linux -o . -w "ok e ara"
 export SYSTEM=linux
 export MACHINE=x86_64
 cd demo/alsa
 g++ -O3 -o alsademo -I../../include -L../../lib/${SYSTEM}/$MACHINE -Wl,-rpath ../../lib/${SYSTEM}/$MACHINE main.cpp -lpv_porcupine -lasound
 cp ../../ok\ e\ ara_linux.ppn ../../resources/keyword_files/pineapple_linux.ppn
 ./alsademo

Depois disso eh soh falar "Ok Iara" que detecta.
 
 
