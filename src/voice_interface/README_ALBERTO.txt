= Hotword com Porcupine (https://github.com/Picovoice/Porcupine) =
- Para estabelecer uma hotword, usei o Porcupine, da Picovoice: https://github.com/Picovoice/Porcupine
- Criei uma hotword com a sequencia de comandos:
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

- Depois disso eh soh falar "Ok Iara" que detecta.

- O codigo agora usa esta hotword. Ela foi instalada com os seguintes comandos
 cp ~/carmen_packages/Porcupine/ok\ e\ ara_linux.ppn ../../data/voice_interface_hotword_data/hotword_oi_iara.ppn
 cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv ../../data/voice_interface_hotword_data/
 cp ~/carmen_packages/Porcupine/include/picovoice.h .
 cp ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a.copy
 ln -s libpv_porcupine.a.copy ../../lib/libpv_porcupine.a
 git add picovoice.h libpv_porcupine.a.copy ../../data/voice_interface_hotword_data/porcupine_params.pv ../../data/voice_interface_hotword_data/hotword_oi_iara.ppn

- TODO: o Porcupine diz que eh livre mais quando roda diz que expira em 90 dias...


== NLTK (https://www.nltk.org/) ==
- Instalacao
 sudo pip install -U nltk
 sudo apt-get install python3-tk
 sudo apt-get install python-tk

- Dowload de datasets (https://medium.com/@viniljf/utilizando-processamento-de-linguagem-natural-para-criar-um-sumariza%C3%A7%C3%A3o-autom%C3%A1tica-de-textos-775cb428c84e)
 python
 >>> import nltk
 >>> nltk.download()
- Na interface grafica escolha e facca dowload (vai para o diretorio ~/nltk_data):
  - Corpora:
	- floresta
	- mac_morpho
	- machado
	- stopwords
	- wordnet
	- words
  - Models:
	- averaged_perceptron_tagger
	- punkt

