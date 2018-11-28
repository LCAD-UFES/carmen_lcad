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
 cp ~/carmen_packages/Porcupine/ok\ e\ ara_linux.ppn $CARMEN_HOME/data/voice_interface_data/hotword_oi_iara.ppn
 cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
 cp ~/carmen_packages/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
 cp ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a.copy
 cd ~/carmen_packages/Porcupine/
 ln -s libpv_porcupine.a.copy $CARMEN_HOME/lib/libpv_porcupine.a
 git add picovoice.h libpv_porcupine.a.copy $CARMEN_HOME/data/voice_interface_hotword_data/porcupine_params.pv $CARMEN_HOME/data/voice_interface_hotword_data/hotword_oi_iara.ppn

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

== RASA (https://rasa.com) ==
- Instalacao
 sudo apt-get install mpg123
 sudo pip3 install -U spacy
 sudo pip3 install rasa_nlu
 sudo pip3 install rasa_nlu[spacy]
 sudo python3 -m spacy download en_core_web_md
 sudo python3 -m spacy link en_core_web_md en
 pip3 install rasa_nlu[tensorflow]
 sudo apt-get install libjsoncpp-dev

- Teste (ver https://www.rasa.com/docs/nlu/0.13.1/quickstart/)
  - Salve o conteudo abaixo no arquivo nlu_config.yml (sem os +++...)
++++++++++++++++++++++++++++++++
language: "pt"
pipeline: "tensorflow_embedding"
++++++++++++++++++++++++++++++++
  - Salve o conteudo abaixo no arquivo nlu.md
++++++++++++++++++++++++++++++++
## intent:greet
- Ei
- Olá
- Oi
- bom Dia
- boa noite
- Olá

## intent:restaurant_search
- Estou procurando um lugar para comer
- Eu quero pegar o almoço
- Estou procurando um local para jantar
- estou procurando um lugar no [norte](local) da cidade
- mostre-me [chinese](culinaria) restaurantes
- mostre-me um lugar [mexicano](culinaria) no [centro](local)
- Estou à procura de um lugar [indiano](culinaria)
- procurar restaurantes
- em qualquer lugar no [oeste](local)
- em qualquer lugar perto de [18328](localizacao)
- Eu estou procurando por comida [asiática fusion](culinaria)
- Estou procurando um restaurante em [29432](localizacao)

## intent:thankyou
- obrigado!
- obrigado
- muito obrigado
++++++++++++++++++++++++++++++++
  - Execute o comando:
 python3 -m rasa_nlu.train -c nlu_config.yml --data nlu.md -o models --fixed_model_name nlu --project current --verbose
  - O comando acima cria o modelo. Teste o modelo via linha de comando com o codigo abaixo. Salve ele em um arquivo rasa_test.py:
++++++++++++++++++++++++++++++++
from rasa_nlu.model import Interpreter
import json
interpreter = Interpreter.load("./models/current/nlu")
message = "Eu gostaria de conhecer um restaurante mexicano no norte"
result = interpreter.parse(message)
print(json.dumps(result, indent=2))
++++++++++++++++++++++++++++++++
  - Teste o modelo via linha de comando com:
 python3 rasa_test.py
  - Teste o modelo via servidor web inciando o servidor:
 python3 -m rasa_nlu.server --path models --response_log logs
  - E, em um outro terminal, mandando um post:
 curl -XPOST localhost:5000/parse -d '{"q":"Eu gostaria de conhecer um restaurante mexicano no norte", "project":"current", "model":"nlu"}'

- Teste em C++
  - Salve o conteudo abaixo no arquivo c_post_example.cpp
++++++++++++++++++++++++++++++++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>

using namespace std;

static size_t 
WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *) userp)->append((char *) contents, size * nmemb);
    return (size * nmemb);
}


int 
main(void)
{
	CURL *curl;
	CURLcode res;
	std::string readBuffer;

	curl = curl_easy_init();
	if (curl) 
	{
		curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:5000/parse?q=restaurante+mexicano&project=current&model=nlu");
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
		res = curl_easy_perform(curl);
		curl_easy_cleanup(curl);

		std::cout << readBuffer << std::endl;

		stringstream ifs(readBuffer);
		Json::Reader reader;
		Json::Value obj;
		reader.parse(ifs, obj);     // Reader can also read strings
		cout << obj << endl;
		cout << "Project: " << obj["project"].asString() << endl;
		cout << "Entities[0]: " << obj["entities"][0] << endl;
		cout << "Entities:entity: " << obj["entities"][0]["entity"].asString() << endl;
		cout << "Intent: " << obj["intent"] << endl;
		cout << "Intent: " << obj["intent"]["name"].asString() << endl;
	}

	return 0;
}
++++++++++++++++++++++++++++++++
  - Compile o codigo com a linha abaixo:
 g++ c_post_example.cpp -lcurl -ljsoncpp
  - Teste: rode o servidor em um terminal
 python3 -m rasa_nlu.server --path models --response_log logs
  - Teste: rode o codigo em C++ em outro
 ./a.out


= Como rodar a voice_interface remota
 export CENTRALHOST=192.168.0.1
 python -m rasa_nlu.server --path models --response_log logs
 ./voice_interface
- Para atualizar rasa DNN
 python -m rasa_nlu.train -c nlu_config.yml --data iara_nlu.md -o models --fixed_model_name nlu --project current --verbose

