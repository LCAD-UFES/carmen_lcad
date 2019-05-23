## Voice_Interface

  * lib[Voice](https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/voice_interface/libvoice)
  * lib[Voice Google](https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/voice_interface/libvoice_google)
  * [Voice-Kit-AIY](https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/voice_interface/voice_kit_aiy)

***

#### Download (only) Voice Interface on GitHub:

[Download a SIngle Folder or Directory from a GitHub Repo](https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)

  ```sh
  sudo apt-get install subversion
  svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/voice_interface/
  ```
#### Python Version

	-- Python == 2.7.12 (not used anymore)
	-- Python3 == 3.5.2 or greater

# 1. Creating an account at Google Cloud Console

To begin to use a Google Cloud API, you must have a Google account.

- Select or create a GCP project names: voice_interface. [Go to the Manage Page Resources](https://console.cloud.google.com/cloud-resource-manager?_ga=2.159473469.-1617484999.1535991245)

- Make sure that billing is enabled for your project. [Learn how to enable Billing](https://cloud.google.com/billing/docs/how-to/modify-project)

- Enable the APIs. First, access the [search page](https://console.cloud.google.com/apis/library?project=voice-iara&folder&organizationId), select the project in the top bar, and search for "Cloud Text-to-Speech API". Click the banner and select activate in the next page. Repeat the process, but this time search for "Cloud Speech API" to activate the  Speech-to-Text API. 

- Set up authentication:
	- Go to the [Create Service Account Key](https://console.cloud.google.com/apis/credentials/serviceaccountkey?_ga=2.62067500.-1617484999.1535991245) page in the GCP Console. (*Menu(≡) ->  API&Services -> Credentials*)
	- From the Service account drop-down list, select *New service account*.
	- Enter a name into the Service account name field: *voice_interface_account* (the Service account name will come with a number series, you'll rename it later).
	- From the Role drop-down list, set *Owner*.
	- Click *Create*.
	- A JSON file that contains your key will start to download with a default name: *project-name-number-series.json*
 	- Rename it to our default name (*voice_interface_credentials.json*) and save it at *~/credentials/*
```sh
cd ~
mkdir credentials
mv ~/Downloads/voice_interface_number_series.json ~/credentials/voice_interface_credentials.json
```
  
# 2. Already have an account? (but changed your computer?)

- Create a new service account key, but at the same service account and project: [Create NEW Service Account Key](https://console.cloud.google.com/apis/credentials/serviceaccountkey?_ga=2.62067500.-1617484999.1535991245)
- Verify if the right project is set at the top of the page.
- Select the service account: voice_interface_account.
- Select 'JSON' as key type.
	- A JSON file that contains your key will start to download with a default name: *project-name-number-series.json*
	- Rename it to our default name (*voice_interface_credentials.json*) and save it at *~/credentials/*
```sh
cd ~
mkdir credentials
mv ~/Downloads/voice_interface_number_series.json ~/credentials/voice_interface_credentials.json
```

# 3. Setting variables
- Set the environment variable GOOGLE_APPLICATION_CREDENTIALS to the file path of the JSON file that contains your service account key.
```
 nano ~/.bashrc
``` 
 Add at the end of bashrc:
```
#Voice Interface
export GOOGLE_APPLICATION_CREDENTIALS=~/voice_interface_credentials.json
export PYTHONPATH=$PYTHONPATH:$CARMEN_HOME/src/voice_interface
```
- At the terminal:
```
bash
```
(*still necessary?*)
- Install and initialize [Google Cloud SDK](https://cloud.google.com/sdk/docs/#deb)
```sh
export CLOUD_SDK_REPO="cloud-sdk-$(lsb_release -c -s)"
echo "deb http://packages.cloud.google.com/apt $CLOUD_SDK_REPO main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install google-cloud-sdk
```
Run the SDK to set configurations:

```sh
 gcloud init
```

# 4. Install the Client Library

Examples of the Text-to-Speech and Speech-to-Text APIs:

Using Python3 (3.5.2 or greater)
```sh
 pip3 install --upgrade google-cloud-texttospeech
 pip3 install --upgrade google-cloud-speech
 sudo apt-get install portaudio19-dev
 pip3 install pyaudio
```

## 5. Some Python Samples

https://github.com/GoogleCloudPlatform/python-docs-samples

# 6. Using Porcupine by PicoVoice
[Github/Picovoice/Porcupine](https://github.com/Picovoice/Porcupine)

```sh
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
```

 - Teste dizendo a hotword/wake-word: "Ok, Iara!"

"Hotword detected!"

### 6.1 Para uso no CARMEN

```sh
 cp ~/carmen_packages/Porcupine/ok\ e\ ara_linux.ppn $CARMEN_HOME/data/voice_interface_data/hotword_oi_iara.ppn
 cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
 cp ~/carmen_packages/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
 cp ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a.copy
 ln -s ~/carmen_packages/Porcupine/libpv_porcupine.a $CARMEN_HOME/lib/libpv_porcupine.a
```

# 7. Using RASA
[RASA Docs](https://www.rasa.com/docs/)


#### Tensorflow

	-- Tensorflow == 1.5


- Installation:
```sh
 sudo apt-get install mpg123
 sudo pip3 install -U spacy
 sudo pip3 install rasa_nlu
 sudo pip3 install rasa_nlu[spacy]
 sudo python3 -m spacy download en_core_web_md
 sudo python3 -m spacy link en_core_web_md en
 sudo pip3 install rasa_nlu[tensorflow]
 sudo apt-get install libjsoncpp-dev
```
- Creating: [RASA/Quickstart]

```sh
cd $CARMEN_HOME/src/voice_interface
```

```sh
gedit nlu_config.yml &
```
- Add the follow content and save it: 
```
language: "pt"
pipeline: "tensorflow_embedding"
```

```sh
gedit nlu.md &
```
- Add the follow content and save it: 

```md
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
```

### 7.1. Testes
#### 7.1.1. Em Python 3: 

- Execute:

```sh
 python3 -m rasa_nlu.train -c nlu_config.yml --data nlu.md -o models --fixed_model_name nlu --project current --verbose
```
- The command above creates a model. Test the template with line command. Save it in a rasa_test.py file:

- At the terminal:

```sh
 gedit rasa_test.py &
```
- Add the content below to the file:

```python
from rasa_nlu.model import Interpreter
import json
interpreter = Interpreter.load("./models/current/nlu")
message = "Eu gostaria de conhecer um restaurante mexicano no norte"
result = interpreter.parse(message)
print(json.dumps(result, indent=2))
```

- Test the model with line command:
```sh
python3 rasa_test.py
```
- Test the model with web server:

```sh
python3 -m rasa_nlu.server --path models --response_log logs
```
- And, in another terminal, send a post:

```sh
curl -XPOST localhost:5000/parse -d '{"q":"Eu gostaria de conhecer um restaurante mexicano no norte", "project":"current", "model":"nlu"}'
```

#### 7.1.2. Em C++

- At the terminal:

```sh
gedit c_post_example.cpp &
```

 - Add the content below to the file and save it:

```c
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
```

- Compile the code:

```sh
 g++ c_post_example.cpp -lcurl -ljsoncpp
```

- Test it: execute the server at the terminal: 
```sh
 python3 -m rasa_nlu.server --path models --response_log logs
```
- Test it: execute the C++ code at other terminal:
```sh
 ./a.out
```


  
### 8. Tips

  
### 9. Problems to be solved (TTS/STT)

	- Billing account? The same json can be used for everybody? 
		- A: Using the same Google account you can create multiple service keys and designate roles to which one.
  	
	- When you run "gcloud init", it asks the user to login. Will it work in a different machine? Can we upload the config file (the .boto saved at ~/.boto)? Is this enough? In the end of the process, it says "You can create additional configurations if you work with multiple accounts and/or projects. Run "gcloud topic configurations" to learn more.". Will we have one account for the whole IARA project, or one for each developer?
		- A:
 
	- Try to make the system work with Python3. (SOLVED)
	- Add a timeout to the listen function.
	- Make sure the packages required by python-pyaudio don't break stuff like the nvidia driver (some 'mesa' related packages are installed).
	- Change the speak function to send the audio to the speakers without having to save the file in the disk. (SOLVED)
	- Make the speak function synchronous: the function should only finish after the speakers played the sentence.

### See more about Google Cloud APIS

> Google Cloud APIs are a key part of Google Cloud Platform, allowing you to
> easily add the power of everything from storage access to machine-learning-based 
> image analysis to your Cloud Platform applications.
> https://cloud.google.com/apis/docs/overview
