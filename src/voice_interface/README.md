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
 ./tools/optimizer/linux/x86_64/pv_porcupine_optimizer -r resources/optimizer_data/ -p linux -o . -w "ok e ara"
 export SYSTEM=linux
 export MACHINE=x86_64
 cd demo/alsa
 g++ -O3 -o alsademo -I../../include -L../../lib/${SYSTEM}/$MACHINE -Wl,-rpath ../../lib/${SYSTEM}/$MACHINE main.cpp -lpv_porcupine -lasound
 cp ../../ok\ e\ ara_linux.ppn ../../resources/keyword_files/pineapple_linux.ppn
 ./alsademo
```

 - Teste dizendo a hotword/wake-word: "Ok, Iara!"

"Hotword detected!"

### 6.1 To use on CARMEN

```sh
 cp ~/carmen_packages/Porcupine/ok\ e\ ara_linux.ppn $CARMEN_HOME/data/voice_interface_data/hotword_oi_iara.ppn
 cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
 cp ~/carmen_packages/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
 cp ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.a libpv_porcupine.a
 ln -s ~/carmen_packages/Porcupine/libpv_porcupine.a $CARMEN_HOME/lib/libpv_porcupine.a
```

- Note:

"update_porcupine_files.sh" does all the commands above, if the license of Porcupine expired.

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

- We use the iara_nlu.md model.

- Execute:

```sh
 python3 -m rasa_nlu.train -c nlu_config.yml --data iara_nlu.md -o models --fixed_model_name nlu --project current --verbose
```
- The command above creates a model. Test the model with web server:

```sh
python3 -m rasa_nlu.server --path models --response_log logs
```

### 8. Running Voice Interface

 - Start central;
 - Initiate server with:
```sh
python3 -m rasa_nlu.server --path models --response_log logs
```
 - Run ./voice_interface


You'll need a microphone and a speaker.
### 9. Problems to be solved (TTS/STT)

	- Billing account? The same json can be used for everybody? 
		- A: To use the Google API you need a billing account. Using the same Google account you can create multiple service keys and designate roles to which one.
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
