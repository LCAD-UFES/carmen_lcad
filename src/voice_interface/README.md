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
 pip3 install google-cloud-texttospeech==2.4.0
 pip3 install google-cloud-speech==2.4.0
 sudo apt-get install portaudio19-dev
 pip3 install pyaudio
```

## 5. Some Python Samples

https://github.com/GoogleCloudPlatform/python-docs-samples

# 6. Setup Porcupine wake-word engine by PicoVoice

### 6.1 Creating the hotword/wake-word
Follow the steps below to create a hotword/wake-word file with .ppn extension:

- Create a Picovoice Console account at [Picovoice/Console](https://console.picovoice.ai/);
- Login and select the Porcupine Wake Word Engine at the dashboard;
- Enter the Phrase="ok e ara", select the Language="English" and click on "Train Wake Word..." button;
- Download the .ppn file containing Custom Wake Word model (it might take several minutes to finish); and
- Move the download .ppn file to $CARMEN_HOME/data/voice_interface_data/hotword_ok_iara.ppn.

Note that the generated file has a 30-day temporary license. Please create another .ppn file after license expiration.

### 6.2 Cloning the Porcupine repository

Clone the [Github/Picovoice/Porcupine](https://github.com/Picovoice/Porcupine) repository as follows:

```sh
 cd ~/packages_carmen
 git clone --branch v1.9 https://github.com/Picovoice/Porcupine.git
```

In the sequel, execute the commands below and test `Porcupine` by saying the hotword/wake-word: *"Ok, Iara!"*
 
```sh
 cd Porcupine
 gcc -std=c99 -O3 -o demo/c/porcupine_demo_mic -I include/ demo/c/porcupine_demo_mic.c -ldl -lasound
 ./demo/c/porcupine_demo_mic lib/linux/x86_64/libpv_porcupine.so lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/hotword_ok_iara.ppn 0.5 default
```
The following message is expected: *"Hotword detected!"*

### 6.3 Using Porcupine at CARMEN

Execute the commands below to enable Carmen to use the `Porcupine` wake-word engine:

```sh
 cp ~/carmen_packages/Porcupine/lib/common/porcupine_params.pv $CARMEN_HOME/data/voice_interface_data/
 cp ~/carmen_packages/Porcupine/include/pv_porcupine.h $CARMEN_HOME/src/voice_interface/
 cp ~/carmen_packages/Porcupine/include/picovoice.h $CARMEN_HOME/src/voice_interface/
 ln -s ~/carmen_packages/Porcupine/lib/linux/x86_64/libpv_porcupine.so $CARMEN_HOME/lib/libpv_porcupine.so
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

- We use the iara_nlu.md model.

- Execute:

```sh
 python3 -m rasa_nlu.train -c nlu_config.yml --data iara_nlu.md -o models --fixed_model_name nlu --project current --verbose
```
- The command above creates a model. Test the model with web server:

```sh
python3 -m rasa_nlu.server --path models --response_log logs
```

### 8. Tips

- Running Voice Interface:

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
	- Add a timeout to the listen function. (SOLVED)
	- Make sure the packages required by python-pyaudio don't break stuff like the nvidia driver (some 'mesa' related packages are installed).
	- Change the speak function to send the audio to the speakers without having to save the file in the disk. (SOLVED)
	- Make the speak function synchronous: the function should only finish after the speakers played the sentence.

### See more about Google Cloud APIS

> Google Cloud APIs are a key part of Google Cloud Platform, allowing you to
> easily add the power of everything from storage access to machine-learning-based 
> image analysis to your Cloud Platform applications.
> https://cloud.google.com/apis/docs/overview
