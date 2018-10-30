# Voice Kit AIY project
Voice Kit AIY project is a *do-it-yourself* intelligent speaker. Experiment with voice recognition and the Google Assistant.

For more information, visit [Voice Kit Aiy Project](https://aiyprojects.withgoogle.com/voice/).

***

### 1. Instalação
*Pode seguir os passos em inglês de instalação, configuração e aplicação  [aqui.](https://aiyprojects.withgoogle.com/voice/#users-guide-1-1--connect-to-google-cloud-platform)*

  - Monte o Raspberry e o Voice Hat como mostrado  no [Assembly Guide: Assemble the Hardware](https://aiyprojects.withgoogle.com/voice/#assembly-guide-2-assemble-the-hardware)

  - Use um cartão micro SD para gravar a ISO do Voice Kit. ([Download ISO](magpi.cc/2x7JQfS)). (Gravação da ISO com ETCHER: magpi.cc/2fZkyJD).

  - Conecte os periféricos necessários: Teclado USB, Mouse USB, Cabo HDMI, cabo de energia *(Ex.: carregador celular, corrente>=2,0 A).*

  - Com o boot do Raspberry Pi, o led dentro da caixa se acenderá.
  > Obs.: Caso apareca *Openbox Syntax Error*, você terá que reescrever a ISO no cartão SD.

  - Clique duas vezes no ícone *Check Audio*. Você deverá ouvir *Front, Centre* e uma mensagem na tela. Responda de acordo as instruções. 
  > Em caso de erro, siga a solução mostrada na mensagem.




### 2. APIS do Google Cloud utilizadas no Voice Kit

(*documentation*)

* [Google Assistant SDK](https://developers.google.com/assistant/sdk/overview)
* [Cloud Speech API](https://cloud.google.com/speech/docs/)
* [Text to Speech API](https://cloud.google.com/text-to-speech/docs/)
* [Google Cloud Translation API](https://cloud.google.com/translate/docs/?hl=th)

### 3. Criando credenciais

Para usar as APIS do Google, é necessário o uso de *Credenciais*:

##### 3.1 Criando uma conta no Google Cloud Console

To begin to use a Google Cloud API, you must have a Google account.

  - Select or create a GCP project. [Go to the Manage Page Resources](https://console.cloud.google.com/cloud-resource-manager?_ga=2.159473469.-1617484999.1535991245)

  - Make sure that billing is enabled for your project. [Learn how to enable Billing](https://cloud.google.com/billing/docs/how-to/modify-project)

  - Enable the APIs. In this case, Text-to-Speech and Speech-to-Text APIs. [Search the API and enable it](https://console.cloud.google.com/apis/library?project=voice-iara&folder&organizationId)

  - Set up authentication:
    -  Go to the [Create Service Account Key](https://console.cloud.google.com/apis/credentials/serviceaccountkey?_ga=2.62067500.-1617484999.1535991245) page in the GCP Console. 
( *Menu (≡)  ->   API&Services  ->  Credentials* ) 
    - From the Service account drop-down list, select *New service account*.
    - Enter a name into the Service account name field. *(Tip: do not use spaces ' ')*
    - Don't select a value from the Role drop-down list. No role is required to access this service. *(Tip: set Owner as role)*
    - Click *Create*. (Perhaps, a note will appear, warning that this service account has no role).
    - Click *Create without role*. A JSON file that contains your key downloads to your computer.


    -  Set the environment variable GOOGLE_APPLICATION_CREDENTIALS to the file path of the JSON file that contains your service account key. This variable only applies to your current shell session, so if you open a new session, set the variable again. 
        > (e.g.)
        ```sh
        export GOOGLE_APPLICATION_CREDENTIALS=/home/user/path/project-name-key.json)
        ```
      - To set the variable in a inderteminate time: set the variable at *~/.bashrc*:
      ```sh
      nano ~/.bashrc 
      ```

  Adicione no final do arquivo:
  ```sh
  GOOGLE_APPLICATION_CREDENTIALS=/home/user/path/project-name-key.json
  ```
  > Ctrl+X to exit

  > Press Y to save it

  > And, at last, Enter

  ```sh
  bash
  ``` 
    
#### 3.2 Install the Client Library

Para usar o cloud-texttospeech:
- Atualize o pip:
--sudo pip3 install --upgrade pip
-Baixe a API:
- pip3 install --upgrade google-cloud-texttospeech

- Para rodar o exemplo:
-- cd /home/pi/python-docs-samples/texttospeech/cloud-client
-- python3 quickstart.py
- Para mudar para portugues, altere esse arquivo, na linha:
-- language_code='en-US'
-para
--language_code='pt-BR'

> For other APIs, look at its documentation.


### 4. Testando demos

*Também em [User Guide: Start the Assistant Library Demo App](https://aiyprojects.withgoogle.com/voice/#users-guide-3-1--start-the-assistant-library-demo-app)*


Para mudar para portugues, altere o arquivo:
- /opt/aiy/projects-python/src/aiy/i18n.py
- Mude a linha 
-- _DEFAULT_LANGUAGE_CODE = 'en-US'
- Para
-- _DEFAULT_LANGUAGE_CODE = 'pt-BR'

Vá para a pasta com as demos:
```sh
 cd /home/pi/AIY-projects-python/src/examples/voice
```
- Teste primeiro o *assistant_library_demo.py*
```sh
python assistant_library_demo.py
```

- Para a demo *assistant_grpc_demo.py*, é necessário o uso do Python 3.5. 
> Problema encontrado/solucionado até ABRIL/2018:
 Renomeie uma pasta *futures* para *oldfutures* dentro da */bin*: 
    ```sh
    cd /usr/local/lib/python3.5/dist-packages/concurrent sudo mv futures oldfutures
    ```
- Re-teste a demo
    ```sh
    python3 assistant_grpc_demo.py
    ```
- Teste as demais demos


***
