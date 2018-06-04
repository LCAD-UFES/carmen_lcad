# == Para colocar para operar o Voice Kit AIY - Raspberry Pi ==

-Instale uma versao do Raspibian >= 2017-09-07 
-Conectar os cabos ligando Raspberry e Voice Hat como mostrado em: https://aiyprojects.withgoogle.com/voice/#assembly-guide-2-assemble-the-hardware
-Use um cartao micro SD para gravar a ISO do Voice Kit. (Download em: magpi.cc/2x7JQfS). (Gravacao da ISO com ETCHER: magpi.cc/2fZkyJD).
-Conecte os perifericos necessarios: Teclado USB, Mouse USB, Cabo HDMI, cabo de energia (Ex.:celular, corrente>=2,0 A).
-Com o boot do Raspberry Pi, o led dentro da caixa se acende. Caso apareca "Openbox Syntax Error", voce tera que reescrever a imagem no cartao SD.
-Clique duas vezes no icone "Check Audio". Voce ouvira "Front, Centre" e uma mensagem na tela. Responda de acordo as instrucoes. Em caso de erro, siga a solucao mostrada na mensagem.

### == ATIVANDO A REDE ==

####Quando for colocar o Raspberry na rede da IARA

- Para ativar a rede wired: https://www.modmypi.com/blog/how-to-give-your-raspberry-pi-a-static-ip-address-update
- Para incluir uma rede WiFi (note que o Raspberry 3 tem hardware de wifi e Bluetooth nativos), edite /etc/wpa_supplicant/wpa_supplicant.conf e inclua sua rede sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
- Em seguida, baixe e suba a interface de rede sudo ifdown wlan1 sudo ifup wlan1
- Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

  1. Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo) cd ssh-keygen -t rsa

  2. Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo:
   "cd ssh pi@192.168.0.14"
   "mkdir -p .ssh "
   "cat .ssh/id_rsa.pub | ssh pi@192.168.0.14 'cat >> .ssh/authorized_keys'"

  3. Teste se funcionou com o comando abaixo ssh pi@192.168.0.1x 'ls'
    - *Confirme IP disponivel na rede do carro para atribuir aqui*

- Mude o endereço do servidor de pacotes (do apt-get) comentando a linha existente e adicionando a abaixo no arquivo indicado abaixo:
  "deb http://linorg.usp.br/raspbian/raspbian/ stretch main contrib non-free rpi"
  "sudo nano /etc/apt/sources.list"

- Instale libs necessárias:
  "sudo apt-get install libncurses5-dev sudo apt-get install nmap"

 - Baixe o Voice-Kit-Aiy (apenas) do github do LCAD (https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo) 

"sudo apt-get install subversion"
"svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/voice_kit_aiy"

### CRIANDO PRIMEIRO PROJETO E CREDENCIAIS

(Se quiser, siga os passos em inglês em: https://aiyprojects.withgoogle.com/voice/#users-guide-1-1--connect-to-google-cloud-platform)
**utilize o Chromium*

- Faça um conta no Google Cloud Plataform (GCP).
- Entre no Console do Cloud com sua ID e senha: https://console.cloud.google.com e siga os passos abaixo: 

 1. Crie projeto (acima, esquerda); 
 2. Clique em "Produtos e Servicos" (linha tripla) 
 3. Escolha "APIS e Servicos" 
 4. Procure por "Google Assistant API" e clique nele 
 5. Clique em "ENABLE" 
 6. Ainda em "APIS e Servicos" va em "Credentials" e crie uma credencial. Escolha "OAuth client ID" 
 7. Va em " Configure consent screen" 
 8. Entre com um nome do "produto" (Ex.: Voice-Assistant) e salve. 
 9. Clique em "Outros". Troque de nome que lembre a credencial (Ex.: Voice Recongnizer) 
 10. Feche a Pop-up que ira aparecer. Nao precisa guardar os numeros.

- Faca o download da credencial. O nome comecara com "client_secrets..." e estara na pasta Download.
- Renomeie a credencial para 'assistant.json' e mova para a pasta '/home/pi' --> No terminal: cd Downloas mv client_secret..... /home/pi/assistant.json
- Ative os controles do dispositivo conectado com sua Google ID em: https://myaccount.google.com/activitycontrols 
- Deixe ligado os seguintes "Controles de Atividades". 
  - Web and app acitivy. Inclua o checkbox de "Incluir historico de busca do Google...." 
  - Device Information
  - Voice and audio activity


##### REGISTANDO GOOGLE_APPLICATION_CREDENTAILS 

- Muitos scripts podem nao "encontrar" o retorno da GOOGLE_APPLICATION_CREDENTIAL (service). Entao, edite o arquivo:
    "$ nano ~/.bashrc"
  - No final, digite: export GOOGLE_APPLICATION_CREDENTIALS=escreva/aqui/caminho/da/credencial/arquivo-credencial.json
   - Teste se deu certo: 
    "$ set | grep GOOGLE_APPLICATION_CREDENTIALS"
    
    
##### OBSERVACOES 
- Para cada tipo de API que voce utilizar, deverá habilitá-la em: https://console.cloud.google.com/ > API e Servicos > Bibliotecas > "Buscar API desejada" > ENABLE
- Atenção! A chave-secreta assistant.JSON é apenas um tipo de credencial (OAuth). Algumas APIs utilizam "service credentials". Cada API tem sua documentação.


### APIS UTILIZADAS/HABILITADAS NO VOICE KIT (ATE' ABRIL/2018)
- Google Assistant SDK: https://developers.google.com/assistant/sdk/overview
- Cloud Speech API:  https://cloud.google.com/speech/docs/
- Text to Speech API: https://cloud.google.com/text-to-speech/docs/
- Google Cloud Translation API: https://cloud.google.com/translate/docs/?hl=th (nao


### TESTANDO DEMOS
*(Tambem em: https://aiyprojects.withgoogle.com/voice/#users-guide-3-1--start-the-assistant-library-demo-app)*

- Em *'/home/pi'*, tanto em *'AIY-voice-kit-python'*, quanto *'AIY-projects-python'*, terão uma pasta *'/src/examples/voice'* com suas demos.
- Teste primeiro o "assistant_library_demo.py"
  - Se Python 3.5, para nao dar error na demo "assistant_grpc_demo.py", siga os passos:
  1. Renomeie uma pasta 'futures' para 'oldfutures' dentro da '/bin': 
  "cd /usr/local/lib/python3.5/dist-packages/concurrent sudo mv futures oldfutures"
  2. Re-teste a demo
- Teste as demais demos

  **OBS.: A credencial da API Cloud Speech é do tipo SERVICE.**
  - Escolha o seu projeto > Va em "IAM e Admin" > "Contas de Serviço" > "+Criar conta de serviço" > Preencha as infos > Concluido
  - Ao lado direito do nome da chave, nos 3 pontos (...) verticais, va' em "Criar chave". Escolha a opção ".JSON" e façaa o download em "/home/pi".

### ASSISTANT-LIBRARY/ASSISTANT-GRPC/CLOUD-SPEECH EM PT-BR

- O servico de Cloud Speech utiliza a Speech API do Google, que suporta mais de 100 idiomas. Para uma interacao em Portugues-BR, fa;ca:
  - O arquivo que controla o idioma principal e' o "i18n.py" em "/home/pi/AIY-projects-python/src/aiy/"
  - Em "_DEFAULT_LANGUAGE_CODE = " coloque " 'pt-BR' ".
 *-Obs.: Algumas APIS que nao suportam esse idioma:
 1. ou ignoram sua mudanca e utilizam o ingles americano como padrao (maioria); ou 
 2. dão erro na sua execução;


### UTILIZANDO A API TEXT TO SPEECH 
*(pode seguir tambem em https://cloud.google.com/text-to-speech/docs/quickstart)*

- Instale o Google SDK (não necessariamente dentro do virtualenv) e siga as instruções: https://cloud.google.com/sdk/docs/
    "$ sudo pip install google-sdk"

- Volte para o passo-a-passo em: https://cloud.google.com/text-to-speech/docs/quickstart

###### CÓDIGO

- Codigo simples utilizando o TTS em: /AIY-projects-python/src/tts/tts_demo.py
 - *OBS.: Instale a biblioteca do player VLC para Python:*
    "$ sudo apt-get install vlc"

# NO SEU COMPUTADOR PARA USO DO TTS

 - Siga os dois últimos tópicos acima: _"Utilizando a API Text-To-Speech"_ e _"CÓDIGO"_
 - Instale o Google Cloud Text-to-Speech:
 "sudo pip install google-cloud-texttospeech"
 - Caso não tenha colocado suas credenciais como 'default', inicie o SDK:
 "gcloud init"
 - Selecione o seu projeto
 - Siga os demais passos em: *https://cloud.google.com/sdk/docs/*
 - Para colocar em português: na linha 39, em _"language_code"_ mude para 'pt-BR'
 
#### SUBINDO ARQUIVOS PARA GITHUB/CARMEN 

- Todas as mudanças futuras no github podem ser incorporadas com: 
  "svn up"
- Para subir coisas para o git use o commit do svn (que já sobe as mudanças). 
  - Exemplo: 
  "svn add _nomedoarquivo_"
  "svn commit -m "adicao de ReadMe"

- Pedirá usuário e senha. Se não for o seu, dar "Enter" e digitar seu usuário do GitHub


