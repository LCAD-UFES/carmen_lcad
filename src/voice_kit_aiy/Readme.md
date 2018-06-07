## PASSOS INICIAIS

### Criando Primeiro Projeto e Credenciais

- Faça um conta no Google Cloud Plataform (GCP).
- Entre no Console do Cloud com sua ID e senha: <https://console.cloud.google.com> e siga os passos abaixo: 

 1. Crie projeto (acima, esquerda); 
 2. Clique em "Produtos e Serviços" _("linha tripla")_;
 3. Escolha "APIS e Serviços";
 4. Procure por "Google Assistant API" e clique nele;
 5. Clique em "ENABLE";
 6. Ainda em "APIS e Serviços", vá em "Credentials" e crie uma credencial. Escolha "OAuth client ID"; 
 7. Vá em " Configure consent screen";
 8. Entre com um nome do "produto" _(Ex.: Voice-Assistant)_ e salve;
 9. Clique em "Outros". _(Opcional)_Troque de nome que lembre a credencial _(Ex.: Voice Recongnizer)_ ;
 10. Feche a Pop-up que irá aparecer.

- Faça o download da credencial, de preferência na _/home_. O nome comecará com "client_secrets...";
- Ative os controles do dispositivo conectado com sua Google ID em: <https://myaccount.google.com/activitycontrols> 
- Deixe ligado os seguintes "Controles de Atividades": 
  - Web and app acitivy. Inclua o checkbox de "Incluir histórico de busca do Google...." 
  - Device Information
  - Voice and audio activity


##### Registrando GOOGLE_APPLICATION_CREDENTAILS 

- Muitos scripts podem não "encontrar" o retorno da GOOGLE_APPLICATION_CREDENTIAL (service). Então, edite o arquivo:
    "$ nano ~/.bashrc"
  - No final, digite: export GOOGLE_APPLICATION_CREDENTIALS=/caminho/da/credencial/arquivo-credencial.json
   - Teste se deu certo: 
    "$ set | grep GOOGLE_APPLICATION_CREDENTIALS"
    
    
##### Observações
- Para cada tipo de API que voce utilizar, deverá habilitá-la em: <https://console.cloud.google.com/> > API e Serviços > Bibliotecas > "Buscar API desejada" > ENABLE.
- Atenção! A chave-secreta _assistant.JSON_ é apenas um tipo de credencial (OAuth). Algumas APIs utilizam "service credentials". Cada API tem sua documentação.


### Utilizando a API Text-to-Speech
*(pode seguir tambem em https://cloud.google.com/text-to-speech/docs/quickstart)*

- Instale o Google SDK e siga as instruções: <https://cloud.google.com/sdk/docs/>
    "$ sudo pip install google-sdk"

- Volte para o passo-a-passo em: <https://cloud.google.com/text-to-speech/docs/quickstart>

###### Código-Base

- Código simples utilizando o TTS em: _/AIY-projects-python/src/tts/tts_demo.py_
 - *OBS.: Instale a biblioteca do player VLC para Python:*
    "$ sudo apt-get install vlc"

# NO SEU COMPUTADOR PARA USO DO TTS

 - Siga os dois últimos tópicos acima: _"Utilizando a API Text-To-Speech"_ e _"Código-Base "_
 - Instale o Google Cloud Text-to-Speech:
 "$ sudo pip install google-cloud-texttospeech"
 - Caso não tenha colocado suas credenciais como 'default', inicie o SDK:
 "gcloud init"
 - Selecione o seu projeto.
 - Siga os demais passos em: *https://cloud.google.com/sdk/docs/*
 - Para colocar em português: na linha 39, em _"language_code"_ mude para 'pt-BR'
 
## USANDO O VOICE KIT AIY NO RASPBERRY
_(Se quiser, siga os passos em inglês em: https://aiyprojects.withgoogle.com/voice/#users-guide-1-1--connect-to-google-cloud-platform)_

 
# == Para colocar para operar o Voice Kit AIY - Raspberry Pi ==

-Conectar os cabos ligando Raspberry e Voice Hat como mostrado em: <https://aiyprojects.withgoogle.com/voice/#assembly-guide-2-assemble-the-hardware>
- Use um cartão micro SD para gravar a ISO do Voice Kit. _(Download em:_ <magpi.cc/2x7JQfS> _)._ (Gravação da ISO com ETCHER: <magpi.cc/2fZkyJD>).
- Conecte os periféricos necessários: Teclado USB, Mouse USB, Cabo HDMI, cabo de energia _(Ex.: celular, corrente>=2,0 A)_.
- Com o boot do Raspberry Pi, o led dentro da caixa se acenderá. Caso apareca "Openbox Syntax Error", você terá que reescrever a ISO no cartão SD.
- Clique duas vezes no ícone "Check Audio". Você ouvirá "Front, Centre" e uma mensagem na tela. Responda de acordo as instruções. Em caso de erro, siga a solução mostrada na mensagem.


### APIS utilizadas/habilitadas no Voice Kit (Até abril/2018)
- Google Assistant SDK: <https://developers.google.com/assistant/sdk/overview>
- Cloud Speech API:  <https://cloud.google.com/speech/docs/>
- Text to Speech API: <https://cloud.google.com/text-to-speech/docs/>
- Google Cloud Translation API: <https://cloud.google.com/translate/docs/?hl=th>


### TESTANDO DEMOS
*(Também em:* <https://aiyprojects.withgoogle.com/voice/#users-guide-3-1--start-the-assistant-library-demo-app> *)*

- Em *'/home/pi'*, tanto em *'AIY-voice-kit-python'*, quanto *'AIY-projects-python'*, há uma pasta *'/src/examples/voice'* com demos.
- Teste primeiro o "assistant_library_demo.py"
  - Use Python 3.5, para não dar error na demo "assistant_grpc_demo.py", siga os passos:
  1. Renomeie uma pasta _'futures'_ para _'oldfutures'_ dentro da _'/bin'_: 
  "cd /usr/local/lib/python3.5/dist-packages/concurrent sudo mv futures oldfutures"
  2. Re-teste a demo
- Teste as demais demos

#### OBS.: A credencial da API Cloud Speech é do tipo SERVICE
  - Escolha o seu projeto > Vá em "IAM e Admin" > "Contas de Serviço" > "+Criar conta de serviço" > Preencha as infos > Concluído
  - Ao lado direito do nome da chave _(linha tripla)_, vá em "Criar chave". Escolha a opção ".JSON" e faça o download em "/home/pi".

### ASSISTANT-LIBRARY/ASSISTANT-GRPC/CLOUD-SPEECH EM PT-BR

- O servico de Cloud Speech utiliza a Speech API do Google, que suporta mais de 100 idiomas. Para uma interação em Português-BR, façca:
  - O arquivo que controla o idioma principal e' o "i18n.py" em "/home/pi/AIY-projects-python/src/aiy/"
  - Em "_DEFAULT_LANGUAGE_CODE = " coloque " 'pt-BR' ".
- Obs.: Algumas APIS que não suportam esse idioma:
 1. ou ignoram sua mudança e utilizam o inglês americano como padrão (maioria); ou 
 2. dão erro na sua execução;



### == ATIVANDO A REDE ==

####Quando for colocar o Raspberry na rede da IARA

- Para ativar a rede wired: <https://www.modmypi.com/blog/how-to-give-your-raspberry-pi-a-static-ip-address-update>
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

 - Baixe o Voice-Kit-Aiy (apenas) do github do LCAD (<https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo>) 

"sudo apt-get install subversion"
"svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/voice_kit_aiy"

 
#### SUBINDO ARQUIVOS PARA GITHUB/CARMEN 

- Todas as mudanças futuras no github podem ser incorporadas com: 
  "svn up"
- Para subir coisas para o git use o commit do svn (que já sobe as mudanças). 
  - Exemplo: 
  "svn add _nomedoarquivo_"
  "svn commit -m "adicao de ReadMe"

- Pedirá usuário e senha. Se não for o seu, dar "Enter" e digitar seu usuário do GitHub


