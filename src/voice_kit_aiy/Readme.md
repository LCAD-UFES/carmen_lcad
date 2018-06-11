[TOC]

### 1. Criando Primeiro Projeto e Credenciais no Google Cloud Plataform (GCP)

1. Faça um conta no [Google Cloud Plataform (GCP)](https://console.developers.google.com/).
2.  Entre no Console do Cloud com sua ID e senha e siga os passos abaixo:

 1. Crie um projeto ;
 2. No menu, escolha "APIS e Serviços";
		1. Em _Biblioteca_: procure pela API desejada e ative-a _(ENABLE)_;
		2. Em _Credenciais_:
		
		b.1) Credencial tipo _"OAuth client ID"_:
	>1. Clique em "Outros". (Opcional) Troque para um nome que lembre a credencial (Ex.: Voice Recongnizer) ;
	> 2. Vá em " Configure consent screen/Tela de Consentimento OAuth";
	> 3. Entre com um nome do "produto" (Ex.: Voice-Assistant) e salve;
	> 4. Feche a Pop-up que irá aparecer.

		b.2) Credencial tipo  _"Conta de Serviço/Service Account"_
	> 1. No menu suspenso, escolha o seu projeto ;
	> 2. Marque o tipo de chave como .'JSON';
	> 3. Clique em "Criar";
	> 4. Salve a credencial.

4. Faça o download da credencial, de preferência na _/home_. O nome comecará com "client_secrets..." para OAuth ID e "nome-produto...." para a "Service Account";
5. [Ative os controles](https://myaccount.google.com/activitycontrols) do dispositivo conectado com sua Google ID;

	Deixe ligado os seguintes "Controles de Atividades":
	1. "Web and app acitivy" . Inclua o checkbox de "Incluir histórico de busca do Google...."
	2. "Device Information"
	3. "Voice and audio activity"



#####1.1. Registrando GOOGLE_APPLICATION_CREDENTAILS

1. Edite o arquivo:
    `nano ~/.bashrc`
  1. No final, digite:
  `export GOOGLE_APPLICATION_CREDENTIALS=/caminho/da/credencial/arquivo-credencial.json`
2. Teste se deu certo: 
    `$ set | grep GOOGLE_APPLICATION_CREDENTIALS`

**Observações**
- Para cada tipo de API que você utilizar, deverá habilitá-la em:
[GCP](https://console.cloud.google.com/) > "API e Serviços">"Bibliotecas">"_Nome_da_API_" > "ENABLE".

- **Atenção!**
A chave-secreta "cliente_secret....JSON" é apenas um tipo de credencial (OAuth).  Algumas APIs utilizam "service credentials". Cada API tem sua documentação.
	- Exemplo: A Text-to-Speech API usa a "Service Credential";

## 2. Registro de conta para possíveis faturamentos
 Pré-Requisitos: Você precisa ser "Propietário" de um projeto.

   1. Vá para [Google Cloud Plataform Console](https://console.developers.google.com/).
   2. No menu, selecione "Billing/Faturamento".
   3. Embaixo de "Projetos" linkados com a conta bancária escolhida, localize o nome do projeto desejado e depois clique no menu próximo a ele.
   5. Selecione "Change billing account/Mudar Conta de Faturamento", depois escolha o destino desejado da conta bancária.
   6. Clique em "Set account".

## 3. Utilizando a API Text-to-Speech
1. Instale o [Google SDK](https://cloud.google.com/sdk/docs/):
    `$ sudo apt install google-sdk`
	- Se não der certo, tente:
	`sudo apt-get update && sudo apt-get install google-cloud-sdk`

2. Inicie o Google Cloud SDK:
`gcloud init`
`gcloud auth activate-service-account --key-file=[PATH]`
_(substitua [PATH] pelo caminho até a credencial de serviço)_
	3. Selecione o projeto desejado.
	4. Siga os passos mostrados no prompt e o SDK estará pronto para uso.

3. Instale o Google Cloud Text-to-Speech:
 `$ sudo pip install google-cloud-texttospeech`

4. Scripts TTS
`python script.py`

	- Caso não tenha nenhum script que use o TTS, use exemplos do [GitHub/Python Docs Samples](https://github.com/GoogleCloudPlatform/python-docs-samples/tree/master/texttospeech/cloud-client).


**Obs.:** Para mais usos dos comandos "gcloud", veja o [guia](https://cloud.google.com/sdk/docs/quickstart-debian-ubuntu).



## 4. Usando o Voice Kit AIY no Raspberry
_([AIY - User's Guide](https://aiyprojects.withgoogle.com/voice/#users-guide-1-1--connect-to-google-cloud-platform))_

#### 4.1. Para colocar para operar o Voice Kit AIY - Raspberry Pi

1. [Conectar os cabos ligando Raspberry e Voice Hat](https://aiyprojects.withgoogle.com/voice/#assembly-guide-2-assemble-the-hardware)
2. Use um cartão micro SD para gravar a [ISO do Voice Kit](magpi.cc/2x7JQfS);
 3.  _(Gravação da ISO com ETCHER: magpi.cc/2fZkyJD)_.
4. Conecte os periféricos necessários: Teclado USB, Mouse USB, Cabo HDMI, cabo de energia _(Ex.: celular, corrente>=2,0 A)_.
5. Com o boot do Raspberry Pi, o led dentro da caixa se acenderá. Caso apareça _"Openbox Syntax Error"_, você terá que reescrever a ISO no cartão SD.
6. Clique duas vezes no ícone "Check Audio":
 6. Você ouvirá "Front, Centre" e uma mensagem na tela. 
 7. Responda de acordo as instruções. Em caso de erro, siga a solução mostrada na mensagem.


#### 4.2 Testando DEMOS
_[Start The Assistant Library Demo App](https://aiyprojects.withgoogle.com/voice/#users-guide-3-1--start-the-assistant-library-demo-app)_

 Tanto em 'AIY-voice-kit-python', quanto 'AIY-projects-python', há uma pasta '/src/examples/voice' com demos.
1. Teste primeiro o "assistant_library_demo.py":

2. Em  "assistant_grpc_demo.py", use Python 3.5, para não dar erros na demo. Siga, também, os passos:
	1. Renomeie uma pasta _'futures'_ para _'oldfutures'_ dentro da _'/bin'_: 
  `cd /usr/local/lib/python3.5/dist-packages/concurrent `
  `sudo mv futures oldfutures`
	2. Re-teste a demo
2.  Teste as demais demos

#### 4.3. Assistant-Library/Assistant-GRPC/Cloud-Speech em PT-BR

 O serviço de Cloud Speech e Text-to-Speech utilizam a Speech API do Google, que suporta mais de 100 idiomas. Para uma interação em Português-BR:
 
 O arquivo que controla o idioma principal é o  "i18n.py" em "/home/pi/AIY-projects-python/src/aiy/". 
 
 Edite-o:
  `_DEFAULT_LANGUAGE_CODE = 'pt-BR' `


#### 4.4. Ativando a Rede

##### 4.4.1. Quando for colocar o Raspberry na rede da IARA

1.  Para ativar a rede wired: <https://www.modmypi.com/blog/how-to-give-your-raspberry-pi-a-static-ip-address-update>
2. Para incluir uma rede WiFi (note que o Raspberry 3 tem hardware de wifi e Bluetooth nativos), edite _/etc/wpa_supplicant/wpa_supplicant.conf_ e inclua sua rede:
`sudo nano /etc/wpa_supplicant/wpa_supplicant.conf`
3. Em seguida, baixe e suba a interface de rede:
` sudo ifdown wlan1 sudo ifup wlan1`
4. Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

   1. Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo para gerá-la em _~/.ssh/id_rsa.pub_ (verifique se você já tem o arquivo para não gera-lo de novo) 
  ` cd ssh-keygen -t rsa`

  2. Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo:
   `cd ssh pi@192.168.0.14`
   ` mkdir -p .ssh `
   ` cat .ssh/id_rsa.pub | ssh pi@192.168.0.14 'cat >> .ssh/authorized_keys`

   3. Teste se funcionou com o comando abaixo:
  ` ssh pi@192.168.0.1x 'ls'`
    - *Confirme IP disponivel na rede do carro para atribuir aqui*

 4. Mude o endereço do servidor de pacotes (do apt-get) comentando a linha existente e adicionando a abaixo no arquivo indicado abaixo:
  ` deb http://linorg.usp.br/raspbian/raspbian/ stretch main contrib non-free rpi`
  ` sudo nano /etc/apt/sources.list`

 5.  Instale libs necessárias:
  ` sudo apt-get install libncurses5-dev sudo apt-get install nmap`

 6. Baixe o Voice-Kit-Aiy (apenas) do github do LCAD: [Download a SIngle Folder or Directory from a GitHub Repo](https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)

    ` sudo apt-get install subversion`
    ` svn checkout https://github.com/LCAD-UFES/carmen_lcad/trun/src/voice_kit_aiy`

### 5. Subindo Arquivos para o GitHub/Carmen

1. Todas as mudanças futuras no github podem ser incorporadas com:
  ` svn up`
2. Para subir novas coisas para o git use o commit do _svn_ (que já sobe as mudanças).
 - Exemplo:
  ` svn add _nomedoarquivo_`
  ` svn commit -m "adicao de ReadMe`

 -  Pedirá usuário e senha. Se não for o seu, dar "Enter" e digitar seu usuário do GitHub
