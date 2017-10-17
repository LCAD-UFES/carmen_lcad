Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

- Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo 
  para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
 cd
 ssh-keygen -t rsa

- Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo
 cd
 ssh pi@192.168.0.14 mkdir -p .ssh
 cat .ssh/id_rsa.pub | ssh pi@192.168.0.14 'cat >> .ssh/authorized_keys'

- Teste se funcionou com o comando abaixo
 ssh pi@192.168.0.14 'ls'


Para rodar o módulo can_dump, antes tem que colocar as interface can junto ao Pi em modo bypass. Para isso, rode os comandos abaixo:
 ssh pi@192.168.0.14
 sudo ./rules_bypass.bat
Em seguida, rode o executavel can_dump no diretorio bin e o process que usa o ford_escape_hybrid (tem que rodar um e depois o outro
rapidamente para nao dar erro de IPC


===========================================================
- O can de cima azul pode tirar e colocar que o Torc nao pira
- O can de baixo verde, se tirar e recolocar, o Torc pira
