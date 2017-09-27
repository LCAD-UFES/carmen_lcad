Para permitir rodar comandos remotamente no Raspberry Pi siga os passos abaixo:

- Se você ainda não tem uma chave pública no computador que vai acessar o Pi, execute os comando abaixo 
  para gera-la em ~/.ssh/id_rsa.pub (verifique se você já tem o arquivo para não gera-lo de novo)
 cd
 ssh-keygen -t rsa

- Copie a chave pública do computador que vai acessar o Pi para o Pi com os comando abaixo
 cd
 ssh pi@192.168.0.13 mkdir -p .ssh
 cat .ssh/id_rsa.pub | ssh pi@192.168.0.13 'cat >> .ssh/authorized_keys'

- Teste se funcionou com o comando abaixo
 ssh pi@192.168.0.13 'ls'


Para rodar o módulo can_dump, antes tem que colocar as interface can junto ao Pi em modo bypass. Para isso, rode os comandos abaixo:
 ssh pi@192.168.0.13
 sudo ./rules_bypass.bat
