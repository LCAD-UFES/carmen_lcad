Crie o novo branch (já cria e entra nele com os arquivos que já estao modificados previamente inclusive)
 git checkout -b novobranch
 git commit -a -m "commit inicial do branch novobranch"
 git push origin novobranch

Depois destes comandos você está no novobranch de mesmo modo que estaria no master (ou main) e pode trabalhar.
Contudo, novos pushs tem que ser como acima. Alternativamente, execute o comando:
 git push --set-upstream origin novobranch

Depois dele, novos git push vao para novobranch.

Para voltar para o branch master (main)
 git checkout master

Para voltar para novobranch
 git checkout novobranch
