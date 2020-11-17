1- Copie o arquivo com o rddf para o bin (tem que ser para o rddf_edited.txt)
ex:
 cp ../data/rndf/rddf-log_voltadaufes-20160513.txt rddf_edited.txt
 
2- Se estiver usando o route_planner para publicar o rddf 
   Desabilite o modo grafo do route_planner no carmen-ford-escape.ini (ou o .ini que voce estiver usado)
ex:
        gedit $CARMEN_HOME/src/carmen-ford-escape.ini
Procure o parametro e coloque como off

        route_planner_in_graph_mode				off

3- Rode seu process e Na interface do navigator_gui2 selecione Goals->Edit Rddf Goals
 - Isso vai abrir o arquivo rddf_edited.txt e exibi-lo em vermelho no navigator_gui2
 
3- Ajuste o mapa para ver claramente a pose do RDDF que voce quer editar 
 - Voce pode colocar o robo em algum ponto do mapa para trocar a regiao de interesse
 - Voce pode dar zoom e centralizar
 - Voce pode alterar o process ini para que o mapa venha na posicao apropriada
   alterando os parametros -map_x 7757168.8 -map_y -363636.0 de map_server 
   (e reiniciar tudo, e repetir os passos 1 e 2)
   
3- Selecione a pose do rddf que voce quer alterar da seguinte forma:
 (a) Clique em 2D Map (fica abaixo de File)
 (b) Aperte a tecla seta para baixo (dificil explicar por que isso eh necessario...)
 (c) Selecione o pose do RDDF que voce quer editar com o botao esquerdo do mouse -> ela vai ficar amarela
 (d) Use:
     - As setas para mover para esquerda, direita, para cima e para baixo
     - As letras "a" e "s" para girar
     - As letras "n" e "p" para selecionar a pose seguinte e a anterior
     - A tecla "d" para deletar uma pose do rddf
     - A letra "f" para parar de editar
     - O botao esquerdo do mouse para selecionar outra pose e voltar a editar
     
4- Equanto estiver editando nao reposicione o robo ou tecle outras coisas na tela sem antes teclar "f"
para desmarcar o que estiver editando

5- Para salvar a qualquer momento deselecione Goals->Edit Rddf Goals
 - Se nao quiser salvar, mate o navigator_gui2

6- Para usar o novo rddf
 - Copie-o para o lugar desejado
 cp rddf_edited.txt ../data/rndf/rddf-log_voltadaufes-20160513.txt

 - Nao se esqueca de remover os indices do rddf anterior
 rm ../data/rndf/rddf-log_voltadaufes-20160513.txt.*


