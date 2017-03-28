Para usar modulos do carmen em rede usando centrals diferentes
Eh necessario usar o multicentral. 
Com Multicentral, dois centrals em maquinas diferentes podem trocar mensagens.
Este modo, evita que um central so cuide de todos os modulos da rede, gerando latencia.

Atualmente existe uma biblioteca chamada multicentral.h ($CARMEN_HOME/src/global/multicentral.h)
essa biblioteca eh um program carmen que aplica a tecnica explicada no manual do IPC para comunicar dois centrals.[($CARMEN_HOME/doc/IPC_Manual.pdf)]

Para usar essa biblioteca conectando mais de uma maquina a IARA. Foi criado um modulo multicentral_bridge.
Esse modulo se subscreve em um dos centrals e republica as mensagens necessarias para o outro central. 
o pograma multicentral_bridge_traffic_light.c faz essa ponte para o modulo traffic light. Recebendo Odometria e annotacoes da car01, 
recebendo a camera no central da car02, e publicando de volta a deteccao para a car01. Deste modo a Bumblebee fica na car02, e as mensagens da bumblebee
nao precisam trafegar pela rede, gerando latencia.

Uso:
 A biblioteca busca no CENTRALHOST os centrals ativos. Para um central na rede, eh necessario usar o parametro -central e um txt com a lista de centrals
 ./bridge_traffic_light -central centrals_list.txt

Em resumo a tecnica, se trata de uma troca de contexto entre os centrals, assim se escuta um central por vez em uma frequencia alta. 
Define-se as mensagens que quer se subscrever, e na troca de contexto, o central que tiver publicando aquela mensagem sera recebido pelo subscriber.

