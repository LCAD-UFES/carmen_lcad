Bom dia,

Estou em um processo para reorganizar o carmen_ipc_bridge para que ele seja mais facil de entender, modificar e manter, alem de preparar o terreno para uma atualizacao futura.

Para isso, vou deixar ele mais proximo de um modulo padrao do carmen. Para isso, use de referencia o arquivo program_with_Astro.md, dentro de doc, para as guidelines de codigo, considerando todos os pontos escritos "astro" como "carmen". Alem disso, para ter um modelo de referencia para os nodos do ros que serao montados, use o arquivo template_for_astro_or_carmen.cpp como um padrao para arquitetar o codigo, e caso tenha duvido pode acessar outros modulos em ~/carmen_lcad/src/ para ter de referencia, como o localize_ackerman. Note que o codigo eh quase sempre single thread, tendo como excecoes as funcoes nativamente multithread, como o subscribe do ros. Mantenha os comentarios em codigo menores.

Atualmente, a funcao principal do carmenipc_bridge eh converter mensagens carmen para ros. A mensagem de lidar eh convertida pelo pointcloud_node, e as outras sao convertidas pelo ipc_bridge_node. Meu objetivo final eh ter 1 ou 2 nodos para cada mensagem, em que 1 nodo seria para fazer a comunicacao carmen para ros, e outro nodo para fazer a comunicacao ros para carmen. Note que para a nossa aplicacao atual, focamos principalmente em pegar mensagens do carmen e converter para ros, que sao consumidas pelo sc_lio_sam modificado nesse workspace.

Para alcancar esse objetivo, vou passar varias etapas para voce ir implementando, uma por vez, e vou oferecendo feedback. Para a primeira etapa, quero que extraia a conversao astro->ros presente no ipc_bridge_node da mensagem de IMU Xsens e coloque em um novo nodo, que sera gerado por um arquivo .cpp chamado "carmen_to_ros_imu_xsens.cpp". Esse arquivo deve ser preferencialmente single thread, manter a mesma funcionalidade que ja existe no ipc_bridge_node.cpp e usar como template base o template_for_astro_or_carmen.cpp. Me pergunte qualquer duvida que tiver durante o processo.

Para executar testes pode executar o comando em ~/carmen_lcad/bin e dar play no proccontrol para tocar o log, e eh possivel ver a geracao do mapa no rviz:

cd ~/carmen_lcad/bin
./proccontrol argos/process-argos-playback-lio-sam.ini