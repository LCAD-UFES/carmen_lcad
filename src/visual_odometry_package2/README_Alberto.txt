A mensagem carmen_robot_ackerman_velocity_message eh a verdadeira odometria (v e phi) vinda do hardware.
Esta mensagem eh transformada em carmen_base_ackerman_odometry_message pelo modulo base_ackerman, que aplica os multipliers e adiciona os bias de v e phi.

O modulo visual_odometry2 pode publicar ambas as mensagens mencionadas acima para dar mais flexibilidade. Ele publica ainda a mensagem carmen_visual_odometry_pose6d_message
e pode publicar uma global_pos baseada unicamente na odometria visual.

Para publicar uma global_pos baseada unicamente na odometria visual ative o parametro visual_odometry_is_global_pos no carmen ini.
Para publicar a carmen_base_ackerman_odometry_message use a flag na linha de comando: -publish_base_ackerman_odometry on

No modo mais usual, o visual_odometry2 publica a odometria no lugar do robo. Para ativar este modo, altere as variaveis visual_odometry_publish e robot_publish_odometry
no carmen ini como indicado abaixo:

visual_odometry_publish on
robot_publish_odometry	off

Note que os multipliers e os bias de v e phi do robo e da visual odometry, presentes no carmen ini, serao aplicados na visual odometry 
pela propria (ela tem estes parametros no carmen ini) e pelo modulo base_ackerman.

Para comparar a odometria visual com a odometria de um robo use as flags na linha de comando: -publish_base_ackerman_odometry on -compare_odometries on
Neste caso, será salvo no diretorio corrente o arquivo visual_odometry_graph.txt com o conteudo do fprintf:
		fprintf(graf, "%lf %lf %lf %lf %lf\n",
			visual_odometry.v, visual_odometry.phi,
			last_robo_odometry.v, last_robo_odometry.phi,
			visual_odometry.timestamp - first_timestamp);

Note que a carmen_base_ackerman_odometry_message nao sera publicada mas a funcao que a publica precisa ser chamada pois eh nela que se faz a
comparacao de odometrias.

Para visualizar e comparar a odometria visual com a odometria de um robo voce pode usar os seguintes comandos de gnuplot:
 gnuplot> plot "visual_odometry_graph.txt" u 5:3 w l t 'robot v', "visual_odometry_graph.txt" u 5:1 w l t 'visual o. v'
 gnuplot> plot "visual_odometry_graph.txt" u 5:4 w l t 'robot phi', "visual_odometry_graph.txt" u 5:2 w l t 'visual o. phi'

