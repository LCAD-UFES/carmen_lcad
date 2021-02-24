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

Para visualizar e comparar a odometria visual com a odometria de um robo voce pode usar o seguinte comando de linha e os seguintes comandos de gnuplot:
 ./visual_odometry2 4 -publish_base_ackerman_odometry on -compare_odometries on

 gnuplot> plot "visual_odometry_graph.txt" u 5:3 w l t 'robot v', "visual_odometry_graph.txt" u 5:1 w l t 'visual o. v'
 gnuplot> plot "visual_odometry_graph.txt" u 5:4 w l t 'robot phi', "visual_odometry_graph.txt" u 5:2 w l t 'visual o. phi'

Existe ainda a opção de combinar a visual odometry com a odometria do carro. 
Caso de estudo: Quando só se tem a informação de giro do volante do robô pode se usar a visual odometry para gerar a velocidade.
Ness caso o ford_escape_hybrid (hardware) assina a mensagem de pose6d da visual odometry e monta a mensagem carmen_robot_ackerman_velocity_message combinando as odometrias.
Para usar essa opção deixe os parametros da seguinte forma:

visual_odometry_publish 			off
robot_publish_odometry				on
robot_publish_combined_visual_and_car_odometry 	on

E escolha a combinação da odometria com os parâmetros:
robot_combine_visual_and_car_odometry_phi       1 #
robot_combine_visual_and_car_odometry_vel       2 # 1-publish VISUAL_ODOMETRY_ velocity instead car velocity / 2-CAR_ODOMETRY_VEL / 3-COMBINE VISUAL and CAR ODOMETRY VELOCITY   

A opção 1 o ford_escape irá publicar o PHI/Velocidade da VISUAL_ODOMETRY em vez do phi/velocidade do hardware.
A opção 2 o ford_escape irá publicar o PHI/Velocidade do hardware em vez do phi/velocidade da VISUAL_ODOMETRY.
A opção 3 o ford_escape irá publicar uma combinação do PHI/Velocidade do hardware com o phi/velocidade da VISUAL_ODOMETRY usando algum algoritimo.

Mais detalhes em $CARMEN_HOME/src/ford_escape_hybrid/README.txt


