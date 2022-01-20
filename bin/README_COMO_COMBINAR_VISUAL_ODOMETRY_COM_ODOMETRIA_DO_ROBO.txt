############################################
#######      ATENÇÂO!!    ##########
# Se você já conhece sobre o sistema para COMBINAR AS ODOMETRIAS e como funciona a visual_odometry2, pule para RESUMO:
#############################################

A mensagem carmen_robot_ackerman_velocity_message eh a verdadeira odometria (v e phi) vinda do hardware.
Esta mensagem eh transformada em carmen_base_ackerman_odometry_message pelo modulo base_ackerman, 
que aplica os multipliers e adiciona os bias de v e phi.

O modulo visual_odometry2 pode publicar ambas as mensagens mencionadas acima para dar mais flexibilidade. 
Ele publica ainda a mensagem carmen_visual_odometry_pose6d_message e pode publicar uma global_pos baseada 
unicamente na odometria visual.

Para publicar uma global_pos baseada unicamente na odometria visual ative o parametro visual_odometry_is_global_pos no carmen ini.
Para publicar a carmen_base_ackerman_odometry_message use a flag na linha de comando: -publish_base_ackerman_odometry on

No modo mais usual, o visual_odometry2 publica a odometria no lugar do robo. Para ativar este modo, altere as variaveis visual_odometry_publish e robot_publish_odometry
no carmen ini como indicado abaixo:

visual_odometry_publish on
robot_publish_odometry	off

Existe ainda a opção de combinar a visual odometry com a odometria do carro. 
Caso de estudo: Quando só se tem a informação de giro do volante do robô pode se usar a visual odometry para gerar a velocidade.
Ness caso o ford_escape_hybrid (hardware) assina a mensagem de pose6d da visual odometry e monta a mensagem 
carmen_robot_ackerman_velocity_message combinando as odometrias.
Para usar essa opção deixe os parametros da seguinte forma:

visual_odometry_publish 			off
robot_publish_odometry				on
robot_publish_combined_odometry 	on

E escolha a combinação da odometria com os parâmetros:
robot_combine_odometry_phi       1 #
robot_combine_odometry_vel       2 # 1-publish VISUAL_ODOMETRY_ velocity instead car velocity / 2-CAR_ODOMETRY_VEL / 3-COMBINE VISUAL and CAR ODOMETRY VELOCITY   

A opção 1 o ford_escape irá publicar o PHI/Velocidade da VISUAL_ODOMETRY em vez do phi/velocidade do hardware.
A opção 2 o ford_escape irá publicar o PHI/Velocidade do hardware em vez do phi/velocidade da VISUAL_ODOMETRY.
A opção 3 o ford_escape irá publicar uma combinação do PHI/Velocidade do hardware com o phi/velocidade da VISUAL_ODOMETRY usando algum algoritimo.

Outros modulos do carmen também tem essa função para que possa ser usado em níveis diferentes
Para o caso de não querer combinar a odometria no Ford_escape, é possível fazer o visual_odometry2 publicar a odometria do robô, junto com o ford_escape (sem combinar)
E passar para o Base_ackerman fazer a combinação (útil para logs)
1- Base_ackerman
Para esse modo usar a flag 
base_ackerman_publish_combined_odometry	on # 
Tem que estar em sintonia com a robot_publish_combined_odometry. Para calibracao de odometria de logs esta flags pode ser ativada para misturar odometria visual com a do robo 
e escolher a combinação da odometria com os parâmetros: 
robot_combine_odometry_phi       1 #
robot_combine_odometry_vel       2 # 1-publish VISUAL_ODOMETRY_ velocity instead car velocity / 2-CAR_ODOMETRY_VEL / 3-COMBINE VISUAL and CAR ODOMETRY VELOCITY   

Assim é possível escolher na hora de fazer mapa, qual melhor combinação, porém os modulos de calibração, leem direto do arquivo de log. 
uma vez gravada a mensagem  carmen_robot_ackerman_velocity_message pelo ford_escape_hybrid e visual_odometry2.
Os modulos abaixo vão ler do log e combinar como desejar 
2- Odometry_calibration
Para ativar a combinação no Odometry_calibration é necessário alterar o arquivo de parametros
calibrate_combined_visual_and_car_odometry = 1
e usar os mesmos parametros para escolher a combinação no carmen.ini que é passado na linha de comando do "./calibrate_bias_from_log"

3 - Grab_data (src/graphslah/grab_data_from_log.cpp)
Atualmente é necessário ativar a flag dentro do código calibrate_combined_visual_and_car_odometry e a combinação esta na main (pesquisar a variavel combine_visual_and_car_odometry_phi)
combine_visual_and_car_odometry_phi
combine_visual_and_car_odometry_vel

Publish_Poses
Uma vez que as mensagens do grab data foram filtradas, o graphslam já tentará usar a mensagem correta combinada.
O publish_poses precisa que o base_ackerman esteja publicando também a combinação desejada, para sincronizar as mensagens. Veja o ponto 1- Base_ackerman acima

##############
RESUMO:
#############
- Para Gravar Log:
    É possível gravar as duas odometrias (visual e Robô) como carmen_robot_ackerman_velocity_message para
    combinar a posteriori(i) ou usar o Ford_escape para gravar a combinação desejada (ii) porém,
    essa forma pode gerar problemas de sincronização prefira a primeira. 
    As mensagens são diferenciadas pelo campo host que identifica qual modulo publicou.
     (i) Modifique os parametros no carmen-...ini para o modo abaixo:
        visual_odometry_publish 			on (isso faz a odometria visual publicar a carmen_robot_ackerman_velocity_message)
        robot_publish_odometry				on
        robot_publish_combined_odometry 	on
      E escolha a combinação da odometria com os parâmetros:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel       
    (ii) Para gravar já combinado Modifique os parametros no carmen-...ini para o modo abaixo:
        visual_odometry_publish 			off
        robot_publish_odometry				on
        robot_publish_combined_odometry 	on
      E escolha a combinação da odometria com os parâmetros:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel       
    Rode o módulo visual_odometry2 e toda a infraestrutura necessaria pro log
- Para tocar o log:
    O base_ackerman fará a combinação da odometria
    Modifique os parametros no carmen-...ini para o modo abaixo:
        visual_odometry_publish 			off
        robot_publish_odometry				on
        robot_publish_combined_odometry 	off
        base_ackerman_publish_combined_odometry on
     E escolha a combinação da odometria com os parâmetros:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel       

- Para andar autonomo:
    Em modo autonomo é possivel usar diretamente o ford_escape (ele recebe a carmen_visual_odometry_pose6d_message e a odometria do veículo via driver)
        visual_odometry_publish 			off
        robot_publish_odometry				on
        base_ackerman_publish_combined_odometry off
        robot_publish_combined_odometry 	on
     E escolha a combinação da odometria com os parâmetros:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel     

- Para fazer mapas:

    Para fazer mapas é necessário ter gravado o log com os modos citados acima, melhor usar o modo (i) para não ter problema de sincronização.
    Com log gravado Para calibrar a odometria com o calibrate_bias_from_log Modifique em 
    calibrate_bias_from_log_config.txt:
        calibrate_combined_visual_and_car_odometry = 1
    E escolha a combinação da odometria com os parametros no carmen-...ini:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel   
    Grab_data (src/graphslah/grab_data_from_log.cpp)
        Atualmente é necessário ativar a flag dentro do código calibrate_combined_visual_and_car_odometry e a combinação esta na main (pesquisar a variavel combine_visual_and_car_odometry_phi)
            combine_visual_and_car_odometry_phi
            combine_visual_and_car_odometry_vel

    Após isso a odometria nos arquivos que serão usados pelo graphslah já vão estar combinados e prontos para otimização.

    Publish_Poses
        O publish_poses precisa que o base_ackerman esteja publicando também a combinação desejada, para sincronizar as mensagens.
          O base_ackerman fará a combinação da odometria
     Modifique os parametros no carmen-...ini para o modo abaixo:
        visual_odometry_publish 			off
        robot_publish_odometry				on
        robot_publish_combined_odometry 	off
        base_ackerman_publish_combined_odometry on
     E escolha a combinação da odometria com os parâmetros:
        robot_combine_odometry_phi       
        robot_combine_odometry_vel       
    Lembre-se que no modo de mapeamentos a FUSED_ODOMETRY DEVE ESTAR DESLIGADA! o Publish_Poses já publica essa mensagem.
