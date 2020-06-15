#Indice:
  -Com criar um mapa usando HyperGraphSCLAM.
  -Com fazer merge de poses para merge de mapas.

##Como criar um mapa usando HyperGraphSCLAM:

HyperGraphSCLAM pode tanto fazer a calibração da odometria, quanto usar do módulo odometry_calibration, 

seguiremos usando o odometry_calibration.

Compile os módulos graphslam, odometry_calibration e hypergraphsclam

    make -C $CARMEN_HOME/src/odometry_calibration
    make -C $CARMEN_HOME/src/graphslam
    make -C $CARMEN_HOME/src/hypergraphsclam


1. Edite o process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini (ou outro equivalente ajustado para o seu caso). 
   Lembre-se de colocar nele o seu log, o rddf desejado, e carmen ini EM TODOS OS LUGARES QUE PRECISA- busque por "log" para garantir.

2. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation_hypergraphsclam.ini (ou outro equivalente ajustado para o seu caso). 

3. Ligue e desligue, no PROCCONTROL GUI, o botão ClTmpDir (clique o botão ClTmpDir e ecolha Start Program, e depois Stop Program) para inicializar o diretório $CARMEN_HOME/bin/tmp.
 
4. Execute a calibração da odometria clicando, no PROCCONTROL GUI, no botão CalibOdo e escolhendo Show Output (para saber quando o programa terminou) e, em seguida, clicando no 
   botão CalibOdo e ecolhendo Start Program. 
	Notas:
		- Você DEVE delimitar o "tempo inicial" e "final" a ser considerado no log verificando o tempo no playback control e escolhendo apropriadamente (TEMPO, não timestamp).
		- Alternativamente, use as flags -l <linha inicial>  e -m <numero de linhas a partir da linha inicial> do arquivo $CARMEN_HOME/bin/calibrate_bias_from_log_config.txt
		- Existem outras opções importantes no $CARMEN_HOME/bin/calibrate_bias_from_log_config.txt
		- Rode mais de uma vez e observe qual a melhor calibração no gráfico final.

  4.1. Altere as variaveis abaixo no carmen-ford-escape.ini de acordo com os Resultados do calibrate_bias_from_log (tmp/calibrated_odometry.txt):
    robot_phi_multiplier			1.056087
    robot_phi_bias					1.065384
    robot_v_multiplier				0.003477


	Notas:
		- Voce pode usar o tmp/odom_poses_opt.txt para fazer mapas sem graphslam! Basta usa-lo como se fosse a saida do graphslam para o 
		  log. Para isso, no process de fazer mapas, basta usar a linha do graphslam_publish como abaixo:
 Publish_poses		graphslam	1		0			./graphslam_publish tmp/odom_poses_opt.txt

5. 
  5.1.Crie o arquivo de configuração do parser do hypergraphsclam para seu log. Use um exemplo em $CARMEN_HOME/src/hypergraphsclam/config/volta_ufes_normal_parser_config.txt
  5.2. Altere as variaveis abaixo no seu arquivo obitidas no odometry calibration. 
        -- PSO
        ODOMETRY_BIAS 1.030113 0.972890 -0.001412 
  5.3. Dentro do arquivo existem diversos parâmetros, os principais para o parser são:
   
	DISABLE_VELODYNE_ODOMETRY  <- Se o GPS estiver bom mantenha descomentado. 
	DISABLE_VELODYNE_LOOP      <- Se seu log tiver loops comente para habilitar
	DISABLE_SICK_ODOMETRY      <- Manhtenha descomentado, ainda não usado para mapas
	DISABLE_SICK_LOOP          <- Manhtenha descomentado, ainda não usado para mapas
	DISABLE_BUMBLEBEE_ODOMETRY <- Comente se quiser usar odometria visual
	DISABLE_BUMBLEBEE_LOOP     <- Comente se quiser usar odometria visual

Outros parâmetros tem comentários direto no arquivo.	

6.Crie o arquivo .list que conterá a lista de logs e seus rspectivos arquivos de configuração.
O arquivo segue o seguinte formato:
caminho_completo_do_log arquvivo_parser_config.txt arquivo_de_parametros_do_robo(para a IARA seria o carmen-ford-escape.ini)
Ex:
/dados/log_vale-20200311-5.txt log_vale_parser_config.txt carmen-ford-escape.ini

7. Dentro da pasta $CARMEN_HOME/bin/tmp/sync rode o parser pasando o arquivo como parâmetro: 
  ../../../src/hypergraphsclam/parser ../../config/log_ufes_aeroporto.list
  
8. Crie o arquivo de parametros de otmização para seu log. use um exemplo como $CARMEN_HOME/src/hypergraphsclam/config/volta_ufes_optimization_config.txt  
 
9. Dentro da pasta $CARMEN_HOME/bin/tmp/poses rode o otimizador pasando o arquivo como parâmetro: 
  ../../../src/hypergraphsclam/hypergraphsclam ../sync/sync.txt poses_opt ../../../src/hypergraphsclam/config/log_vale_5_optimization_config.txt 

10. Limpe o diretório de mapas temporários (../data/mapper_teste2) usando o CleanMap. Ele roda muito rápido. Assim, basta escolher Start Program e depois Stop Program.

11. Para construir seu mapa no diretório ../data/mapper_teste2, rode o PubPoses e reinicie seu log no início do trecho de intesse. Rode o log por todo o trecho de 
   interesse em velocidade (Speed do playback control) compatível com o hardware onde você estiver trabalhando.
   Pronto. Seu mapa estará disponível em ../data/mapper_teste2. Mate o proccontrol e examine seu mapa com um proccontrol de playback_viewer_3D.ini apropriado.

12. Se o mapa estiver em condições de produção, mova para o data usando o padrão de nome de pasta de mapas.
        cp $CARMEN_HOME/data/mapper_teste2  $CARMEN_HOME/data/map_<nome(sem o nome log)do log usado>

13. Copie os arquivos da pasta tmp para a pasta $CARMEN_HOME/data/graphslam/ assim as poses originais do mapa serão preservadas

        cp $CARMEN_HOME/bin/tmp  $CARMEN_HOME/data/graphslam/poses_opt_<log_usado>


##Com fazer merge de poses para merge de mapas.

O hypergraphsclam tem a facilidade de otimizar vários logs ao mesmo tempo, inclusive de robôs diferentes.
Siga os passo normais, porém no arquivo .list para o parser (item 6.) coloque em cada linha o log e arquivos adequados.

Lembre-se de na hora de fazer o mapa (itens 11 abaixo) ativar no carmen.ini usado

mapper_mapping_mode_on_use_merge_between_maps 			on

 1.1 O parametro mapper_mapping_mode_on_use_merge_between_maps deve estar on, ele é o responsável por fazer o merge de
     forma inteligente sem apagar informações importantes de um log para outro. 
     Ela deve ser off no caso que se deseja sobreescrever as informações do mapa anterior.
 1.2 O parametro mapper_rays_threshold_to_merge_between_maps deve ser ajustado para melhor merge dos mapas. 
     Ele define o numero de raios que bateram em uma célula para saber se ela deve ser atualizada, é adhoc.


