Como criar um mapa usando GraphSLAM:

1. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation.ini (ou outro equivalente ajustado para o seu caso). 
   Lembre-se de colocar nele o seu log, o rddf desejado, e carmen ini em todos os lugares que precisa.

2. Ligue e desligue, no PROCCONTROL GUI, o botão ClTmpDir (clique o botão ClTmpDir e ecolha Start Program, e depois Stop Program) para inicializar o diretório $CARMEN_HOME/bin/tmp.
 
3. Execute a calibração da odometria clicando, no PROCCONTROL GUI, no botão CalibOdo e escolhendo Show Output (para saber quando o programa terminou) e, em seguida, clicando no 
   botão CalibOdo e ecolhendo Start Program. 
	Notas:
		- Rode mais de uma vez e observe qual a melhor calibração no gráfico final.
		- Você pode delimitar o tempo inicial e final a ser considerado no log verificando o tempo no playback control e escolhendo apropriadamente (tempo, não timestamp). 
		  Use as flags -l <linha inicial>  e -m <numero de linhas a partir da linha inicial> do arquivo $CARMEN_HOME/bin/calibrate_bias_from_log_config.txt
		- Existem outras opções importantes no $CARMEN_HOME/bin/calibrate_bias_from_log_config.txt

3.1. Altere as variaveis abaixo no carmen-ford-escape.ini de acordo com os Resultados do calibrate_bias_from_log (tmp/calibrated_odometry.txt):
	robot_phi_multiplier				1.056087
	robot_phi_bias					1.065384
	robot_v_multiplier				0.003477

	Notas:
		- Voce pode usar o tmp/odom_poses_opt.txt para fazer mapas sem graphslam! Basta usa-lo como se fosse a saida do graphslam para o 
		  log. Para isso, no process de fazer mapas, basta usar a linha do graphslam_publish como abaixo:
 Publish_poses		graphslam	1		0			./graphslam_publish tmp/odom_poses_opt.txt

4. Gere o arquivo tmp/sync.txt clicando, no PROCCONTROL GUI, no botão GrabData e escolhendo Show Output (para saber quando o programa terminou) e, em seguida, clicando no 
   botão GrabData e ecolhendo Start Program. 
   Esta ação vai rodar o comando abaixo:
     ./grab_data_from_log /dados/log_gpx_20190510_ford_fusion-6.txt tmp/calibrated_odometry.txt tmp/sync.txt
   Quando o programa terminar, clique no botão Step_1 e escolha Stop Program.
   Você pode examinar o arquivo tmp/sync.txt com o gnuplot (ver campos de cada linha na função build_optimization_graph() de graphslam.cpp):
     gnuplot
     gnuplot> set size square; set size ratio -1; plot "tmp/sync.txt" u 4:5 w l t 'gps xyz'
   Saia do gnuplot e entre novamente:
     gnuplot> quit
     gnuplot
     gnuplot> set size square; set size ratio -1; plot "tmp/sync.txt" u 1:2 w l t 'odometry'

5. Para otimizar PRELIMINARMENTE suas poses com o graphslam, clique no botão GSlam escolha Show Output (para saber quando o programa terminou) e, em seguida, 
   clique no botão GSlam e escolha Start Program. Quando o programa terminar, clique no botão GSlam e escolha Stop Program.

   Este passo produz uma versão preliminar do arquivo tmp/poses_opt.txt, que você pode examinar com o gnuplot (ver campos de cada linha na função save_corrected_vertices() de graphslam.cpp):
     gnuplot> set size square; set size ratio -1; plot "tmp/sync.txt" u 4:5 w l t 'gps xyz', "tmp/poses_opt.txt" u 1:2 w l t 'car'

   Se seu log não tiver loops ou você não quiser tratar loops, pule para o passo 8.

6. Se você desejar fechamento de loops, gere o arquivo tmp/loops.txt da seguinte forma:

6.1. Delimite o trecho a ser considerado para fechamento de loops usando o process-volta_da_ufes_playback_viewer_3D.ini (ou equivalente).
     Para isso, identifique no playback control o momento inicial (segundo_inicial_1) e final (segundo_final_1) da primeira passada em uma região, e o momento inicial (segundo_inicial_2) 
     e final (segundo_final_2) da segunda passada na mesma região.

6.2. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation.ini novamente (ou outro equivalente ajustado para o seu caso) e:

6.2.1. Limpe o diretório de mapas temporários (../data/mapper_teste2) clicando no botão CleanMap. Ele roda muito rápido. Assim, basta escolher Start Program e depois Stop Program.

6.2.2. Faça o mapa da segunda passada na região de loop ligando o PubPoses, escolhendo, no playback control, "Message play:stop" t segundo_inicial_2:segundo_final_2, e teclando play. 
       Quando terminar de rodar, o arquivo tmp/gp1.txt será gerado pelo localizer e conterá a globalpos da segunda passada na região de loop.

6.2.3. Gere poses (globalpos) da primeira passada pela região de loop com o localizer matando o processo anterior e rodando o process-volta_da_ufes_playback_viewer_3D.ini.  
       Escolha no playback control "Message play:stop" t XXXX, onde XXXX é um momento (segundo) antes de segundo_inicial_1, e tecle play. 
       Ajuste a pose do robô no mapa (use o play e o stop) a garanta uma boa localização antes do segundo_inicial_1. Depois, deixe rodar o log observando
       a pose do robô no mapa - ele deve percorrer uma região que garanta boa localização no mapa da região de loop gerado no passo 6.2.2.
       Quando terminar de rodar, o arquivo tmp/gp2.txt terá sido gerado pelo localizer e conterá a globalpos da primeira passada na região de loop.

6.2.4. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation.ini (ou outro equivalente ajustado para o seu caso) e
       clique no botão LoopC escolha Show Output (para saber quando o programa terminou) e, em seguida, clique no botão LoopC e escolha Start Program para 
       gerar o arquivo tmp/loops.txt (ver campos de cada linha na função build_optimization_graph() de graphslam.cpp).

7. Clique no botão GSlam escolha Show Output (para saber quando o programa terminou) e, em seguida, clique no botão GSlam e escolha Start Program para otimizar suas poses 
    com o graphslam. Quando o programa terminar, clique no botão GSlam e escolha Stop Program.

    Este passo produz a versão final do arquivo tmp/poses_opt.txt, que você pode examinar com o gnuplot (ver campos de cada linha na função save_corrected_vertices() de graphslam.cpp):
     gnuplot> set size square; set size ratio -1; plot "tmp/sync.txt" u 4:5 w l t 'gps xyz', "tmp/poses_opt.txt" u 1:2 w l t 'car'

8. Limpe o diretório de mapas temporários (../data/mapper_teste2) usando o CleanMap. Ele roda muito rápido. Assim, basta escolher Start Program e depois Stop Program.

9. Para construir seu mapa no diretório ../data/mapper_teste2, rode o PubPoses e reinicie seu log no início do trecho de intesse. Rode o log por todo o trecho de 
   interesse em velocidade (Speed do playback control) compatível com o hardware onde você estiver trabalhando.
   Pronto. Seu mapa estará disponível em ../data/mapper_teste2. Mate o proccontrol e examine seu mapa com um proccontrol de playback_viewer_3D.ini apropriado.


==============================================================================================================================================================================

Como criar um mapa usando GraphSLAM [usando o process]:

1. Execute a calibração da odometria como descrito em $CARMEN_HOME/src/odometry_calibration/README.txt
2. Crie o diretorio $CARMEN_HOME/bin/tmp
3. Rode o process-volta_da_ufes_playback_viewer_3D_map_generation.ini ou outro equivalente. Lembre-se de colocar nele o seu log e o rddf desejado.

3.1 Você pode delimitar o trecho de interesse do log escolhendo no playback control o tempo inicial e final. Exemplo t 10:100 (tempo inicial 10s tempo final 100s).
    Para saber o tempo final de seu log coloque t 100000000 no playback control (onde 100000000 é um número grande) que ele dirá qual o tempo final.

3.2 Ligue no PROCCONTROL GUI o programa Step_0 (clique o botão Step_0 e ecolha Start Program) para limpar o diretório $CARMEN_HOME/bin/tmp.

3.3 Ligue no PROCCONTROL GUI o programa Step_1 (clique o botão Step_1 e ecolha Start Program) e rode o seu log completamente ou no trecho de tempo de interesse em velocidade 
    (Speed do playback control) compatível com o hardware onde você estiver trabalhando. 
    Quando terminar de rodar o log, termine o Step_1 escolhendo Stop Program no botão Step_1.

    Este passo gera o arquivo tmp/sync.txt, que você pode examinar com o gnuplot (ver campos de cada linha na função build_optimization_graph() de graphslam.cpp):
     gnuplot> set size square; set size ratio -1; plot "tmp/sync.txt" u 4:5 w l t 'gps xyz'

3.4 Se você desejar tratar fechamento de loops, clique no botão Step_2 escolha Show Output (para saber quando o programa terminou) e, em seguida, 
    clique no botão Step_2 e escolha Start Program. Quando o programa terminar, clique no botão Step_2 e escolha Stop Program.
    Para delimitar o trecho a ser considerado para fechamento de loops, use os parâmetros do programa que é rodado no Step_2:
    ./run_icp_for_loop_closure <input-file> <velodyne-dir> <output_file> <dist_for_detecting_loop_closure (meters)> <time_difference_for_detecting_loop_closure (seconds)>
    Para não considerar loops, use uma diferença de tempo maior que o tamanho total do log.

    Este passo produz o arquivo tmp/loops.txt (ver campos de cada linha na função build_optimization_graph() de graphslam.cpp).

3.5 Clique no botão Step_3 escolha Show Output (para saber quando o programa terminou) e, em seguida, clique no botão Step_3 e escolha Start Program para otimizar suas poses 
    com o graphslam. Quando o programa terminar, clique no botão Step_3 e escolha Stop Program.

    Este passo produz o arquivo tmp/poses_opt.txt, que você pode examinar com o gnuplot (ver campos de cada linha na função save_corrected_vertices() de graphslam.cpp):
     set size square; set size ratio -1; plot "tmp/sync.txt" u 4:5 w l t 'gps xyz', "tmp/poses_opt.txt" u 1:2 w l t 'car'

3.6 Limpe o diretório de mapas temporários (../data/mapper_teste2) usando o CleanMap. Ele roda muito rápido. Assim, basta escolher Start Program e depois Stop Program.

3.7 Para construir seu mapa no diretório ../data/mapper_teste2, rode o Step_4 e reinicie seu log no início do trecho de intesse. Rode o log por todo o trecho de 
    interesse em velocidade (Speed do playback control) compatível com o hardware onde você estiver trabalhando.
    Pronto. Seu mapa estará disponível em ../data/mapper_teste2. Mate o proccontrol e examine seu mapa com um proccontrol de playback_viewer_3D.ini apropriado.


==============================================================================================================================================================================

Documentação antiga.


Como criar um mapa usando GraphSLAM [usando o process]:

#  Antes de executar os passos abaixo, leia o ../src/odometry_calibration/README.txt e execute os passos de lá

#  Nos casos em que o navigator_gui2 eh usado abaixo, certifique-se de que o mapa corrente no menu Maps da interface do navigator_gui2 eh o Map.

1. Compile o modulo graphslam (requer a biblioteca pcl e o framework g2o)
2. Crie um ramdisk no diretorio tmp2:
	> sudo mount -t ramfs -o nosuid,noexec,nodev,mode=0777,size=4096M ramdisk $CARMEN_HOME/bin/tmp2

	***** NOTA: O ramdisk ira criar um diretorio na memoria com o tamanho especificado na variavel size da linha de comando. Fique atento
		para que o espaco do ramdisk nao acabe. Voce pode verificar o espaco ocupado usando "du -sh tmp2". Se o tamanho estiver chegando
		no limite, mova os arquivos .ply para o diretorio tmp usando "mv tmp2/*.ply tmp".
	***** Crie os diretorios $CARMEN_HOME/data/mapper_teste e $CARMEN_HOME/bin/tmp

3. Execute o central
4. Modifique o process-volta_da_ufes_playback_viewer_3D_map_generation.ini para que ele faca playback de seu log:
		./playback [seu log]

5. Modifique o carmen-ford-escape.ini para ativar a criacao de mapas:

mapper_update_and_merge_with_snapshot_map	off
mapper_global_map 				off
mapper_merge_with_offline_map 			off
mapper_update_and_merge_with_mapper_saved_maps	on
mapper_build_snapshot_map			off
mapper_velodyne_range_max		 	50.0
mapper_velodyne_range_max_factor 		4.0

6. Execute o programa "Step_1" na interface.
7. Faca playback do seu log ate o final a uma velocidade menor (0.5 ou menos), mas, a cada 2 minutos, (i) pare (stop), (ii) execute "mv tmp2/*.ply tmp" (ver a NOTA em 2., acima), e (iii) play.
8. Mate o programa "Step_1" na interface.
8. Mova os arquivos que estiverem na pasta tmp2 para a pasta tmp usando "mv tmp2/* tmp"
10. Clique em Show_output e Execute o programa "Step_2" na interface. Assim que aparecer a mensagem indicando que o programa terminou, desligue o programa.
11. Clique em Show_output e Execute o programa "Step_3" na interface. Assim que aparecer a mensagem indicando que o programa terminou, desligue o programa.

	***** NOTA: Você pode usar o gnuplot para visualizar o resultado. Use os comandos abaixo dentro do gnuplot.
		gnuplot> plot 'tmp/sync.txt' using 4:5 with lines
		gnuplot> replot 'tmp/poses_opt.txt' using 1:2 with lines

12. Mate o procccontrol e apague os arquivos no ../data/mapper_teste2
	rm -rf ../data/mapper_teste2/*
		
13. Reinicie o proccontrol e desligue o programa "fused_odometry".
14. Execute o programa "Step_4" na interface.
15. Faca o playback com velocidade 0.5 de seu log novamente para criar o mapa.
16. Seu mapa está pronto em ../data/mapper_teste2/ !!!! Pode matar o proccontrol e copiar seu novo mapa para seu lugar definitivo.

Para limpar o mapa use os programas
	bin/build_complete_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -map_type m
	bin/map_editor <path do diretorio do mapa>/complete_map.map
	bin/complete_map_to_block_map -map_path <path do diretorio do mapa> -map_resolution 0.2 -block_size_in_meters 210.0
	Exclua o complete_map.map após finalizar antes de subir para o git
17.O process já constroe o rndf(rddf) usando o ./rddf_build ../data/rndf/rndf.kml
	Após criar o mapa, renomear o arquivo ../data/rndf/rndf.kml para ../data/rndf/rddf-log_voltadaufes-<data do log>.kml
		mv ../data/rndf/rndf.kml ../data/rndf/rddf-log_voltadaufes-<data do log>.kml

--------------------------------------------------------------------------------------------------------------------------------------

Como criar um mapa usando GraphSLAM [sem usar o process]:

1. Compile o modulo graphslam (requer a biblioteca pcl e o framework g2o)
2. Crie um ramdisk no diretorio tmp2:
	> sudo mount -t ramfs -o nosuid,noexec,nodev,mode=0777,size=4096M ramdisk $CARMEN_HOME/bin/tmp2

	***** NOTA: O ramdisk ira criar um diretorio na memoria com o tamanho especificado na variavel size da linha de comando. Fique atento
		para que o espaco do ramdisk nao acabe. Voce pode verificar o espaco ocupado usando "du -sh tmp2". Se o tamanho estiver chegando
		no limite, mova os arquivos .ply para o diretorio tmp usando "mv tmp2/*.ply tmp".

3. Execute o central
4. Execute o programa "./grab_data tmp2 tmp2/sync_data.txt"
5. Faca playback do seu log ate o final
6. Mate o programa "./grab_data tmp2 tmp2/sync_data.txt"
7. Mova os arquivos que estiverem na pasta tmp2 para a pasta tmp usando "mv tmp2/* tmp"
8. Execute o programa "./run_icp_for_loop_closure tmp/sync_data.txt tmp tmp/loops.txt"
9. Execute o programa "./graphslam tmp/sync_data.txt tmp/loops.txt tmp/poses_opt.txt"
10. Execute o programa "./graphslam_publish tmp/poses_opt.txt" 
11. Faca o playback de seu log novamente para criar o mapa
	
	***** NOTA: Lembre-se de mudar os parametros no carmen-ford-escape.ini para o modo de criacao de mapas. Os valores devem ser:
		mapper_update_and_merge_with_snapshot_map	off
		mapper_global_map 				off
		mapper_merge_with_offline_map 			off
		mapper_update_and_merge_with_mapper_saved_maps	on
		mapper_build_snapshot_map			off


--------------------------------------------------------------------------------------------------------------------------------------


