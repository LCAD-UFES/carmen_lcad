Como criar um mapa usando GraphSLAM [usando o process]:

1. Execute a calibração da odometria como descrito em $CARMEN_HOME/src/odometry_calibration/README.txt
2. Crie o diretorio $CARMEN_HOME/bin/tmp
3. 
















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


