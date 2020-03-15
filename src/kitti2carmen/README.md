# Como gerar um log do carmen a partir da base kitti:

O módulo kitti2carmen é executado usando a seguinte linha de comando

	$ ./kitti2carmen_main <pasta data do velodyne> <timestamps.txt do velodyne> <pasta data camera esq> <pasta data camera dir> <timestamps.txt da camera> <pasta data do gps> <timestamps.txt do gps>

Exemplo:

	$ ./kitti2carmen_main ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/velodyne_points/data/ ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/velodyne_points/timestamps.txt ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/image_02/data/ ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/image_03/data/ ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/image_03/timestamps.txt ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/oxts/data/ ~/Downloads/2011_09_26/2011_09_26_drive_0027_sync/oxts/timestamps.txt
	
A saída será um arquivo log.txt na pasta kitti2carmen, porém, os timestamps estarão desordenados.
Para ordenar o log os timestamps é necessário rodar o comando abaixo:

	awk '{print $NF,$0}' log.txt | sort | cut -f2- -d' ' > log_ordenado.txt
	
O arquivo log_ordenado.txt pode ser usado no playback do carmen.
Atualmente a mensagem utilizada é a partial scan, se caso o código da variable_scan for reativado, o numero do velodyne deverá ser adicionado no kitti2carmen_main.cpp no na variável kitty_velodyne_number, atualmente = 9
O carmen-ford-escape.ini deve ter os parâmetros do carro da KITTI como mostrado no paper:
http://www.cvlibs.net/publications/Geiger2013IJRR.pdf

