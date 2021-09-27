Para que o mapa de remission fique bom e seu uso seja o melhor possível é importante calibrar e usar a remission do Velodyne.

Para calibrar, rode a etapa de mapeamento uma vez (a etapa em que as poses otimizadas do graphslam ou hypergraphsclam são usadas) com o mapper configurado como abaixo:
 ./mapper -map_path ../data/mapper_teste2/ -mapping_mode on  -save_calibration_file calibration_file.txt

O arquivo calibration_file.txt (você pode ecolher outro nome, obviamente) será gerado e conterá os dados necessários para a calibração. Este arquivo fica 
muito grande se o log for longo. Não é necessário que o log seja longo para uma boa calibração da remission do Velodyne, mas é importante que ele contenha 
situações representativas das remissions de interesse (que ocorrem no mundo real na região de operação).

De posse do arquivo calibration_file.txt, rode o comando abaixo:
 ./compute_velodyne_calibration_table calibration_file.txt calibration_table_gpx_ford_fusion.txt

O programa acima demora para rodar e, quanto maior o log, mais tempo. Quando ele termina de rodar, o arquivo calibration_table_gpx_ford_fusion.txt é gerado
(você pode escolher outro nome, obviamente). Este é o arquivo com a calibração do Velodyne. Para gerar um mapa utilizando a calibração, rode o
processo de mapeamento de novo (a etapa em que as poses otimizadas do graphslam ou hypergraphsclam são usadas; não se esqueça de apagar o conteúdo 
de ../data/mapper_teste2/) com o mapper configurado como abaixo:
 ./mapper -map_path ../data/mapper_teste2/ -mapping_mode on  -calibration_file calibration_table_gpx_ford_fusion.txt

MUITO IMPORTANTE: Uma vez gerado o mapa de remission com o mapper configurado com o arquivo de calibração, você deve utiliza-lo para localização com o 
localizer_ackerman configurado abaixo:
 ./localize_ackerman -mapping_mode off  -calibration_file calibration_table_gpx_ford_fusion.txt
