Para construir uma matriz NHCM, é necessário executar o cost_matrix com o central e um param_deamon rodando com o astro ini do veículo 
(pois o programa usa alguns métodos do astro para obter os parâmetros do veículo). Ele deve ser executado do diretório bin, isto é:
 ../src/offroad_planner/NHCM/cost_matrix

O executável cost_matrix_trailer gera uma matriz NHCM para um veículo do tipo trailer (existe duas dimensões a mais, os dois beta). 
É necessário executar o cost_matrix_trailer com o central e um param_deamon rodando (pois o programa usa alguns métodos do astro 
para obter os parâmetros do veículo).

Ele deve ser executado do bin, isto é:
 ../src/offroad_planner/NHCM/cost_matrix_trailer

Para ver o gráfico de cada path gerado, compile com #define PLOT_PATH descomentado.

Para ver um gráfico de todos os custos por cada uma das coordenadas de interesse no caso de ./cost_matrix use:
 gnuplot> splot "cost_matrix_02_101x101x72.data" u 1:2:3:4 w p pt 7 ps 2 palette notitle

Para ver um gráfico de todos os custos por cada uma das coordenadas de interesse no caso de ./cost_matrix_trailer use:
 gnuplot> splot "../data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data" u 1:2:3:( ($6 > -1.0) && ((($7 > -1.0) && ($6 < $7)) || ($7 == -1.0))? $6: $7 ) w p pt 7 ps 2 palette notitle


ou 
 grep -v "1000000.000000" ../data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data > caco.txt
 gnuplot> splot "caco.txt" u 1:2:3:( (($7 != -1.0) && ($7 < 10000.0))? (int(2*$7)<<16+(int(0x00)<<24)): ( (($6 != -1) && ($6 < 10000.0))? (int(2*$6)<<8+(int(0x00)<<24)): (0x00AAFF+(int(0xFF)<<24)) ) ) pt 7 lc rgb variable ps 1

ou
 grep -v "1000000.000000" ../data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data > caco.txt
 gnuplot> splot "caco.txt" u 1:2:3:( (($7 != -1.0) && ($4 == 0) && ($5 == 0))? (int(2*$7)<<16+(int(0x00)<<24)): ( (($6 != -1) && ($4 == 0) && ($5 == 0))? (int(2*$6)<<8+(int(0x00)<<24)): (0x00AAFF+(int(0xFF)<<24)) ) ) pt 7 lc rgb variable ps 1

####################################

Para rodar na monster

ssh lcad@200.137.66.172
senha k40N...

ssh lcad@10.9.8.251 {N...}

 sudo UFES
 ctrl+z & bg

 cd astro/bin
 ../src/offroad_planner/NHCM/cost_matrix_trailer
 
 #Se estiver em uma sessão ssh e deseja manter o processo rodando mesmo ao desligar a conexão ssh:
 stdbuf -i0 -o0 -e0 nohup ../src/offroad_planner/NHCM/cost_matrix_trailer2 1 256 &

