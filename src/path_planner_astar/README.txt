Para utilizar o path_planner_astar basta executar o central, o process-navigate-volta-da-ufes-pid.ini, compilar (usando o make) e executar o path_planner_astar.

Então seleciona o robot position no navigator_gui e então o final goal. Após isso aguarde o algoritmo rodar e enviar as poses para o sistema.

./central
./proccontrol process-anderson-astar.ini 
./path_planner_astar

Para utilizar o algoritmo é necessário ter a matriz para a heurística sem obstáculos. Para gerar a matriz, é necessário executar o código cost_matrix (a execução demora, então é melhor baixar a matriz em algum lugar).
Caso não deseje utilizar a matriz, pode-se alterar o parâmetro path_planner_astar_use_matrix_cost_heuristic para off no carmen-ford-escape.ini. Isso faz com que o algoritmo utilize o reed shepp, que é um pouco mais lento mas é uma heurística aceitável para um teste.

Faça o download da matriz da heurística sem obstáculos nesse link:
https://drive.google.com/file/d/1BqqAVreRr5M87O2lU2iZQT77I8FhTOfE/view?usp=sharing


Uma explicação de cada parâmetro que o código utiliza

###############################
# Path_planner_astar parameters

path_planner_astar_state_map_resolution		1.0	#Resolução do mapa que o a-estrela utiliza para realizar a busca
path_planner_astar_state_map_theta_resolution	36 	#O tamanho do theta do mapa, isso significa que o mapa terá x dimensões em theta
path_planner_astar_precomputed_cost_size	100	#Tamanho do mapa da matriz de heurística sem obstáculos
path_planner_astar_precomputed_cost_theta_size	72	#O tamanho do theta da matriz de heurística sem obstáculos
path_planner_astar_precomputed_cost_resolution	0.2	#Resolução da matriz de heurística sem obstáculos
path_planner_astar_precomputed_cost_file_name	cost_matrix_02_101x101x72.data #Nome do arquivo da matriz de heurística sem obstáculos
path_planner_astar_use_matrix_cost_heuristic	on	#on para utilizar a matriz e off para utirlizar reed-shepp no lugar
path_planner_astar_max_steering_angle		0.4	#Apenas para que o reed-shepp não utilize outro ângulo máximo


###############################
