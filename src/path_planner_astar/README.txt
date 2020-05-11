Para utilizar o path_planner_astar basta executar o central, o process-navigate-volta-da-ufes-pid.ini, compilar (usando o make) e executar o path_planner_astar.

Então seleciona o robot position no navigator_gui e então o final goal. Após isso aguarde o algoritmo rodar e enviar as poses para o sistema.
Se não estiver a matriz de heurística sem obstáculos, basta colocar o USE_MATRIX_HEURISTIC com valor 0 para rodar o reed shepp como substituto da heurística.

./central
./proccontrol process-navigate-volta-da-ufes-pid.ini 
./path_planner_astar
