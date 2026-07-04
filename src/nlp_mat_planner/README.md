Para usar o offroad, execute: ./offroad_planner
Existem argumentos para modificar o comportamento do uso do cache de path.
Para escolher o arquivo de cache, execute:
  ./offroad_planner -cache_filename arquivo_de_cache.txt
Para impedir que novos caminhos sejam inseridos no cache, execute:
  ./offroad_planner -cache_writing off

Executar o offroad_planner sem argumentos irá estar com o cache_writing habilitado, e o nome do arquivo de cache padrão (offroad_planner_cache.txt)

======== Passos necessários para utilizar o offroad park truck & trailer:

```
pip3 install casadi==3.5.5
```

- Add the following lines to your bash profile

```
#offroad_planner
export CASADI_DIR=$(/usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
export LD_LIBRARY_PATH=$CASADI_DIR:$LD_LIBRARY_PATH
```

Configure o número de semi-trailers definindo o parâmetro do astro.ini semi_trailer_initial_type

========================================================================


Para baixar as tabelas (matriz NHCM - Non-Holonomic Cost Matrix) do offroad_planner Execute:
make download

Para atualizar as tabelas (matriz NHCM - Non-Holonomic Cost Matrix) do offroad_planner Execute:
make update

As tabelas serão baixadas e colocadas na pasta astro/data. Eles ficam na raiz deste diretório:
https://drive.google.com/drive/folders/1jAnfylZNC23Ag2_DhNPCxt9ReOcrmIVL?usp=share_link

Para saber como gerar as tabelas, examine o README.txt do subdiretorio NHCM.

======== Passos para utilizar as tabelas não holonomicas:

Primeiramente verifique qual tabela deve ser utilizada. No Makefile existem opções para download de tabelas específicas, no momento existem:
- Tabela para a IARA	 offroad_planner_car_cost_matrix_101x36x1.0.data
- Tabela para a IARA utilizando a carreta	offroad_planner_trailer_cost_matrix_81x10x5x1.0.data
- Tabela para para tractor e trailer (incerto quanto qual tractor é referente)	offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data
- Tabela para o tractor da axor	offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano
- Tabela para o tractor da mosaic	offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic

Para baixar as tabelas, execute 
make download

Para garantir que as tabelas estejam atualizadas execute:
make update
- Essa opção vai excluir todas as tabelas do computador e baixá-las de novo

Para baixar somente a tabela da axor ou da mosaic, execute:
make axor_suzano
make atego_mosaic

Fique atento para mensagens de erro. Às vezes o script para baixar do drive falha e os arquivos da tabela são criados mas ficam com 0 bytes. Caso isso aconteça, é necessário excluir esses arquivos vazios e baixá-los novamente.

Caso o offroad morra com aviso a respeito de parâmetros da tabela, verifique se o parâmetro
	offroad_planner_precomputed_cost_trailer_file_name
do .ini utilizado está com o nome correto.

Se o parâmetro estiver correto e mesmo assim acontecer erro, é possível que os parâmetros de tamanho da tabela estão incorretos. Para descobrir o tamanho correto dos parâmetros, execute:
	tail <nome_da_tabela_baixada>
	
O retorno será algo do tipo:
	80 80 8 3 0 -2.000000 -2.000000 0 0
	80 80 8 3 1 -1.000000 -1.000000 0 0
	80 80 8 3 2 -2.000000 -2.000000 0 0
	80 80 8 3 3 -2.000000 -2.000000 0 0
	80 80 8 3 4 -2.000000 -2.000000 0 0
	80 80 8 4 0 -2.000000 -2.000000 0 0
	80 80 8 4 1 -2.000000 -2.000000 0 0
	80 80 8 4 2 -2.000000 -2.000000 0 0
	80 80 8 4 3 -2.000000 -2.000000 0 0
	80 80 8 4 4 -2.000000 -2.000000 0 0
	
Esses valores são a posição do index de um vetor, então o size verdadeiro é o valor + 1
As duas primeiras colunas se referem ao tamanho da tabela (offroad_planner_precomputed_cost_trailer_size),
 a terceira é o tamanho do theta (discretizado) (offroad_planner_precomputed_cost_trailer_theta_size),
 e a quarta e a quinta são referente ao tamanho do beta (discretizado) (offroad_planner_precomputed_cost_trailer_beta_size)
 Até o momento, o parâmetro offroad_planner_precomputed_cost_trailer_resolution é sempre 1.0
Nessa tabela acima, os valores correto dos parâmetros são:

offroad_planner_precomputed_cost_trailer_size		81.0 # em metros
offroad_planner_precomputed_cost_trailer_theta_size	9 # Tem que ser impar
offroad_planner_precomputed_cost_trailer_beta_size	5 # Tem que ser impar 
offroad_planner_precomputed_cost_trailer_resolution	1.0
offroad_planner_precomputed_cost_trailer_file_name	../data/<nome_da_tabela_baixada>
====================================================================

## Relevant Parameters for Planned Path Quality

| Parameter                                                              | Function                                                                                                                                                                                                                                                                                                                                                                     |
| ---------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| offroad_planner_penalties_w1                                           | Penalty cost for expanded nodes in the reverse direction.                                                                                                                                                                                                                                                                                                                    |
| offroad_planner_penalties_w2                                           | Penalty cost for expanded nodes that change the direction (from forward to backwards, and vice versa).                                                                                                                                                                                                                                                                       |
| offroad_planner_penalties_w3                                           | Penalty cost for expanded nodes based on its distance to obstacles. The penalty is multiplied by the inverse of the distance.                                                                                                                                                                                                                                                |
| offroad_planner_penalties_w4                                           | Not used.                                                                                                                                                                                                                                                                                                                                                                    |
| offroad_planner_penalties_w5                                           | Not used.                                                                                                                                                                                                                                                                                                                                                                    |
| offroad_planner_min_dist_motion_change                                 | Minimum length of the path from last change of direction (or from the initial pose) to allow the node to expand into nodes that change the current direction. If the value is `5.0`, the path must have at least `5.0` meters to change its direction (forward to backwards, or vice versa). If the value is `0.0` or negative, this will be disabled.                       |
| offroad_planner_max_reed_shepp_distance {optional parameter}           | Maximum length of the path computed by the analytic expansion reed_shepp. Negative value or if the parameter is not found, this limitation is disabled.                                                                                                                                                                                                                      |
| offroad_planner_max_reed_shepp_distance_backwards {optional parameter} | Maximum length of the backward segment of the path computed by the analytic expansion reed_shepp. Negative value or if the parameter is not found, this limitation is disabled.                                                                                                                                                                                              |
| offroad_planner_max_change_direction {optional parameter}              | Maximum change of directions of the path computed by the A-star search. New nodes that  disobey the restrictions are removed. Negative value or if the parameter is not found, this limitation is disabled.                                                                                                                                                                  |
| offroad_planner_max_reed_shepp_change_direction {optional parameter}   | Maximum change of directions computed by the analytic expansion reed_shepp. Negative value or if the parameter is not found, this limitation is disabled.                                                                                                                                                                                                                    |
| offroad_planner_use_reed_shepp {optional parameter}                    | An integer where 1 means that reed_shepp will be used, and 0 means that it will not. If the parameter is not found the default choice will be 1.                                                                                                                                                                                                                             |
| offroad_planner_max_phi_multiplier                                     | A double that is multiplied by robot_max_phi. This is used to guarantee that the curvature of the path found is not at the limit of the vehicle capacity. It needs to be a value smaller or equal to 1.0.                                                                                                                                                                    |
| offroad_planner_goal_achieved_distance                                 | A double value that is used to classify the precision of the distance to the final goal where the planner can assume the final goal was achieved.                                                                                                                                                                                                                            |
| offroad_planner_goal_achieved_theta                                    | A double value that is used to classify the precision of the orientation of the current pose to the final goal where the planner can assume the final goal was achieved.                                                                                                                                                                                                     |
| offroad_planner_radius_circle_to_ignore_obstacles_from_final_goal      | A double value that represents the radius from the final goal to make the offroad ignore collisions from poses inside that area. It is used when it is expected to find collision close to the final goal but we want the planner to return a path.                                                                                                                          |
| offroad_planner_state_map_resolution                                   | A double value representing the size of x y cell of the grid state map used by the hybrid a-star. This value is also associated with the v_step taken by the expansion of nodes. The smaller the value, more memory is allocated. If the value is too small, the expansion of nodes can't make a state leave its grid cell because of limitations of the ackermann movement. |
| offroad_planner_state_map_theta_resolution                             | A double value representing the size of theta resolution of the grid state map used by the hybrid a-star. The bigger it is, more memory is allocated.                                                                                                                                                                                                                        |

