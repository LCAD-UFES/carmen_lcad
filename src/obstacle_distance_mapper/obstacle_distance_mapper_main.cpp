#include <carmen/carmen.h>
#include "obstacle_distance_mapper_interface.h"
#include <carmen/mapper_interface.h>
#include <carmen/grid_mapping.h>

#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <iostream>
#include <string.h>

#include <carmen/behavior_selector_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/route_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include "obstacle_distance_mapper_datmo.h"


double obstacle_probability_threshold 	= 0.5;
double obstacle_cost_distance 			= 1.0;
double min_moving_object_velocity 		= 0.3;
double max_moving_object_velocity 		= 150.0 / 3.6; // 150 km/h
double moving_object_merge_distance		= 1.0; // m
int behavior_selector_use_symotha 		= 0;

carmen_map_t 										occupancy_map;
carmen_mapper_compact_map_message 					compact_occupancy_map;
carmen_prob_models_distance_map 					distance_map;

carmen_obstacle_distance_mapper_compact_map_message compact_distance_map;
carmen_obstacle_distance_mapper_compact_map_message compact_lane_contents;
carmen_map_t 										cost_map;
carmen_compact_map_t 								compacted_cost_map;
carmen_map_server_compact_lane_map_message			*compact_lane_map = NULL;
carmen_obstacle_distance_mapper_compact_map_message *behaviour_selector_compact_lane_contents_message = NULL;
carmen_route_planner_road_network_message 			*road_network_message = NULL;

carmen_point_t g_goal_position;
carmen_point_t g_robot_position;
carmen_point_t goal_list_message;

using namespace std;

//
// O obstacle_distance_mapper (ODM) recebe (a) um mapa de ocupação compactado do mapper (apenas as células com probabilidade
// de ocupação maior que mapper_min_occupied_prob (algo próximo de 0.5) e (b) o road map do route_planner. A partir destes dados,
// o ODM computa: (i) o mapa de distâncias para células ocupadas com objetos não móveis (ou com movimento lento - SODM); e
// (ii) os objetos móveis no road map (MO). Estes dois itens são enviados via mensagem existentes para os demais módulos.
// Alternativamente, estes dois itens podem ser agrupados em uma mensagem, por exemplo, carmen_obstacle_distance_mapper_output_message,
// e publicados de forma síncrona. Os objetos móveis devem ser codificados nesta mensagem de modo a poder ser acomodados em
// uma mensagem do tipo carmen_moving_objects_point_clouds_message.
//
// O behavior_selector (BS) recebe estes itens via mensagem(s) do ODM e outros dados e: (i) computa um conjunto de paths usando
// Frenet Frames; (ii) escolhe um goal que possa ser alcançado sem colisão em cada path; (iii) determina a velocidade em cada um
// destes goals; (iv) e escolhe o melhor conjunto path/goal/velocidade do goal.
//
// A escolha do melhor conjunto é feita pontuando: a distância para o goal (quanto maior melhor), a velocidade no goal (quanto
// maior melhor), e a distância para o path central que leva ao RDDF (quanto menor melhor).
//
// O itens (ii) e (iii) imediatamente acima são feitos em conjunto. Isto é, o BS pode, para cada path, fazer busca binária
// de qual velocidade pode ser atingida sem colisão, respeitando as acelerações máximas e o horizonte de tempo de planejamento.
// Ele tenta a velociade mais alta possível primeiro, V. Se houver colisão no horizonte de tempo de planejamento,
// ele tenta a 1/2 de V. Se na metada de V não houver colisão, ele tenta 3/4 de V, caso contrário, 1/4 de V e assim por diante até
// um nível mínimo de discretização de V em que consiga estabelecer um goal e uma velocidade.
//
// Na busca de um par velocidade / goal em um path, o BS examina cada ponto do path em busca de colisões com o SODM (colisão com objetos
// estáticos) e colisões com os MOs. Além disso, o BS checa anotações e se o goal vai ficar fora do mapa (neste
// caso, coloca o goal uma posição antes). A ideia é atribuir custos para cada path (e goal neste path e velocidade neste path) e
// escolher o de menor custo.
//


static void
build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_config_t config, carmen_prob_models_distance_map *distance_map,
		double distance_for_zero_cost)
{
	carmen_prob_models_initialize_cost_map(cost_map, config, config.resolution);

	double resolution = distance_map->config.resolution;
	for (int x = 0; x < distance_map->config.x_size; x++)
	{
		for (int y = 0; y < distance_map->config.y_size; y++)
		{
			double distance = distance_map->distance[x][y] * resolution;
			cost_map->map[x][y] = (distance > distance_for_zero_cost)? 0.0: 1.0 - (distance / distance_for_zero_cost);
		}
	}
}


carmen_obstacle_distance_mapper_compact_map_message
clear_lane_in_distance_map()
{
	carmen_obstacle_distance_mapper_compact_map_message lane_contents;

	if (compact_lane_map != NULL)
	{
		lane_contents.config = distance_map.config;
		// Aloca o minimo necessario, mas pode nao precisar de tudo, ja que os mapas podem ter origens diferentes e a lane vazar o distance_map
		lane_contents.coord_x = (short int *) malloc(compact_lane_map->size * sizeof(short int));
		lane_contents.coord_y = (short int *) malloc(compact_lane_map->size * sizeof(short int));
		lane_contents.x_offset = (char *) malloc(compact_lane_map->size * sizeof(char));
		lane_contents.y_offset = (char *) malloc(compact_lane_map->size * sizeof(char));
		int k = 0;
		for (int i = 0; i < compact_lane_map->size; i++)
		{
			cell_coords_t map_cell = carmen_obstacle_distance_mapper_get_map_cell_from_configs(distance_map.config, compact_lane_map->config,
					compact_lane_map->coord_x[i], compact_lane_map->coord_y[i]);
			if ((map_cell.x >= 0) && (map_cell.x < distance_map.config.x_size) && (map_cell.y >= 0) && (map_cell.y < distance_map.config.y_size))
			{
				lane_contents.coord_x[k] = map_cell.x;
				lane_contents.coord_y[k] = map_cell.y;
				lane_contents.x_offset[k] = distance_map.x_offset[map_cell.x][map_cell.y];
				lane_contents.y_offset[k] = distance_map.y_offset[map_cell.x][map_cell.y];
				distance_map.x_offset[map_cell.x][map_cell.y] = DISTANCE_MAP_HUGE_DISTANCE;
				distance_map.y_offset[map_cell.x][map_cell.y] = DISTANCE_MAP_HUGE_DISTANCE;
				distance_map.distance[map_cell.x][map_cell.y] = (double) DISTANCE_MAP_HUGE_DISTANCE * 1.414213562; // DISTANCE_MAP_HUGE_DISTANCE * raiz de 2.0
				k++;
			}
		}
		lane_contents.size = k;
	}
	else
	{
		lane_contents.config = distance_map.config;
		lane_contents.size = 0;
		lane_contents.coord_x = NULL;
		lane_contents.coord_y = NULL;
		lane_contents.x_offset = NULL;
		lane_contents.y_offset = NULL;
	}

	return (lane_contents);
}


void
free_compact_lane_contents()
{
	if (compact_lane_contents.coord_x != NULL)
		free(compact_lane_contents.coord_x);
	if (compact_lane_contents.coord_y != NULL)
		free(compact_lane_contents.coord_y);
	if (compact_lane_contents.x_offset != NULL)
		free(compact_lane_contents.x_offset);
	if (compact_lane_contents.y_offset != NULL)
		free(compact_lane_contents.y_offset);
}


void
compute_orm_and_irm_occupancy_maps(carmen_map_t *orm_occupancy_map, carmen_map_t *irm_occupancy_map, carmen_map_t *occupancy_map)
{
	if (!road_network_message)
		return;

	for (int i = 0; i < road_network_message->number_of_nearby_lanes; i++)
	{
		carmen_ackerman_traj_point_t *lane = &(road_network_message->nearby_lanes[road_network_message->nearby_lanes_indexes[i]]);
		int *traffic_restrictions = &(road_network_message->traffic_restrictions[road_network_message->nearby_lanes_indexes[i]]);
		int lane_size = road_network_message->nearby_lanes_sizes[i];
		for (int j = 0; j < lane_size - 1; j++)
		{
			double lane_width = ROUTE_PLANNER_GET_LANE_WIDTH(traffic_restrictions[j]);
			for (double s = 0.0; s < DIST2D(lane[j], lane[j + 1]); s += occupancy_map->config.resolution * 0.5)
			{
				double lane_x = lane[j].x + s * cos(lane[j].theta);
				double lane_y = lane[j].y + s * sin(lane[j].theta);
				for (double d = -lane_width / 2.0; d < lane_width / 2.0; d += occupancy_map->config.resolution * 0.5)
				{
					double x = lane_x + d * cos(lane[j].theta + M_PI / 2.0);
					double y = lane_y + d * sin(lane[j].theta + M_PI / 2.0);

					// Move global path point coordinates to map coordinates
					int x_map_cell = (int) round((x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
					int y_map_cell = (int) round((y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
					if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
						continue;

					irm_occupancy_map->map[x_map_cell][y_map_cell] = orm_occupancy_map->map[x_map_cell][y_map_cell];
					orm_occupancy_map->map[x_map_cell][y_map_cell] = 0.0;
				}
			}
		}
	}
}


void 
moving_objects_detection_tracking_publishing_and_occupancy_map_removal(carmen_map_t &occupancy_map, double timestamp)
{
	carmen_moving_objects_point_clouds_message *moving_objects = obstacle_distance_mapper_datmo(road_network_message, occupancy_map, timestamp);
	if (moving_objects)
	{
		carmen_moving_objects_point_clouds_publish_message(moving_objects);
		obstacle_distance_mapper_remove_moving_objects_from_occupancy_map(&occupancy_map, moving_objects);

		for (int i = 0; i < moving_objects->num_point_clouds; i++)
			free(moving_objects->point_clouds[i].points);
		free(moving_objects);
	}
}


void
build_occupancy_map(carmen_map_t &occupancy_map, carmen_mapper_map_message* msg)
{
	occupancy_map.config = msg->config;
	occupancy_map.complete_map = msg->complete_map;
	static double** occupancy_map_map = NULL;
	if (!occupancy_map_map)
		occupancy_map_map = (double **) malloc(occupancy_map.config.x_size * sizeof(double *));

	occupancy_map.map = occupancy_map_map;
	for (int i = 0; i < occupancy_map.config.x_size; i++)
		occupancy_map.map[i] = occupancy_map.complete_map + i * occupancy_map.config.y_size;
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
obstacle_distance_mapper_publish_compact_distance_map(double timestamp)
{
	carmen_obstacle_distance_mapper_create_compact_distance_map(&compact_distance_map, &distance_map, DISTANCE_MAP_HUGE_DISTANCE);
	carmen_obstacle_distance_mapper_publish_compact_distance_map_message(&compact_distance_map, timestamp);
}


static void
obstacle_distance_mapper_publish_compact_distance_and_compact_lane_contents_maps(double timestamp)
{
	compact_lane_contents = clear_lane_in_distance_map();
	carmen_obstacle_distance_mapper_create_compact_distance_map(&compact_distance_map, &distance_map, DISTANCE_MAP_HUGE_DISTANCE);
	carmen_obstacle_distance_mapper_publish_compact_distance_map_message(&compact_distance_map, timestamp);
	carmen_obstacle_distance_mapper_publish_compact_lane_contents_message(&compact_lane_contents, timestamp);
	free_compact_lane_contents();
}


void
obstacle_distance_mapper_publish_compact_cost_map(double timestamp)
{
	build_obstacle_cost_map(&cost_map, distance_map.config, &distance_map, obstacle_cost_distance);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);

	carmen_map_server_publish_compact_cost_map_message(&compacted_cost_map,	timestamp);
	carmen_prob_models_free_compact_map(&compacted_cost_map);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_mapper_compact_map_message_handler(carmen_mapper_compact_map_message *message)
{
	static carmen_compact_map_t *compact_occupancy_map = NULL;
	static carmen_map_t occupancy_map;

	compact_occupancy_map = obstacle_distance_mapper_uncompress_occupancy_map(&occupancy_map, compact_occupancy_map, message);

	moving_objects_detection_tracking_publishing_and_occupancy_map_removal(occupancy_map, message->timestamp);
	
	if (distance_map.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&distance_map, message->config);
	carmen_prob_models_create_distance_map(&distance_map, &occupancy_map, obstacle_probability_threshold);

	obstacle_distance_mapper_publish_compact_distance_map(message->timestamp);
	obstacle_distance_mapper_publish_compact_cost_map(message->timestamp);

	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_distance_map);
}


static void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *msg)
{
	road_network_message = msg;
}


static void
carmen_mapper_map_message_handler(carmen_mapper_map_message *msg)
{
	build_occupancy_map(occupancy_map, msg);

//	moving_objects_detection_tracking_publishing_and_occupancy_map_removal(occupancy_map, msg->timestamp);

	if (distance_map.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&distance_map, msg->config);
	carmen_prob_models_create_distance_map(&distance_map, &occupancy_map, obstacle_probability_threshold);
	obstacle_distance_mapper_publish_compact_distance_and_compact_lane_contents_maps(msg->timestamp);
	
	obstacle_distance_mapper_publish_compact_cost_map(msg->timestamp);

	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_distance_map);
}


static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
	compact_lane_map = message;
}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	behaviour_selector_compact_lane_contents_message = message;
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("obstacle_distance_mapper: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initialization                                                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
register_handlers()
{
	if (behavior_selector_use_symotha)
		carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) carmen_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_compact_lane_map(NULL, (carmen_handler_t) map_server_compact_lane_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
		{(char *) "rrt",						(char *) "obstacle_cost_distance",			CARMEN_PARAM_DOUBLE,	&obstacle_cost_distance,			1, NULL},
		{(char *) "rrt",						(char *) "obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "min_moving_object_velocity",		CARMEN_PARAM_DOUBLE,	&min_moving_object_velocity,		1, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "max_moving_object_velocity",		CARMEN_PARAM_DOUBLE,	&max_moving_object_velocity,		1, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "moving_object_merge_distance",	CARMEN_PARAM_DOUBLE,	&moving_object_merge_distance,		1, NULL},

		{(char *) "behavior_selector",			(char *) "use_symotha",						CARMEN_PARAM_ONOFF,		&behavior_selector_use_symotha,		1, NULL}
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////


int 
main(int argc, char **argv) 
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);
	register_handlers();

    carmen_moving_objects_point_clouds_define_messages();

	distance_map.complete_distance = NULL;

	carmen_ipc_dispatch();

	return (0);
}
