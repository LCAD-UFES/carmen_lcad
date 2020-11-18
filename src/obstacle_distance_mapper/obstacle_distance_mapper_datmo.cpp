#include <vector>
#include <algorithm>
#include <carmen/carmen.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/collision_detection.h>
#include <prob_map.h>
#include "obstacle_distance_mapper_datmo.h"
#include "obstacle_distance_mapper_interface.h"

#define LATERAL_V_MULTIPLIER 0.1

extern double min_moving_object_velocity;
extern double max_moving_object_velocity;
extern double obstacle_probability_threshold;
extern double moving_object_merge_distance;
extern double distance_car_pose_car_front;

carmen_localize_ackerman_globalpos_message *localize_ackerman_globalpos_message = NULL;
carmen_behavior_selector_goal_list_message *behavior_selector_goal_list_message = NULL;

double maximum_acceleration_forward;

//
// No codigo abaixo, a deteccao e o traqueamento de objetos moveis eh feito apenas nas lanes da road_network
// enviada pelo route planner.
//
// INICIALIZACAO
//
//  Para realizar a deteccao e o traqueamento, este codigo mantem uma lista de lanes que eh composta de uma lista de objetos moveis
//  encontrados em cada lane.
//
//  Primeiramente, a lista de lanes eh atualizada com as lanes na road_network recebida. Lanes jah presentes na lista de lanes
//  sao atualizadas com o conteudo da road_network. Lanes presentes  na road_network e nao presentes na lista sao incluidas, e
//  lanes na lista nao presentes em road_network sao removidas da lista.
//
// TRAQUEAMENTO
//
//  Apos a inicializacao, a lista de lanes e respectivos objetos móveis eh examinada para o traqueamento dos objetos jah detectados
//  em cada lane em ciclos anteriores.
//
//  Este traqueamento eh feito fazendo uma predicao de para onde os objetos de cada lane se moveram e examinando se
//  os objetos estao lah. Um vetor do tamanho de cada lane eh mantido para marcar as posicoes da lane jah visitadas e, deste modo,
//  durante a deteccao (abaixo), evitar a redeteccao por engano. Estatisticas dos objetos sao computadas
//  neste processo de traqueamento. Objetos não reencontrados no processo recebem incremento de contagem para remocao.
//
// DETECCAO
//
//  A deteccao de objetos em uma lane eh feita examinando em sequencia cada pose nao examinada no processo de traqueamento e checando se,
//  nesta pose, existe um obstaculo *ateh a uma distancia igual aa metada da largura da lane*.
//
//  Detectado um objeto, posicoes subsequentes na lane sao examinadas para delimitar o tamanho e eventuais outras caracteristicas do objeto.
//  Listas de objetos moveis de outras lanes são tambem examinadas para ver se o objeto nao estah mudando para esta lane.
//  Se este for o caso, as estatisticas acumuladas na outra lane sao copiada para esta.
//
// FINALIZACAO
//
//  Objetos com contagem elevada para remoccao sao removidos da lista de lanes e respectivos objetos móveis.
//  Os objetos moveis traqueados sao devolvidos para o sistema em uma mensagem carmen_moving_objects_point_clouds_message.
//


using namespace std;


typedef struct
{
	int lane_id;
	int size;
	carmen_ackerman_traj_point_t *lane_points;
	int *traffic_restrictions;
	vector<bool> examined;
	vector<moving_object_t> moving_objects;
} lane_t;


int next_mo_id = 0;


static int
get_num_valid_samples(moving_object_history_t *history)
{
	int sum = 0;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
		if (history[i].valid)
			sum++;

	return (sum);
}


static int
get_last_valid_index(moving_object_history_t *history)
{
	int index = -1;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (history[i].valid)
		{
			index = history[i].index;
			break;
		}
	}

	return (index);
}


static int
get_last_valid_history_sample(moving_object_history_t *history)
{
	int sample = -1;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (history[i].valid)
		{
			sample = i;
			break;
		}
	}

	return (sample);
}


static lane_t *
get_lane(vector<lane_t> &lanes, int lane_id)
{
	lane_t *lane;
	for (unsigned int i = 0; i < lanes.size(); i++)
    {
    	if (lane_id == lanes[i].lane_id)
    	{
    		lane = &(lanes[i]);
    		return (lane);
    	}
    }

	return (NULL);
}


static void
shift_history(moving_object_t *moving_object)
{
	if (moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].v_valid)
	{
		moving_object->average_longitudinal_v.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].pose.v);
		moving_object->average_lateral_v.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].lateral_v);
	}

	if (moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].valid)
	{
		moving_object->average_length.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].length);
		moving_object->average_width.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].width);
	}

	moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].moving_object_points.clear();

	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0; i--)
		moving_object->history[i + 1] = moving_object->history[i];

	moving_object->history[0].valid = 0;
	moving_object->history[0].v_valid = 0;
}


inline bool
touch_offline_map(carmen_map_server_offline_map_message *offline_map, double x_world, double y_world)
{
	if (!offline_map)
		return (false);

	// Precisa computar novamente pois a origin pode ser diferente
	int x_0 = (int) (round((x_world - offline_map->config.x_origin) / offline_map->config.resolution));
	int y_0 = (int) (round((y_world - offline_map->config.y_origin) / offline_map->config.resolution));
	int x_size = offline_map->config.x_size;
	int y_size = offline_map->config.y_size;
	for (int x_map_cell = x_0 - 1; x_map_cell <= x_0 + 1; x_map_cell++)
	{
		if ((x_map_cell < 0 || x_map_cell >= x_size))
			continue;
		for (int y_map_cell = y_0 - 1; y_map_cell <= y_0 + 1; y_map_cell++)
		{
			if ((y_map_cell < 0 || y_map_cell >= y_size))
				continue;
			if (offline_map->complete_map[x_map_cell * y_size + y_map_cell] >= obstacle_probability_threshold)
				return (true);
		}
	}

	return (false);
}


static bool
obstacle_detected(lane_t *lane, int i, carmen_map_t *occupancy_map, carmen_map_server_offline_map_message *offline_map)
{
	if ((i < 0) || (i >= (lane->size - 1)))
		carmen_die("Fatal error: (i < 0) || (i >= (lane->size - 1)) in obstacle_detected().");

	double lane_left_width = ROUTE_PLANNER_GET_LANE_LEFT_WIDTH(lane->traffic_restrictions[i]);
	double lane_right_width = ROUTE_PLANNER_GET_LANE_RIGHT_WIDTH(lane->traffic_restrictions[i]);
	double distance = DIST2D(lane->lane_points[i], lane->lane_points[i + 1]);
	if (distance > 2.5)
	{
		printf("DIST2D(lane->lane_points[i] (x %.2lf, y %.2lf), lane->lane_points[i + 1]  (x %.2lf, y %.2lf)) > 2.5 in obstacle_detected().\n",
				lane->lane_points[i].x, lane->lane_points[i].y, lane->lane_points[i + 1].x, lane->lane_points[i + 1].y);
		distance = 2.5;
	}
	// Se colocar distance no lugar de DIST2D(lane->lane_points[i], lane->lane_points[i + 1]) abaixo não funciona direito em modo otimizado. Vai entender...
	for (double s = 0.0; s < DIST2D(lane->lane_points[i], lane->lane_points[i + 1]); s += occupancy_map->config.resolution * 0.5)
	{
		double lane_x = lane->lane_points[i].x + s * cos(lane->lane_points[i].theta);
		double lane_y = lane->lane_points[i].y + s * sin(lane->lane_points[i].theta);
		for (double d = -lane_right_width; d < lane_left_width; d += occupancy_map->config.resolution * 0.5)
		{
			double x = lane_x + d * cos(lane->lane_points[i].theta + M_PI / 2.0);
			double y = lane_y + d * sin(lane->lane_points[i].theta + M_PI / 2.0);

			// Move global path point coordinates to map coordinates
			int x_map_cell = (int) round((x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
			int y_map_cell = (int) round((y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
			if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
				continue;

			if (occupancy_map->map[x_map_cell][y_map_cell] >= obstacle_probability_threshold)
				if (!touch_offline_map(offline_map, x, y))
					return (true);
		}
	}

	return (false);
}


static int
get_index_of_the_nearest_lane_pose(carmen_ackerman_traj_point_t pose, lane_t *lane)
{
	int nearest_pose_index = 0;
	double min_distance = DIST2D(pose, lane->lane_points[0]);
	for (int i = 1; i < lane->size; i++)
	{
		double distance = DIST2D(pose, lane->lane_points[i]);
		if (distance < min_distance)
		{
			min_distance = distance;
			nearest_pose_index = i;
		}
	}

	return (nearest_pose_index);
}


static int
predict_object_pose_index(moving_object_t *moving_object, lane_t *lane, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	int index = get_index_of_the_nearest_lane_pose(moving_object->history[1].pose, lane);

	// Prediz o novo indice em função da velocidade atual
	double delta_t = timestamp - moving_object->history[1].timestamp;
//	double displacement = delta_t * moving_object->v;
	double displacement = delta_t * moving_object->average_longitudinal_v.arithmetic_mean();
	if (fabs(displacement) != 0.0)
	{
		int new_index = index;
		double displacement_in_lane = 0.0;
		while (fabs(displacement_in_lane) < fabs(displacement))
		{
			int aux;
			if (displacement > 0.0)
			{
				aux = new_index + 1;
				if (aux ==  lane->size)
					break;
			}
			else
			{
				aux = new_index - 1;
				if (aux ==  -1)
					break;
			}
			displacement_in_lane += DIST2D(lane->lane_points[aux], lane->lane_points[new_index]);
			new_index = aux;
		}
		index = new_index;
	}

	// Procura pelo objeto móvel na lane (daí, só precisa da velocidade longitudinal) assumindo que a velocidade está dentro de certos limites
	int delta_index = 0;
	bool within_up_limit = true;
	bool within_down_limit = true;
	int found_index = -1;
	double v_max_displacement;
	if (moving_object->average_longitudinal_v.num_samples() >= 2)
		v_max_displacement = 3.0 * moving_object->average_longitudinal_v.standard_deviation(); // 3 sigmas
	else
		v_max_displacement = max_moving_object_velocity;
	double max_disp = delta_t * v_max_displacement;

	while (within_up_limit || within_down_limit)
	{
		int increased_index = index + delta_index;
		if ((increased_index >= (lane->size - 1)) ||
			(DIST2D(lane->lane_points[index], lane->lane_points[increased_index]) > max_disp)) // isso é apenas uma aproximação em curvas
			within_up_limit = false;
		int decreased_index = index - delta_index;
		if ((decreased_index < 0) ||
			(DIST2D(lane->lane_points[index], lane->lane_points[decreased_index]) > max_disp)) // isso é apenas uma aproximação em curvas
			within_down_limit = false;

		if (within_up_limit)
		{
			if ((increased_index > 0) && !lane->examined[increased_index] && obstacle_detected(lane, increased_index, occupancy_map, offline_map))
			{
				lane->examined[increased_index] = true;
				found_index = increased_index;
				break;
			}
			if (increased_index > 0)
				lane->examined[increased_index] = true;
		}

		if (within_down_limit)
		{
			if ((decreased_index < (lane->size - 1)) && !lane->examined[decreased_index] && obstacle_detected(lane, decreased_index, occupancy_map, offline_map))
			{
				lane->examined[decreased_index] = true;
				found_index = decreased_index;
				break;
			}
			if (decreased_index < lane->size)
				lane->examined[decreased_index] = true;
		}
		delta_index++;
	}

	return (found_index);
}


static int
get_last_valid_history(moving_object_t *moving_object)
{
	int last_valid_history = -1;
	for (int i = 1; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (moving_object->history[i].valid)
		{
			last_valid_history = i;
			break;
		}
	}

	return (last_valid_history);
}


static void
try_and_add_a_v_sample(moving_object_t *moving_object)
{
	double delta_t = 0.0;

	int next_valid = get_last_valid_history(moving_object);
	if (next_valid != -1)
		delta_t = moving_object->history[0].timestamp - moving_object->history[next_valid].timestamp;

	if (delta_t > 0.01)
	{
		double dist = DIST2D(moving_object->history[0].pose, moving_object->history[next_valid].pose);
		// distance in the direction of the lane: https://en.wikipedia.org/wiki/Vector_projection
		double angle = ANGLE2D(moving_object->history[next_valid].pose, moving_object->history[0].pose);

		double distance = dist * cos(angle - moving_object->history[0].pose.theta);
		moving_object->history[0].pose.v = distance / delta_t;
		moving_object->average_longitudinal_v.add_sample(moving_object->history[0].pose.v);

		distance = dist * sin(angle - moving_object->history[0].pose.theta) * LATERAL_V_MULTIPLIER;
		moving_object->history[0].lateral_v = distance / delta_t;
		moving_object->average_lateral_v.add_sample(moving_object->history[0].lateral_v);

		moving_object->history[0].v_valid = 1;
	}
	else
	{
		moving_object->history[0].v_valid = 0;
		moving_object->history[0].pose.v = 0.0;
	}
}


static void
get_object_history_sample_mass_center(moving_object_history_t *moving_object, vector <carmen_position_t> moving_object_points)
{
	moving_object->pose.x = 0.0;
	moving_object->pose.y = 0.0;

	for (unsigned int i = 0; i < moving_object_points.size(); i++)
	{
		moving_object->pose.x += moving_object_points[i].x;
		moving_object->pose.y += moving_object_points[i].y;
	}

	moving_object->pose.x /= (double) moving_object_points.size();
	moving_object->pose.y /= (double) moving_object_points.size();
}


static void
compute_object_history_sample_width_and_length_and_correct_pose(moving_object_history_t *mo_sample, double map_resolution)
{
	double min_x, max_x, min_y, max_y;
	min_x = max_x = min_y = max_y = 0.0;
	double sin, cos;
	sincos(-mo_sample->pose.theta, &sin, &cos);
	for (unsigned int i = 0; i < mo_sample->moving_object_points.size(); i++)
	{
		double x = mo_sample->moving_object_points[i].x - mo_sample->pose.x;
		double y = mo_sample->moving_object_points[i].y - mo_sample->pose.y;
		double xr = x * cos - y * sin;
		double yr = x * sin + y * cos;
		max_x = (xr > max_x)? xr: max_x;
		max_y = (yr > max_y)? yr: max_y;
		min_x = (xr < min_x)? xr: min_x;
		min_y = (yr < min_y)? yr: min_y;
	}

	double length = max_x - min_x;
	double width = max_y - min_y;
	double additional_size_due_to_map_discretization = map_resolution * M_SQRT2;
	mo_sample->length = ((length > map_resolution)? length: map_resolution) + additional_size_due_to_map_discretization;
	mo_sample->width = ((width > map_resolution)? width: map_resolution) + additional_size_due_to_map_discretization;

	double x = (max_x + min_x) / 2.0; // pose correction in x
	double y = (max_y + min_y) / 2.0; // pose correction in y
	sincos(mo_sample->pose.theta, &sin, &cos); // reorient to theta
	double xr = x * cos - y * sin;
	double yr = x * sin + y * cos;
	mo_sample->pose.x += xr;
	mo_sample->pose.y += yr;
}


static void
update_object_statistics(moving_object_t *moving_object, carmen_map_t *occupancy_map)
{
	get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object->history[0].moving_object_points);
	compute_object_history_sample_width_and_length_and_correct_pose(&(moving_object->history[0]), occupancy_map->config.resolution);

	if (moving_object->history[0].valid)
	{
		try_and_add_a_v_sample(moving_object);

		moving_object->non_detection_count = 0;
		moving_object->pose = moving_object->history[0].pose;
		moving_object->average_length.add_sample(moving_object->history[0].length);
		moving_object->average_width.add_sample(moving_object->history[0].width);
//		if (moving_object->average_longitudinal_v.num_samples() != 0)
//			moving_object->v = moving_object->v + 0.2 * (moving_object->average_longitudinal_v.arithmetic_mean() - moving_object->v);
//		else
//			moving_object->v = 0.0;
	}
}


static bool
get_object_points(vector <carmen_position_t> &moving_object_points, lane_t *lane, int i, bool forward, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map)
{
	double signal = (forward)? 1.0: -1.0;
	if ((i < 0) || (i + signal >= lane->size) || (i + signal < 0))
		carmen_die("Fatal error: (i < 0) || (i + signal >= lane->size) || (i + signal < 0) in get_object_points().");

	bool found = false;
	double lane_left_width = ROUTE_PLANNER_GET_LANE_LEFT_WIDTH(lane->traffic_restrictions[i]);
	double lane_right_width = ROUTE_PLANNER_GET_LANE_RIGHT_WIDTH(lane->traffic_restrictions[i]);
	double s_distance = DIST2D(lane->lane_points[i], lane->lane_points[i + (int) signal]);
	if (s_distance > 2.5)
	{
		printf("DIST2D(lane->lane_points[i] (x %.2lf, y %.2lf), lane->lane_points[i + (int) signal]  (x %.2lf, y %.2lf)) > 2.5 in get_object_points().\n",
				lane->lane_points[i].x, lane->lane_points[i].y, lane->lane_points[i + (int) signal].x, lane->lane_points[i + (int) signal].y);
		s_distance = 2.5;
	}

	for (double s = 0.0; s < s_distance; s += occupancy_map->config.resolution * 0.5)
	{
		double lane_x = lane->lane_points[i].x + signal * s * cos(lane->lane_points[i].theta);
		double lane_y = lane->lane_points[i].y + signal * s * sin(lane->lane_points[i].theta);
		for (double d = -lane_right_width; d < lane_left_width; d += occupancy_map->config.resolution * 0.5)
		{
			double x = lane_x + d * cos(lane->lane_points[i].theta + M_PI / 2.0);
			double y = lane_y + d * sin(lane->lane_points[i].theta + M_PI / 2.0);

			// Move global path point coordinates to map coordinates
			int x_map_cell = (int) round((x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
			int y_map_cell = (int) round((y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
			if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
				continue;

			if (occupancy_map->map[x_map_cell][y_map_cell] >= obstacle_probability_threshold)
			{
				if (!touch_offline_map(offline_map, x, y))
				{
					found = true;

					x = occupancy_map->config.resolution * round(x / occupancy_map->config.resolution);
					y = occupancy_map->config.resolution * round(y / occupancy_map->config.resolution);
					moving_object_points.push_back({x, y});
				}
			}
		}
	}

	return (found);
}


static bool
compare_points(carmen_position_t a, carmen_position_t b)
{
	if (a.x < b.x)
	{
		return (true);
	}
	else if (a.x == b.x)
	{
		if (a.y < b.y)
		{
			return (true);
		}
	}

    return (false);
}


static bool
same_points(carmen_position_t a, carmen_position_t b)
{
	if ((a.x == b.x) && (a.y == b.y))
		return (true);
	else
		return (false);
}


static void
remove_duplicate_points(vector <carmen_position_t> &moving_object_points)
{
	sort(moving_object_points.begin(), moving_object_points.end(), compare_points);
	moving_object_points.erase(unique(moving_object_points.begin(), moving_object_points.end(), same_points), moving_object_points.end());
}


static bool
add_object_history_sample(moving_object_t *moving_object, int first_pose_index, lane_t *lane,
		carmen_map_t *occupancy_map, carmen_map_server_offline_map_message *offline_map, double timestamp, bool forward)
{
	int i = first_pose_index;
	vector <carmen_position_t> moving_object_points;
	moving_object->history[0] = {};


	bool found = false;
	if (forward)
	{
 		while ((i < (lane->size - 1)) && get_object_points(moving_object_points, lane, i, forward, occupancy_map, offline_map))
		{
 			found = true;
			lane->examined[i] = true;
			i++;
		}
		i--;
	}
	else
	{
		while ((i > 0) && get_object_points(moving_object_points, lane, i, forward, occupancy_map, offline_map))
		{
 			found = true;
			lane->examined[i] = true;
			i--;
		}
		i++;
	}
//	printf("last %d\n", i);

	if (found)
	{
		moving_object->history[0].valid = 1;
		int index = round((double) (first_pose_index + i) / 2.0);
		moving_object->history[0].index = index;
		lane->examined[index] = true;
		remove_duplicate_points(moving_object_points);
		get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object_points);
		moving_object->history[0].moving_object_points = moving_object_points;
		moving_object->history[0].pose.theta = lane->lane_points[moving_object->history[0].index].theta;
		compute_object_history_sample_width_and_length_and_correct_pose(&(moving_object->history[0]), occupancy_map->config.resolution);
		moving_object->history[0].timestamp = timestamp;

		return (true);
	}
	else
	{
		moving_object->history[0].valid = 0;

		return (false);
	}
}


static bool
data_association(moving_object_t *moving_object, lane_t *lane, int index, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	int last_valid_history = get_last_valid_history(moving_object);

	if (last_valid_history == -1)
		return (false);

	bool found = false;
	while ((index >= 0) && obstacle_detected(lane, index, occupancy_map, offline_map))
	{
		lane->examined[index] = true;
		index--;
	}
	index++;

	found = add_object_history_sample(moving_object, index, lane, occupancy_map, offline_map, timestamp, true);
//	printf("track %d (%.2lf), ", moving_object->id, moving_object->pose.theta);
	if (found)
		return (true);
	else
		return (false);
}


static bool
track(moving_object_t *moving_object, lane_t *lane, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	int index = predict_object_pose_index(moving_object, lane, occupancy_map, offline_map, timestamp);
//	printf("index %d, id %d, ", index, moving_object->id);

	if (index != -1)
		return (data_association(moving_object, lane, index, occupancy_map, offline_map, timestamp));
	else
		return (false);
}


static void
round_points(vector <carmen_position_t> &moving_object_points, carmen_map_t *occupancy_map)
{
	for (int i = 0; i < (int) moving_object_points.size(); i++)
	{
		double x = moving_object_points[i].x;
		double y = moving_object_points[i].y;
		x = occupancy_map->config.resolution * round(x / occupancy_map->config.resolution);
		y = occupancy_map->config.resolution * round(y / occupancy_map->config.resolution);
		moving_object_points[i].x = x;
		moving_object_points[i].y = y;
	}
}


static bool
estimate_new_state(moving_object_t *moving_object, lane_t *lane, carmen_map_t *occupancy_map, double timestamp)
{
	int sample = get_last_valid_history_sample(moving_object->history);
	if (sample < 0)
		return (true); // delete moving object...
	
	moving_object->history[0].moving_object_points = moving_object->history[sample].moving_object_points;
	moving_object->history[0].pose = moving_object->history[sample].pose; // para copiar v

	int new_index = get_index_of_the_nearest_lane_pose(moving_object->history[sample].pose, lane);
	double delta_t = timestamp - moving_object->history[sample].timestamp;
	double longitudinal_displacement = delta_t * moving_object->average_longitudinal_v.arithmetic_mean();
	double lateral_displacement = delta_t * moving_object->average_lateral_v.arithmetic_mean();
	bool delete_moving_object = false;
	if ((fabs(longitudinal_displacement) != 0.0) || (fabs(lateral_displacement) != 0.0))
	{
		double delta_s = longitudinal_displacement / (2.0 * (double) sample);
		double delta_d = lateral_displacement / (2.0 * (double) sample);
		for (int i = 0; i < 2 * sample; i++)
		{
			if ((new_index >=  lane->size) || (new_index < 0))
			{
				delete_moving_object = true;
				break;
			}

			for (unsigned int i = 0; i < moving_object->history[0].moving_object_points.size(); i++)
			{
				moving_object->history[0].moving_object_points[i].x += delta_s * cos(lane->lane_points[new_index].theta) +
						delta_d * cos(lane->lane_points[new_index].theta + M_PI / 2.0);
				moving_object->history[0].moving_object_points[i].y += delta_s * sin(lane->lane_points[new_index].theta) +
						delta_d * sin(lane->lane_points[new_index].theta + M_PI / 2.0);
			}

			get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object->history[0].moving_object_points);
			compute_object_history_sample_width_and_length_and_correct_pose(&(moving_object->history[0]), occupancy_map->config.resolution);
			new_index = get_index_of_the_nearest_lane_pose(moving_object->history[0].pose, lane);
		}

		round_points(moving_object->history[0].moving_object_points, occupancy_map);
		get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object->history[0].moving_object_points);
		compute_object_history_sample_width_and_length_and_correct_pose(&(moving_object->history[0]), occupancy_map->config.resolution);
	}
	moving_object->history[0].index = new_index;
	moving_object->history[0].width =  moving_object->history[sample].width;
	moving_object->history[0].length =  moving_object->history[sample].length;
	moving_object->pose = moving_object->history[0].pose;

	return (delete_moving_object);
}


static void
track_moving_objects(lane_t *lane, carmen_map_t *occupancy_map, carmen_map_server_offline_map_message *offline_map, double timestamp)
{
    for (int i = 0; i < (int) lane->moving_objects.size(); i++)
    {
    	moving_object_t *moving_object = &(lane->moving_objects[i]);
    	shift_history(moving_object);
    	bool found = track(moving_object, lane, occupancy_map, offline_map, timestamp);
    	if (!found)
    	{
    		bool delete_moving_object = estimate_new_state(moving_object, lane, occupancy_map, timestamp);
    		moving_object->non_detection_count++;
    		if (delete_moving_object || (moving_object->non_detection_count > MAX_NON_DETECTION_COUNT))
    		{
//				printf("erase in track: %d, ", lane->moving_objects[i].id);
    			lane->moving_objects.erase(lane->moving_objects.begin() + i);
    			i--;
    		}
//    		else
//    			printf("track_es %d (%.2lf), ", moving_object->id, moving_object->pose.theta);
    	}
    }
}


vector <carmen_position_t>
get_points_within_moving_object_rectangle(moving_object_t *mo)
{
	if (moving_object_merge_distance == 0.0)
		carmen_die("moving_object_merge_distance == 0.0 in get_points_within_moving_object_rectangle()");

	double length = mo->average_length.arithmetic_mean();
	double width = mo->average_width.arithmetic_mean();
	double slices_x = ceil(length / (moving_object_merge_distance / 2.0));
	double delta_x = length / slices_x;
	double slices_y = ceil(width / (moving_object_merge_distance / 2.0));
	double delta_y = width / slices_y;
	double sin, cos;
	sincos(mo->pose.theta, &sin, &cos);

	vector <carmen_position_t> points;
	for (double x = -length / 2.0; x < ((length / 2.0) + delta_x / 2.0); x += delta_x)
	{
		for (double y = -width / 2.0; y < ((width / 2.0) + delta_y / 2.0); y += delta_y)
		{
			double xr = x * cos - y * sin;
			double yr = x * sin + y * cos;
			points.push_back({mo->pose.x + xr, mo->pose.y + yr});
		}
	}

	return (points);
}


static bool
close_orientation(moving_object_t *mo_j, moving_object_t *mo_k)
{
	double delta_angle = carmen_normalize_theta(mo_j->history[0].pose.theta - mo_k->history[0].pose.theta);
	double mo_j_v = mo_j->average_longitudinal_v.arithmetic_mean();
	double mo_k_v = mo_k->average_longitudinal_v.arithmetic_mean();
	double delta_v = fabs(mo_j_v - mo_k_v * cos(delta_angle));
	double delta_v_ratio = delta_v / ((fabs(mo_j_v) + fabs(mo_k_v))) / 2.0;
	if (delta_v_ratio < 0.3)
		return (true);
	else
		return (false);

}


static bool
near(moving_object_t *mo_j, moving_object_t *mo_k)
{	// A ordem é importante. O mais completo e conhecido precisa ser o mo_k.
	for (unsigned int j = 0; j < mo_j->history[0].moving_object_points.size(); j++)
	{
		carmen_position_t point_j = mo_j->history[0].moving_object_points[j];
		vector <carmen_position_t> points = get_points_within_moving_object_rectangle(mo_k);
		for (unsigned int k = 0; k < points.size(); k++)
		{
			if (DIST2D(point_j, points[k]) < moving_object_merge_distance)
			{
				points.clear();
				return (true);
			}
		}
		points.clear();
	}
	return (false);
}


bool
near_old(moving_object_t *mo_j, moving_object_t *mo_k)
{
	for (unsigned int j = 0; j < mo_j->history[0].moving_object_points.size(); j++)
	{
		carmen_position_t point_j = mo_j->history[0].moving_object_points[j];
		for (unsigned int k = 0; k < mo_k->history[0].moving_object_points.size(); k++)
		{
			carmen_position_t point_k = mo_k->history[0].moving_object_points[k];
			if (DIST2D(point_j, point_k) < moving_object_merge_distance)
				return (true);
		}
	}
	return (false);
}


static moving_object_t *
get_near_moving_object_from_another_lane(moving_object_t *moving_object, vector<lane_t> &lanes, int lane_index)
{
	for (unsigned int i = 0; i < lanes.size(); i++)
	{
		if (i != (unsigned int) lane_index)
		{
			for (unsigned int j = 0; j < lanes[i].moving_objects.size(); j++)
			{
				if (near(moving_object, &(lanes[i].moving_objects[j])))
					return (&(lanes[i].moving_objects[j]));
			}
		}
	}

	return (NULL);
}


static void
copy_points(moving_object_t *to, vector <carmen_position_t> from, double map_resolution)
{
	to->history[0].moving_object_points.insert(to->history[0].moving_object_points.end(), from.begin(), from.end());
	remove_duplicate_points(to->history[0].moving_object_points);
	get_object_history_sample_mass_center(&(to->history[0]), to->history[0].moving_object_points);
	compute_object_history_sample_width_and_length_and_correct_pose(&(to->history[0]), map_resolution);
}


static void
copy_history(moving_object_t *to, moving_object_t *from, double map_resolution)
{
	double delta_angle = carmen_normalize_theta(from->history[0].pose.theta - to->history[0].pose.theta);
//	to->v = from->v * cos(delta_angle);
	to->history[0].pose.v = from->history[0].pose.v * cos(delta_angle);
	copy_points(to, from->history[0].moving_object_points, map_resolution);

	for (int i = 1; i < MOVING_OBJECT_HISTORY_SIZE; i++) // A history[0] nao deve ser copiada pois jah contem a info mais recente
	{
		to->history[i] = from->history[i];
		// Tem que somar a diferença de ângulo entre poses das duas lanes para corrigir o sinal da velocidade, se necessário.
		to->history[i].pose.theta = carmen_normalize_theta(from->history[i].pose.theta + delta_angle);

		if (from->history[i].valid)
		{
			to->average_length.add_sample(from->history[i].length);
			to->average_width.add_sample(from->history[i].width);

			if (from->history[i].v_valid)
			{
				to->history[i].pose.v = from->history[i].pose.v * cos(delta_angle);
				to->average_longitudinal_v.add_sample(to->history[i].pose.v);
				to->history[i].lateral_v = from->history[i].lateral_v * cos(delta_angle);
				to->average_lateral_v.add_sample(to->history[i].lateral_v);
			}
		}
	}
//	printf("copy history %d (%.2lf) <- %d (%.2lf), ", to->id, to->history[0].pose.theta, from->id, from->pose.theta);
}


static void
merge(moving_object_t *mo_j, moving_object_t *mo_k, double map_resolution)
{
//	printf("merge %d (%.2lf) <- %d (%.2lf), ", mo_j->id, mo_j->pose.theta, mo_k->id, mo_k->pose.theta);
//	if (mo_j->history[0].valid)
	{
		int size_j = mo_j->history[0].moving_object_points.size();
		int size_k = mo_k->history[0].moving_object_points.size();
		mo_j->history[0].index = (mo_j->history[0].index * size_j + mo_k->history[0].index * size_k) / (size_j + size_k);

		mo_j->history[0].moving_object_points.insert(mo_j->history[0].moving_object_points.end(), mo_k->history[0].moving_object_points.begin(), mo_k->history[0].moving_object_points.end());
		remove_duplicate_points(mo_j->history[0].moving_object_points);
		get_object_history_sample_mass_center(&(mo_j->history[0]), mo_j->history[0].moving_object_points);
		compute_object_history_sample_width_and_length_and_correct_pose(&(mo_j->history[0]), map_resolution);
	}
//	else
//		mo_j->history[0] = mo_k->history[0];
}


static bool
merge_with_in_lane_nearby_objects(lane_t *lane, moving_object_t *moving_object, double map_resolution)
{
	for (unsigned int j = 0; j < lane->moving_objects.size(); j++)
	{
		if (near(moving_object, &(lane->moving_objects[j])))
		{
//			printf("in detect: ");
			fflush(stdout);
			merge(&(lane->moving_objects[j]), moving_object, map_resolution);
			return (true);
		}
	}

	return (false);
}


static void
add_detected_objects(vector<lane_t> &lanes, int lane_index, int first_pose_index, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	moving_object_t moving_object = {};
//	memset(&(moving_object.history), 0, sizeof(moving_object.history));

	add_object_history_sample(&moving_object, first_pose_index, lane, occupancy_map, offline_map, timestamp, true);

	if (!merge_with_in_lane_nearby_objects(lane, &moving_object, occupancy_map->config.resolution))
	{
		moving_object.id = next_mo_id++;
		if (next_mo_id > 999)
			next_mo_id = 0;

		moving_object_t *moving_object_from_another_lane;
		if ((moving_object_from_another_lane = get_near_moving_object_from_another_lane(&moving_object, lanes, lane_index)) != NULL)
			copy_history(&moving_object, moving_object_from_another_lane, occupancy_map->config.resolution);

//		get_object_history_sample_mass_center(&(moving_object.history[0]), moving_object.history[0].moving_object_points);
//		compute_object_history_sample_width_and_length_and_correct_pose(&(moving_object.history[0]), occupancy_map->config.resolution);
		moving_object.pose = moving_object.history[0].pose;

//		printf("added %d (%.2lf), ", moving_object.id, moving_object.pose.theta);
		lane->moving_objects.push_back(moving_object);
	}
//	if (moving_object.history[0].moving_object_points.size() > 300)
//		printf(" ");
}


static void
detect_new_objects(vector<lane_t> &lanes, int lane_index, carmen_map_t *occupancy_map,
		carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	for (int i = 0; i < (lane->size - 1); i++)
		if (!lane->examined[i] && obstacle_detected(lane, i, occupancy_map, offline_map))
			add_detected_objects(lanes, lane_index, i, occupancy_map, offline_map, timestamp);
}


static char
get_vehicle_category(double width, double length, double longitudinal_v, double lateral_v, int num_samples)
{
	vector<vehicle_category_t> categories = {{BIKE, 0.9, 3.0}, {BIKE, 0.5, 1.5},
											 {PEDESTRIAN, 0.5, 0.5}, {BUS, 2.2, 15.0},
											 {CAR, 1.7, 4.1}, {CAR, 1.7, 2.0}, {CAR, 2.0, 8.0}}; // Trung-Dung Vu Thesis

	double v = sqrt(longitudinal_v * longitudinal_v + lateral_v * lateral_v);

//	printf("w %.2lf, l %0.2lf, ", width, length);

	if ((v == 0.0) && (num_samples <= 1))
	{
//		printf("u - U\n");
		return ('U');
	}

	for (int i = 0; i < (int) categories.size(); i++)
	{
		if ((categories[i].length > (0.5 * length)) && (categories[i].length < (1.7 * length)) &&
			(categories[i].width > (0.5 * width)) && (categories[i].width < (1.7 * width)))
		{
			if ((categories[i].category == PEDESTRIAN) && (v > 15.0/3.6)) // pedestrian too fast
			{
//				printf("a - %c -> (c.length %.1lf > %.2lf) && (c.length %.1lf < %.2lf) && (c.width %.1lf > %.2lf) && (c.width %.1lf < %.2lf)\n",
//						categories[category].category, categories[category].length, 0.5 * length, categories[category].length, 1.7 * length,
//						categories[category].width, 0.5 * width, categories[category].width, 1.7 * width);
				return (UNKNOWN);
			}
			else if (((categories[i].category == BIKE) && ((length / width) < 2.0)) || // no motorcycle
					 ((categories[i].category == BIKE) && (width > 1.55)))
			{
				continue;
			}
			else
			{
//				printf("b - %c -> (c.length %.1lf > %.2lf) && (c.length %.1lf < %.2lf) && (c.width %.1lf > %.2lf) && (c.width %.1lf < %.2lf)\n",
//						categories[category].category, categories[category].length, 0.5 * length, categories[category].length, 1.7 * length,
//						categories[category].width, 0.5 * width, categories[category].width, 1.7 * width);
				return (categories[i].category);
			}
		}
	}

//	printf("c - %c -> (c.length %.1lf > %.2lf) && (c.length %.1lf < %.2lf) && (c.width %.1lf > %.2lf) && (c.width %.1lf < %.2lf)\n",
//			categories[category].category, categories[category].length, 0.5 * length, categories[category].length, 1.7 * length,
//			categories[category].width, 0.5 * width, categories[category].width, 1.7 * width);

	return (UNKNOWN);
}


static void
fill_in_moving_objects_message_element(t_point_cloud_struct *point_cloud, moving_object_t moving_object)
{
	point_cloud->in_front = moving_object.in_front;
	point_cloud->lane_id = moving_object.lane_id;
	point_cloud->lane_index = moving_object.history[0].index;
	point_cloud->index_in_poses_ahead = moving_object.index_in_poses_ahead;
	point_cloud->r = 0.0;
	point_cloud->g = 0.0;
	point_cloud->b = 1.0;
	point_cloud->num_valid_samples = get_num_valid_samples(moving_object.history);
	point_cloud->linear_velocity = moving_object.average_longitudinal_v.arithmetic_mean();
	point_cloud->linear_velocity_std = moving_object.average_longitudinal_v.standard_deviation();
	point_cloud->lateral_velocity = moving_object.average_lateral_v.arithmetic_mean();
	point_cloud->lateral_velocity_std = moving_object.average_lateral_v.standard_deviation();
	point_cloud->orientation = moving_object.pose.theta;
	point_cloud->object_pose.x = moving_object.pose.x;
	point_cloud->object_pose.y = moving_object.pose.y;
	point_cloud->object_pose.z = 0.0;
	point_cloud->length = moving_object.average_length.arithmetic_mean();
	point_cloud->length_std = moving_object.average_length.standard_deviation();
	point_cloud->width = moving_object.average_width.arithmetic_mean();
	point_cloud->width_std = moving_object.average_width.standard_deviation();
	point_cloud->height = moving_object.average_width.arithmetic_mean();
	point_cloud->height_std = moving_object.average_width.standard_deviation();
	point_cloud->geometric_model = get_vehicle_category(moving_object.average_width.arithmetic_mean(),
			moving_object.average_length.arithmetic_mean(),
			moving_object.average_longitudinal_v.arithmetic_mean(),
			moving_object.average_lateral_v.arithmetic_mean(),
			moving_object.average_length.num_samples());
	point_cloud->num_associated = moving_object.id;
	object_model_features_t &model_features = point_cloud->model_features;
	model_features.model_id = point_cloud->geometric_model;

	switch (model_features.model_id)
	{
		case BUS:
			model_features.model_name = (char *) ("Bus");
			model_features.red = 1.0;
			model_features.green = 1.0;
			model_features.blue = 0.0;
			break;
		case CAR:
			model_features.model_name = (char *) ("Car");
			model_features.red = 0.5;
			model_features.green = 1.0;
			model_features.blue = 0.0;
			break;
		case BIKE:
			model_features.model_name = (char *) ("(Motor)Bike");
			model_features.red = 1.0;
			model_features.green = 0.0;
			model_features.blue = 0.0;
			break;
		case PEDESTRIAN:
			model_features.model_name = (char *) ("Pedestrian");
			model_features.red = 0.0;
			model_features.green = 1.0;
			model_features.blue = 1.0;
			break;
		case UNKNOWN:
			model_features.model_name = (char *) (" ");
			model_features.red = 0.3;
			model_features.green = 0.3;
			model_features.blue = 0.3;
			break;
	}
	model_features.geometry.length = moving_object.average_length.arithmetic_mean();
	model_features.geometry.width = moving_object.average_width.arithmetic_mean();
	model_features.geometry.height = moving_object.average_width.arithmetic_mean();

	vector <carmen_position_t> moving_object_points = moving_object.history[0].moving_object_points;
	if (moving_object_points.size() > 0)
	{
		point_cloud->point_size = moving_object_points.size();
		point_cloud->points = (carmen_vector_3D_t *) malloc(point_cloud->point_size * sizeof(carmen_vector_3D_t));
		for (unsigned int i = 0; i < moving_object_points.size(); i++)
			point_cloud->points[i] = {moving_object_points[i].x, moving_object_points[i].y, 0.0};
	}
	else
	{
		point_cloud->point_size = 0;
		point_cloud->points = NULL;
	}
}


void
obstacle_distance_mapper_free_moving_objects_message(carmen_moving_objects_point_clouds_message *moving_objects)
{
	if (!moving_objects)
		return;

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
		free(moving_objects->point_clouds[i].points);
	free(moving_objects->point_clouds);
	free(moving_objects);
}


static carmen_moving_objects_point_clouds_message *
fill_in_moving_objects_point_clouds_message(vector<lane_t> lanes, double timestamp)
{
	carmen_moving_objects_point_clouds_message *message = (carmen_moving_objects_point_clouds_message *) malloc(sizeof(carmen_moving_objects_point_clouds_message));
	message->point_clouds = NULL;
	message->host = carmen_get_host();
	message->timestamp = timestamp;

	int num_moving_objects = 0;
	for (unsigned int i = 0; i < lanes.size(); i++)
		num_moving_objects += lanes[i].moving_objects.size();

	message->point_clouds = (t_point_cloud_struct *) realloc(message->point_clouds, sizeof(t_point_cloud_struct) * num_moving_objects);
	message->num_point_clouds = num_moving_objects;

	int mo_num = 0;
	for (unsigned int i = 0; i < lanes.size(); i++)
	{
		for (unsigned int j = 0; j < lanes[i].moving_objects.size(); j++)
		{
			fill_in_moving_objects_message_element(&(message->point_clouds[mo_num]), lanes[i].moving_objects[j]);

			mo_num++;
		}
	}

	return (message);
}


static void
remove_lanes_absent_from_road_network(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
//	printf("remove_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
//			road_network->number_of_nearby_lanes, (int) lanes.size());
//
    for (int i = 0; i < (int) lanes.size(); i++)
    {
    	int lane_id = lanes[i].lane_id;
    	bool not_found = true;
		for (int j = 0; j < road_network->number_of_nearby_lanes; j++)
		{
			if (lane_id == road_network->nearby_lanes_ids[j])
			{
				not_found = false;
				break;
			}
		}

		if (not_found)
		{
			for (int j = 0; j < (int) lanes[i].moving_objects.size(); j++)
			{
				for (int k = 0; k < MOVING_OBJECT_HISTORY_SIZE; k++)
					lanes[i].moving_objects[j].history[k].moving_object_points.clear();
				lanes[i].moving_objects.clear();
			}
			lanes.erase(lanes.begin() + i);
			i--;
		}
    }
}


static void
include_new_lanes_in_road_network(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
//	printf("include_new_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
//			road_network->number_of_nearby_lanes, (int) lanes.size());
//
	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
    	int lane_id = road_network->nearby_lanes_ids[i];
    	bool not_found = true;
        for (unsigned int j = 0; j < lanes.size(); j++)
        {
//        	printf("%d %d, ", lane_id, lanes[j].lane_id);
			if (lane_id == lanes[j].lane_id)
			{
				not_found = false;
				break;
			}
		}

		if (not_found)
		{
			lane_t new_lane;
			new_lane.lane_id = lane_id;
			new_lane.size = road_network->nearby_lanes_sizes[i];
			new_lane.lane_points = &(road_network->nearby_lanes[road_network->nearby_lanes_indexes[i]]);
			new_lane.traffic_restrictions = &(road_network->traffic_restrictions[road_network->nearby_lanes_indexes[i]]);
			vector<bool> examined(new_lane.size, false);
			new_lane.examined = examined;

			lanes.push_back(new_lane);
		}
    }
//	printf("\n");
//	fflush(stdout);
}


static void
update_lanes(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
//	printf("update_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
//			road_network->number_of_nearby_lanes, (int) lanes.size());
//
//	if (road_network->number_of_nearby_lanes != (int) lanes.size())
//		printf("Error: road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()",
//				road_network->number_of_nearby_lanes, (int) lanes.size());
//
	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
    	lane_t *lane = get_lane(lanes, road_network->nearby_lanes_ids[i]);
    	lane->lane_points = &(road_network->nearby_lanes[road_network->nearby_lanes_indexes[i]]);
    	int j;
    	for (j = 0; j < (road_network->nearby_lanes_sizes[i] - 1); j++)
    	{
			double distance = DIST2D(lane->lane_points[j], lane->lane_points[j + 1]);
			if (distance > 1.5)
			{
				printf("Error: too large distance between lane points (%lf)\n", distance);
				break;
			}
    	}
    	lane->size = j;

		vector<bool> examined(lane->size, false);
		lane->examined = examined;
    	lane->traffic_restrictions = &(road_network->traffic_restrictions[road_network->nearby_lanes_indexes[i]]);
    }
}


void
print_road_network(carmen_route_planner_road_network_message *road_network)
{
	printf("\n");

	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
		printf("lane_id %d, size %d\n", road_network->nearby_lanes_ids[i], road_network->nearby_lanes_sizes[i]);

		carmen_route_planner_junction_t *lane_merges = &(road_network->nearby_lanes_merges[road_network->nearby_lanes_merges_indexes[i]]);
		for (int j = 0; j < road_network->nearby_lanes_merges_sizes[i]; j++)
			printf("lane_id %d, merge %d: target_lane_id %d, node_id %d, index_of_node_in_current_lane %d, target_node_index_in_nearby_lane %d\n",
					road_network->nearby_lanes_ids[i], j,
					lane_merges[j].target_lane_id, lane_merges[j].node_id,
					lane_merges[j].index_of_node_in_current_lane, lane_merges[j].target_node_index_in_nearby_lane);

		carmen_route_planner_junction_t *lane_forks = &(road_network->nearby_lanes_forks[road_network->nearby_lanes_forks_indexes[i]]);
		for (int j = 0; j < road_network->nearby_lanes_forks_sizes[i]; j++)
			printf("lane_id %d, fork %d: target_lane_id %d, node_id %d, index_of_node_in_current_lane %d, target_node_index_in_nearby_lane %d\n",
					road_network->nearby_lanes_ids[i], j,
					lane_forks[j].target_lane_id, lane_forks[j].node_id,
					lane_forks[j].index_of_node_in_current_lane, lane_forks[j].target_node_index_in_nearby_lane);
	}

	printf("\n");
}


void
obstacle_distance_mapper_remove_moving_objects_from_occupancy_map(carmen_map_t *occupancy_map, carmen_moving_objects_point_clouds_message *moving_objects)
{
	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if ((fabs(moving_objects->point_clouds[i].linear_velocity) > min_moving_object_velocity) ||
			(moving_objects->point_clouds[i].num_valid_samples < 2))
		{
			for (int k = 0; k < moving_objects->point_clouds[i].point_size; k++)
			{
				int x_map_cell = (int) round((moving_objects->point_clouds[i].points[k].x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
				int y_map_cell = (int) round((moving_objects->point_clouds[i].points[k].y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
				if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
					continue;

				occupancy_map->map[x_map_cell][y_map_cell] = 0.0;
			}
		}
	}
}


void
obstacle_distance_mapper_restore_moving_objects_to_occupancy_map(carmen_map_t *occupancy_map, carmen_moving_objects_point_clouds_message *moving_objects)
{
	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if (fabs(moving_objects->point_clouds[i].linear_velocity) > min_moving_object_velocity)
		{
			for (int k = 0; k < moving_objects->point_clouds[i].point_size; k++)
			{
				int x_map_cell = (int) round((moving_objects->point_clouds[i].points[k].x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
				int y_map_cell = (int) round((moving_objects->point_clouds[i].points[k].y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
				if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
					continue;

				occupancy_map->map[x_map_cell][y_map_cell] = 0.9;
			}
		}
	}
}


carmen_compact_map_t *
obstacle_distance_mapper_uncompress_occupancy_map(carmen_map_t *occupancy_map, carmen_compact_map_t *compact_occupancy_map,
		carmen_mapper_compact_map_message *message)
{
	if (compact_occupancy_map == NULL)
	{
		carmen_grid_mapping_create_new_map(occupancy_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(occupancy_map->complete_map, 0, occupancy_map->config.x_size * occupancy_map->config.y_size * sizeof(double));
		occupancy_map->config = message->config;
		compact_occupancy_map = (carmen_compact_map_t *) calloc(1, sizeof(carmen_compact_map_t));
		carmen_cpy_compact_map_message_to_compact_map(compact_occupancy_map, message);
		carmen_prob_models_uncompress_compact_map(occupancy_map, compact_occupancy_map);
	}
	else
	{
		occupancy_map->config = message->config;
		carmen_prob_models_clear_carmen_map_using_compact_map(occupancy_map, compact_occupancy_map, 0.0);
		carmen_prob_models_free_compact_map(compact_occupancy_map);
		carmen_cpy_compact_map_message_to_compact_map(compact_occupancy_map, message);
		carmen_prob_models_uncompress_compact_map(occupancy_map, compact_occupancy_map);
	}

	return (compact_occupancy_map);
}


void
print_mo(vector<lane_t> lanes, double timestamp)
{
	printf("%lf\n", timestamp);
	for (unsigned int i = 0; i < lanes.size(); i++)
	{
		for (unsigned int j = 0; j < lanes[i].moving_objects.size(); j++)
		{
			printf("id %d, lane %d, index %d, in_front %d, index_in_p_a %d, valid %d, w_s %d, l_s %d, l %0.2lf, w %0.2lf, v_valid %d, vs_s %d, vd_s %d, vd %0.2lf, vd_std %0.2lf, vs %.2lf, vs_std %0.2lf   -   points %ld\n",
					lanes[i].moving_objects[j].id,
					lanes[i].lane_id,
					get_last_valid_index(lanes[i].moving_objects[j].history),
					lanes[i].moving_objects[j].in_front,
					lanes[i].moving_objects[j].index_in_poses_ahead,
					lanes[i].moving_objects[j].history[0].valid,
					lanes[i].moving_objects[j].average_width.num_samples(),
					lanes[i].moving_objects[j].average_length.num_samples(),
					lanes[i].moving_objects[j].average_length.arithmetic_mean(),
					lanes[i].moving_objects[j].average_width.arithmetic_mean(),
					lanes[i].moving_objects[j].history[0].v_valid,
					lanes[i].moving_objects[j].average_longitudinal_v.num_samples(),
					lanes[i].moving_objects[j].average_lateral_v.num_samples(),
					lanes[i].moving_objects[j].average_lateral_v.arithmetic_mean(),
					lanes[i].moving_objects[j].average_lateral_v.standard_deviation(),
					lanes[i].moving_objects[j].average_longitudinal_v.arithmetic_mean(),
					lanes[i].moving_objects[j].average_longitudinal_v.standard_deviation(),
//					lanes[i].moving_objects[j].v,
					lanes[i].moving_objects[j].history[0].moving_object_points.size());
		}
	}
	printf("\n");
}


static void
merge_in_lane_objects(lane_t *lane, double map_resolution)
{
    for (int i = 0; i < (int) lane->moving_objects.size(); i++)
    {
    	moving_object_t *moving_object_i = &(lane->moving_objects[i]);
    	for (int j = i + 1; j < (int) lane->moving_objects.size(); j++)
    	{
        	moving_object_t *moving_object_j = &(lane->moving_objects[j]);
    		if (near(moving_object_i, moving_object_j))
    		{
    			int num_samples_i = get_num_valid_samples(moving_object_i->history);
    			int num_samples_j = get_num_valid_samples(moving_object_j->history);

    			moving_object_t *mo_k;
    			moving_object_t *mo_l;
    			bool remove_j;
    			if (num_samples_i >= num_samples_j)
    			{
    				mo_k = moving_object_i;
    				mo_l = moving_object_j;
    				remove_j = true;
    			}
    			else
    			{
    				mo_k = moving_object_j;
    				mo_l = moving_object_i;
    				remove_j = false;
    			}

//    			printf("in lane: ");
    			merge(mo_k, mo_l, map_resolution);

    			if (remove_j)
    			{
//    				printf("erase in merge in lane: %d, ", lane->moving_objects[j].id);
    				lane->moving_objects.erase(lane->moving_objects.begin() + j);
    				j--;
    			}
    			else
    			{
//    				printf("erase in merge in lane: %d, ", lane->moving_objects[i].id);
    				lane->moving_objects.erase(lane->moving_objects.begin() + i);
    				i--;
    				break;
    			}
    		}
    	}
    }
}


static void
share_points_between_objects(vector<lane_t> &lanes, carmen_map_t *occupancy_map)
{
	// Salva os pontos antes de processo de cópia para não copiar duplicado (ida e volta)
	vector < vector <vector <carmen_position_t> > > all_mo_points;
	for (int lane_index = 0; lane_index < (int) lanes.size(); lane_index++)
	{
		lane_t *lane = &(lanes[lane_index]);
		vector <vector <carmen_position_t> > lane_mo_points;
	    for (int mo_index = 0; mo_index < (int) lane->moving_objects.size(); mo_index++)
	    {
	    	moving_object_t *moving_object = &(lane->moving_objects[mo_index]);
	    	lane_mo_points.push_back(moving_object->history[0].moving_object_points);
	    }
	    all_mo_points.push_back(lane_mo_points);
	}

	// Copia pontos de outros objetos quando próximos
	for (int lane_index = 0; lane_index < (int) lanes.size(); lane_index++)
	{
		lane_t *lane = &(lanes[lane_index]);
	    for (int mo_index = 0; mo_index < (int) lane->moving_objects.size(); mo_index++)
	    {
	    	moving_object_t *moving_object = &(lane->moving_objects[mo_index]);
			for (int i = 0; i < (int) lanes.size(); i++)
			{
				for (int j = 0; j < (int) lanes[i].moving_objects.size(); j++)
				{
					bool same_mo = (lane_index == i) && (mo_index == j);
					if (!same_mo && close_orientation(moving_object, &(lanes[i].moving_objects[j])) && near(moving_object, &(lanes[i].moving_objects[j])))
					{
//						printf("copy to %d (%.2lf) from %d (%.2lf), ",
//								moving_object->id, moving_object->pose.theta,
//								lanes[i].moving_objects[j].id, lanes[i].moving_objects[j].pose.theta);
						copy_points(moving_object, all_mo_points[i][j], occupancy_map->config.resolution);
					}
				}
			}
	    }
	}

	all_mo_points.clear();
}


bool
get_robot_lane_id_and_index_in_lane(int &robot_lane_id, int &robot_index_in_lane,
		carmen_route_planner_road_network_message *road_network)
{
	if (road_network->number_of_poses == 0)
		return (false);

	carmen_ackerman_traj_point_t robot_pose = road_network->poses[0];
	double min_distance = 100000000.0;
	for (int lane = 0; lane < road_network->number_of_nearby_lanes; lane++)
	{
		carmen_ackerman_traj_point_t *lane_poses = &(road_network->nearby_lanes[road_network->nearby_lanes_indexes[lane]]);
		for (int i = 0; i < road_network->nearby_lanes_sizes[lane]; i++)
		{
			double distance = DIST2D(robot_pose, lane_poses[i]);
			if (distance < min_distance)
			{
				robot_index_in_lane = i;
				robot_lane_id = road_network->nearby_lanes_ids[lane];
				if (distance == 0.0)
					return (true);

				min_distance = distance;
			}
		}
	}

	return (true);
}


carmen_ackerman_traj_point_t *
get_poses_ahead_lane_and_lane_size(int &size, carmen_route_planner_road_network_message *road_network, int robot_lane_id)
{
	int lane = 0;
	for (  ; lane < road_network->number_of_nearby_lanes; lane++)
	{
		if (road_network->nearby_lanes_ids[lane] == robot_lane_id)
			break;
	}

	size = road_network->nearby_lanes_sizes[lane];
	return (&(road_network->nearby_lanes[road_network->nearby_lanes_indexes[lane]]));
}


double
get_s_displacement(carmen_ackerman_traj_point_t *path, int i, int last_path_pose)
{
	double s_range = 0.0;
	for ( ; (i < (last_path_pose - 1)); i++)
		s_range += DIST2D(path[i], path[i + 1]);

	return (s_range);
}


double
simulate_moving_object_in_its_lane(moving_object_t *moving_object, carmen_ackerman_traj_point_t *lane_poses,
		int index_in_lane, int target_node_index_in_lane)
{
	double S = get_s_displacement(lane_poses, index_in_lane, target_node_index_in_lane);
	double t = S / fabs(moving_object->average_longitudinal_v.arithmetic_mean());

	return (t);
}


double
get_robot_acc(carmen_ackerman_traj_point_t pose)
{
	if (!behavior_selector_goal_list_message)
		return (0.0);

	int last_goal_list_size = behavior_selector_goal_list_message->size;
	if (last_goal_list_size == 0)
		return (0.0);

	carmen_ackerman_traj_point_t goal = behavior_selector_goal_list_message->goal_list[0];
	double S = DIST2D(pose, goal);
	double acc;
	if (S < 0.1)
		return (0.0);
	else
		acc = ((goal.v * goal.v) - (pose.v * pose.v)) / (2.0 * S);
	if (acc > maximum_acceleration_forward)
		acc = maximum_acceleration_forward;

	return (acc);
}


double
simulate_robot_in_its_lane(carmen_ackerman_traj_point_t *lane_poses, int index_in_lane, int target_node_index_in_lane)
{
	double a = get_robot_acc(lane_poses[index_in_lane]);
	double v = 0.0;
	if (localize_ackerman_globalpos_message)
		v = localize_ackerman_globalpos_message->v;

	double S = get_s_displacement(lane_poses, index_in_lane, target_node_index_in_lane);

	if ((a == 0.0) && (v != 0.0))
		return (S / v);
	else
	{
		double sigma = 2.0 * a * S + v * v;
		if (sigma >= 0.0)
		{
			double sqrt_sigma = sqrt(sigma);
			double t1 = -((sqrt_sigma + v) / a);
			double t2 = (sqrt_sigma - v) / a;

			if (t1 >= 0.0)
			{
				if (t1 < t2)
					return (t1);
				else
					return (t2);
			}
			else
				return (t2);
		}
	}

	return (1000.0);
}


bool
moving_object_arrives_first(moving_object_t *moving_object,
		carmen_route_planner_road_network_message *road_network,
		int moving_object_lane_index, int robot_lane_id, int moving_object_index_in_lane, int robot_index_in_lane,
		int target_node_index_in_robot_lane, int target_node_index_in_moving_object_lane)
{
	int lane_size;
	carmen_ackerman_traj_point_t *lane_poses = get_poses_ahead_lane_and_lane_size(lane_size, road_network, robot_lane_id);
	double robot_time_to_merge = simulate_robot_in_its_lane(lane_poses, robot_index_in_lane, target_node_index_in_robot_lane);

	lane_poses = &(road_network->nearby_lanes[moving_object_lane_index]);
	double moving_object_time_to_merge = simulate_moving_object_in_its_lane(moving_object, lane_poses, moving_object_index_in_lane,
			target_node_index_in_moving_object_lane);

	if ((robot_time_to_merge + 2.0) > moving_object_time_to_merge)
		return (true);
	else
		return (false);
}


int
moving_object_arrives_in_merge_in_front_of_robot(moving_object_t *moving_object,
		carmen_route_planner_road_network_message *road_network,
		int moving_object_lane_index, int robot_lane_id, int moving_object_index_in_lane, int robot_index_in_lane)
{
	int arrives_in_front = 0;
	carmen_route_planner_junction_t *moving_object_lane_merges = &(road_network->nearby_lanes_merges[road_network->nearby_lanes_merges_indexes[moving_object_lane_index]]);
	for (int i = 0; i < road_network->nearby_lanes_merges_sizes[moving_object_lane_index]; i++)
	{
		if ((moving_object_lane_merges[i].target_lane_id == robot_lane_id) &&
			(moving_object_index_in_lane < moving_object_lane_merges[i].index_of_node_in_current_lane) &&
			((moving_object_lane_merges[i].target_node_index_in_nearby_lane > robot_index_in_lane) ||
			 (moving_object_lane_merges[i].target_node_index_in_nearby_lane == -1)))
		{
			if (moving_object_arrives_first(moving_object, road_network, moving_object_lane_index, robot_lane_id,
					moving_object_index_in_lane, robot_index_in_lane, moving_object_lane_merges[i].target_node_index_in_nearby_lane,
					moving_object_lane_merges[i].index_of_node_in_current_lane))
			{
				arrives_in_front = 1;
				return (arrives_in_front);
			}
		}
	}

	return (arrives_in_front);
}


int
move_robot_index_in_lane_to_consider_robot_and_moving_object_dimensions(int robot_index_in_lane,
			int robot_lane_id, carmen_route_planner_road_network_message *road_network, double mo_length)
{
	int lane_size;
	carmen_ackerman_traj_point_t *lane_poses = get_poses_ahead_lane_and_lane_size(lane_size, road_network, robot_lane_id);
	double distance_to_move = distance_car_pose_car_front - mo_length / 2.0;
	if (distance_to_move >= 0.0)
	{
		for (int i = robot_index_in_lane; i < lane_size - 1; i++)
		{
			distance_to_move -= DIST2D(lane_poses[i], lane_poses[i + 1]);
			if (distance_to_move < 0.0)
				return (i);
		}
	}
	else
	{
		for (int i = robot_index_in_lane; i > 0; i--)
		{
			distance_to_move += DIST2D(lane_poses[i], lane_poses[i - 1]);
			if (distance_to_move > 0.0)
				return (i);
		}
	}

	return (0);
}


int
is_in_front(carmen_route_planner_road_network_message *road_network, int moving_object_lane_index, moving_object_t *moving_object,
		int robot_lane_id, int robot_index_in_lane, int robot_index_in_lane_moved)
{
	if (moving_object->lane_id == robot_lane_id)
	{
		if (moving_object->history[0].index > robot_index_in_lane_moved)
			return (1);
		else
			return (0);
	}
	else
	{
		if (moving_object_arrives_in_merge_in_front_of_robot(moving_object, road_network, moving_object_lane_index,
				robot_lane_id, moving_object->history[0].index, robot_index_in_lane))
			return (1);
		else
			return (0);
	}
}


int
get_moving_object_index_in_poses_ahead(carmen_route_planner_road_network_message *road_network, moving_object_t *moving_object,
		int robot_lane_id, int robot_index_in_lane, int robot_index_in_lane_moved, double mo_length)
{
	if ((moving_object->lane_id != robot_lane_id) || (moving_object->history[0].index <= robot_index_in_lane_moved))
		return (-1);

	int moving_object_index_in_poses_ahead = moving_object->history[0].index - robot_index_in_lane;
	if (moving_object_index_in_poses_ahead >= road_network->number_of_poses)
		return (-1);

	int lane_size;
	carmen_ackerman_traj_point_t *lane_poses = get_poses_ahead_lane_and_lane_size(lane_size, road_network, robot_lane_id);
	double distance_to_move = mo_length / 2.0;
	for (int i = moving_object_index_in_poses_ahead; i > 0; i--)
	{
		distance_to_move -= DIST2D(lane_poses[i], lane_poses[i + 1]);
		if (distance_to_move < 0.0)
			return (i);
	}

	return (0);
}


int
get_moving_object_lane_index(int lane_id, carmen_route_planner_road_network_message *road_network)
{
	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
    	if (lane_id == road_network->nearby_lanes_ids[i])
    		return (i);
    }

	carmen_die("Could not find lane_id in get_moving_object_lane_index()");

	return (-1);
}


static void
update_statistics(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network,
		carmen_map_t *occupancy_map)
{
	for (unsigned int lane_index = 0; lane_index < lanes.size(); lane_index++)
	{
		lane_t *lane = &(lanes[lane_index]);
		for (unsigned int i = 0; i < lane->moving_objects.size(); i++)
		{
			moving_object_t *moving_object = &(lane->moving_objects[i]);
			moving_object->lane_id = lane->lane_id;
			int moving_object_lane_index = get_moving_object_lane_index(lane->lane_id, road_network);
			int robot_lane_id;
			int robot_index_in_lane;
			if (get_robot_lane_id_and_index_in_lane(robot_lane_id, robot_index_in_lane, road_network))
			{
				int robot_index_in_lane_moved = move_robot_index_in_lane_to_consider_robot_and_moving_object_dimensions(robot_index_in_lane,
						robot_lane_id, road_network, moving_object->average_length.arithmetic_mean());
				moving_object->in_front = is_in_front(road_network, moving_object_lane_index, moving_object, robot_lane_id,
						robot_index_in_lane, robot_index_in_lane_moved);
				moving_object->index_in_poses_ahead = get_moving_object_index_in_poses_ahead(road_network, moving_object,
						robot_lane_id, robot_index_in_lane, robot_index_in_lane_moved, moving_object->average_length.arithmetic_mean());
			}
			else
			{
				moving_object->in_front = 0;
				moving_object->index_in_poses_ahead = -1;
			}
			update_object_statistics(moving_object, occupancy_map);
		}
	}
}


carmen_moving_objects_point_clouds_message *
obstacle_distance_mapper_datmo(carmen_route_planner_road_network_message *road_network,
		carmen_map_t &occupancy_map, carmen_map_server_offline_map_message *offline_map, double timestamp)
{
	if (!road_network)
		return (NULL);

//	print_road_network(road_network);

	static vector<lane_t> lanes;

	remove_lanes_absent_from_road_network(lanes, road_network);
	include_new_lanes_in_road_network(lanes, road_network);
	update_lanes(lanes, road_network);

	for (unsigned int lane_index = 0; lane_index < lanes.size(); lane_index++)
		track_moving_objects(&(lanes[lane_index]), &occupancy_map, offline_map, timestamp);

	for (unsigned int lane_index = 0; lane_index < lanes.size(); lane_index++)
		merge_in_lane_objects(&(lanes[lane_index]), occupancy_map.config.resolution);

	for (unsigned int lane_index = 0; lane_index < lanes.size(); lane_index++)
		detect_new_objects(lanes, lane_index, &occupancy_map, offline_map, timestamp);

	share_points_between_objects(lanes, &occupancy_map);

	update_statistics(lanes, road_network, &occupancy_map);

//	print_mo(lanes, timestamp);

	carmen_moving_objects_point_clouds_message *mo = fill_in_moving_objects_point_clouds_message(lanes, timestamp);

	return (mo);
}
