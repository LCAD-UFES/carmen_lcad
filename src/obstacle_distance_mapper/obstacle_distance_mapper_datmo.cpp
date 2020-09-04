#include <vector>
#include <carmen/carmen.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/collision_detection.h>
#include <prob_map.h>
#include "obstacle_distance_mapper_datmo.h"
#include "obstacle_distance_mapper_interface.h"

extern double min_moving_object_velocity;
extern double max_moving_object_velocity;
extern double obstacle_probability_threshold;

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

typedef struct
{
	int c;
	int id;
	double x;
	double y;
	double theta;
	double v;
	double average_width;
	double average_length;
} box_model_t;


int next_mo_id = 0;


int
get_num_valid_samples(moving_object_history_t *history)
{
	int sum = 0;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
		if (history[i].valid)
			sum++;

	return (sum);
}


int
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


int
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


lane_t *
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


void
shift_history(moving_object_t *moving_object)
{
	if (moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].v_valid)
		moving_object->average_v.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].pose.v);

	if (moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].valid)
	{
		moving_object->average_length.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].length);
		moving_object->average_width.remove_sample(moving_object->history[MOVING_OBJECT_HISTORY_SIZE - 1].width);
	}

	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0; i--)
		moving_object->history[i + 1] = moving_object->history[i];

	moving_object->history[0].valid = 0;
	moving_object->history[0].v_valid = 0;
}


bool
obstacle_detected(lane_t *lane, int i, carmen_map_t *occupancy_map)
{
	if ((i < 0) || (i >= (lane->size - 1)))
		carmen_die("Fatal error: (i < 0) || (i >= (lane->size - 1)) in obstacle_detected().");

	double lane_width = ROUTE_PLANNER_GET_LANE_WIDTH(lane->traffic_restrictions[i]);
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
		for (double d = -lane_width / 2.0; d < lane_width / 2.0; d += occupancy_map->config.resolution * 0.5)
		{
			double x = lane_x + d * cos(lane->lane_points[i].theta + M_PI / 2.0);
			double y = lane_y + d * sin(lane->lane_points[i].theta + M_PI / 2.0);

			// Move global path point coordinates to map coordinates
			int x_map_cell = (int) round((x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
			int y_map_cell = (int) round((y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
			if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
				continue;

			if (occupancy_map->map[x_map_cell][y_map_cell] > obstacle_probability_threshold)
				return (true);
		}
	}

	return (false);
}


int
predict_object_pose_index(moving_object_t *moving_object, lane_t *lane, carmen_map_t *occupancy_map, double timestamp)
{
	int index = moving_object->history[1].index;

	// Prediz o novo indice em função da velocidade atual
	double delta_t = timestamp - moving_object->history[1].timestamp;
	double displacement = delta_t * moving_object->v;
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

	// Procura pelo objeto móvel assumindo que a velocidade está dentro de certos limites
	int delta_index = 0;
	bool within_up_limit = true;
	bool within_down_limit = true;
	int found_index = -1;
	double forward_max_disp = delta_t * (moving_object->v + max_moving_object_velocity);
	double backward_max_disp = delta_t * (moving_object->v - max_moving_object_velocity);

	while (within_up_limit || within_down_limit)
	{
		int increased_index = index + delta_index;
		if ((increased_index >= (lane->size - 1)) ||
			(DIST2D(lane->lane_points[index], lane->lane_points[increased_index]) > forward_max_disp)) // isso é apenas uma aproximação em curvas
			within_up_limit = false;
		int decreased_index = index - delta_index;
		if ((decreased_index < 0) ||
			(DIST2D(lane->lane_points[index], lane->lane_points[decreased_index]) > backward_max_disp)) // isso é apenas uma aproximação em curvas
			within_down_limit = false;

		if (within_up_limit)
		{
			if ((increased_index > 0) && !lane->examined[increased_index] && obstacle_detected(lane, increased_index, occupancy_map))
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
			if ((decreased_index < (lane->size - 1)) && !lane->examined[decreased_index] && obstacle_detected(lane, decreased_index, occupancy_map))
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


int
get_last_valid_history(moving_object_t *moving_object)
{
	int last_valid_history;
	for (last_valid_history = 1; last_valid_history < MOVING_OBJECT_HISTORY_SIZE; last_valid_history++)
		if (moving_object->history[last_valid_history].valid)
			break;

	if (last_valid_history == MOVING_OBJECT_HISTORY_SIZE)
		return (-1);

	return (last_valid_history);
}


void
try_and_add_a_v_sample(moving_object_t *moving_object)
{
	int next_valid = -1;
	for (int i = 1; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (moving_object->history[i].valid)
		{
			next_valid = i;
			break;
		}
	}

	double delta_t = moving_object->history[0].timestamp - moving_object->history[next_valid].timestamp;
	if ((next_valid != -1) && (delta_t > 0.01))
	{
		double dist = DIST2D(moving_object->history[0].pose, moving_object->history[next_valid].pose);
		// distance in the direction of the lane: https://en.wikipedia.org/wiki/Vector_projection
		double angle = ANGLE2D(moving_object->history[next_valid].pose, moving_object->history[0].pose);
		double distance = dist * cos(angle - moving_object->history[0].pose.theta);
		moving_object->history[0].pose.v = distance / delta_t;
		moving_object->average_v.add_sample(moving_object->history[0].pose.v);
		moving_object->history[0].v_valid = 1;
	}
	else
	{
		moving_object->history[0].v_valid = 0;
		moving_object->history[0].pose.v = 0.0;
	}
}


void
update_object_statistics(moving_object_t *moving_object)
{
	try_and_add_a_v_sample(moving_object);

	moving_object->non_detection_count = 0;
	moving_object->pose = moving_object->history[0].pose;
	moving_object->average_length.add_sample(moving_object->history[0].length);
	moving_object->average_width.add_sample(moving_object->history[0].width);
	if (moving_object->average_v.num_samples() != 0)
		moving_object->v = moving_object->v + 0.2 * (moving_object->average_v.arithmetic_mean() - moving_object->v);
	else
		moving_object->v = 0.0;
}


void
compute_object_history_sample_width_and_length(moving_object_history_t *mo, double map_resolution)
{
	double width = 0.0;
	double length = 0.0;
	for (unsigned int i = 0; i < mo->moving_object_points.size(); i++)
	{
		double x = mo->moving_object_points[i].x - mo->pose.x;
		double y = mo->moving_object_points[i].y - mo->pose.y;
		double sin, cos;
		sincos(-mo->pose.theta, &sin, &cos);
		double xr = fabs(x * cos - y * sin);
		double yr = fabs(x * sin + y * cos);
		width = (yr > width)? yr: width;
		length = (xr > length)? xr: length;
	}

	double additional_size_due_to_map_discretization = map_resolution * M_SQRT2;
	mo->width = 2.0 * ((width > map_resolution)? width: map_resolution) + additional_size_due_to_map_discretization;
	mo->length = 2.0 * ((length > map_resolution)? length: map_resolution) + additional_size_due_to_map_discretization;
}


bool
get_object_points(vector <carmen_position_t> &moving_object_points, lane_t *lane, int i, bool forward, carmen_map_t *occupancy_map)
{
	double signal = (forward)? 1.0: -1.0;
	if ((i < 0) || (i + signal >= lane->size) || (i + signal < 0))
		carmen_die("Fatal error: (i < 0) || (i + signal >= lane->size) || (i + signal < 0) in get_object_points().");

	bool found = false;
	double lane_width = ROUTE_PLANNER_GET_LANE_WIDTH(lane->traffic_restrictions[i]);
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
		for (double d = -lane_width / 2.0; d < lane_width / 2.0; d += occupancy_map->config.resolution * 0.5)
		{
			double x = lane_x + d * cos(lane->lane_points[i].theta + M_PI / 2.0);
			double y = lane_y + d * sin(lane->lane_points[i].theta + M_PI / 2.0);

			// Move global path point coordinates to map coordinates
			int x_map_cell = (int) round((x - occupancy_map->config.x_origin) / occupancy_map->config.resolution);
			int y_map_cell = (int) round((y - occupancy_map->config.y_origin) / occupancy_map->config.resolution);
			if ((x_map_cell < 0 || x_map_cell >= occupancy_map->config.x_size) || (y_map_cell < 0 || y_map_cell >= occupancy_map->config.y_size))
				continue;

			if (occupancy_map->map[x_map_cell][y_map_cell] > obstacle_probability_threshold)
			{
				found = true;

				x = occupancy_map->config.resolution * round(x / occupancy_map->config.resolution);
				y = occupancy_map->config.resolution * round(y / occupancy_map->config.resolution);
				moving_object_points.push_back({x, y});
			}
		}
	}

	return (found);
}


void
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


bool
add_object_history_sample(moving_object_t *moving_object, int first_pose_index, lane_t *lane,
		carmen_map_t *occupancy_map, double timestamp, bool forward)
{
	int i = first_pose_index;
	vector <carmen_position_t> moving_object_points;
	memset(&(moving_object->history[0]), 0, sizeof(moving_object_history_t));

	bool found = false;
	if (forward)
	{
 		while ((i < (lane->size - 1)) && get_object_points(moving_object_points, lane, i, forward, occupancy_map))
		{
 			found = true;
			lane->examined[i] = true;
			i++;
		}
		i--;
	}
	else
	{
		while ((i > 0) && get_object_points(moving_object_points, lane, i, forward, occupancy_map))
		{
 			found = true;
			lane->examined[i] = true;
			i--;
		}
		i++;
	}

	if (found)
	{
		moving_object->history[0].valid = 1;
		moving_object->history[0].index = (first_pose_index + i) / 2;
		lane->examined[(first_pose_index + i) / 2] = true;
		get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object_points);
		moving_object->history[0].pose.theta = lane->lane_points[moving_object->history[0].index].theta;
		moving_object->history[0].moving_object_points = moving_object_points;
		compute_object_history_sample_width_and_length(&(moving_object->history[0]), occupancy_map->config.resolution);

		moving_object->history[0].timestamp = timestamp;

		return (true);
	}
	else
	{
		moving_object->history[0].valid = 0;

		return (false);
	}
}


bool
update(moving_object_t *moving_object, lane_t *lane, int index, carmen_map_t *occupancy_map, double timestamp)
{
	int last_valid_history = get_last_valid_history(moving_object);

	if (last_valid_history == -1)
		return (false);

	bool found = false;
	while ((index >= 0) && obstacle_detected(lane, index, occupancy_map))
		index--;
	index++;

	found = add_object_history_sample(moving_object, index, lane, occupancy_map, timestamp, true);

	if (found)
	{
		update_object_statistics(moving_object);
		return (true);
	}
	else
		return (false);
}


bool
track(moving_object_t *moving_object, lane_t *lane, carmen_map_t *occupancy_map, double timestamp)
{
	int index = predict_object_pose_index(moving_object, lane, occupancy_map, timestamp);
//	printf("index %d, id %d, ", index, moving_object->id);

	if (index != -1)
		return (update(moving_object, lane, index, occupancy_map, timestamp));
	else
		return (false);
}


void
estimate_new_state(moving_object_t *moving_object, lane_t *lane, double timestamp)
{
	int sample = get_last_valid_history_sample(moving_object->history);
	if (sample < 0)
		return;
	
	moving_object->history[0].moving_object_points = moving_object->history[sample].moving_object_points;
	moving_object->history[0].pose = moving_object->history[sample].pose; // para copiar v
	int index = moving_object->history[sample].index;
	int new_index = index;
	double delta_t = timestamp - moving_object->history[sample].timestamp;
	double displacement = delta_t * moving_object->v;
	if (fabs(displacement) != 0.0)
	{
		double signal = displacement / fabs(displacement);
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
			double disp = DIST2D(lane->lane_points[aux], lane->lane_points[new_index]);
			for (unsigned int i = 0; i < moving_object->history[0].moving_object_points.size(); i++)
			{
				moving_object->history[0].moving_object_points[i].x += signal * disp * cos(lane->lane_points[new_index].theta);
				moving_object->history[0].moving_object_points[i].y += signal * disp * sin(lane->lane_points[new_index].theta);
			}
			displacement_in_lane += disp;
			new_index = aux;
		}

		get_object_history_sample_mass_center(&(moving_object->history[0]), moving_object->history[0].moving_object_points);
	}
	moving_object->history[0].index = new_index;
	moving_object->history[0].width =  moving_object->history[sample].width;
	moving_object->history[0].length =  moving_object->history[sample].length;
	moving_object->pose = moving_object->history[0].pose;
}


void
track_moving_objects(lane_t *lane, carmen_map_t *occupancy_map, double timestamp)
{
    for (unsigned int i = 0; i < lane->moving_objects.size(); i++)
    {
    	moving_object_t *moving_object = &(lane->moving_objects[i]);
    	shift_history(moving_object);
    	bool found = track(moving_object, lane, occupancy_map, timestamp);
    	if (!found)
    	{
    		estimate_new_state(moving_object, lane, timestamp);
    		moving_object->non_detection_count++;
    		if (moving_object->non_detection_count > MAX_NON_DETECTION_COUNT)
    			lane->moving_objects.erase(lane->moving_objects.begin() + i);
    	}
    }
}


carmen_ackerman_traj_point_t
get_other_moving_object_pose_and_index(moving_object_t other_moving_object, int &other_moving_object_index, lane_t lane, double timestamp)
{
	carmen_ackerman_traj_point_t pose;
	double other_moving_object_timestamp = other_moving_object.history[0].timestamp;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (other_moving_object.history[i].valid)
		{
			pose = other_moving_object.history[i].pose;
			other_moving_object_timestamp = other_moving_object.history[i].timestamp;
			other_moving_object_index = other_moving_object.history[i].index;
			break;
		}
	}

	double displacement = other_moving_object.v * (timestamp - other_moving_object_timestamp);
	if (other_moving_object.v > 0)
	{
		while (displacement > 0.0)
		{
			if (other_moving_object_index < (lane.size + 1))
			{
				displacement -= DIST2D(lane.lane_points[other_moving_object_index], lane.lane_points[other_moving_object_index + 1]);
				other_moving_object_index++;
			}
			else
				carmen_die("Fatal error: other_moving_object_index >= (lane.size + 1) in get_other_moving_object_pose_and_index().");
		}
	}
	else
	{
		while (displacement < 0.0)
		{
			if (other_moving_object_index > 0)
			{
				displacement += DIST2D(lane.lane_points[other_moving_object_index], lane.lane_points[other_moving_object_index - 1]);
				other_moving_object_index--;
			}
			else
				carmen_die("Fatal error: other_moving_object_index <= 0 in get_other_moving_object_pose_and_index().");
		}
	}
	carmen_point_t displaced_position = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&lane.lane_points[other_moving_object_index], displacement);
	pose = lane.lane_points[other_moving_object_index]; // para copiar os outros campos
	pose.x = displaced_position.x;
	pose.y = displaced_position.y;

	return (pose);
}


moving_object_t *
get_near_moving_object_from_another_lane(moving_object_t *moving_object, vector<lane_t> &lanes, int lane_index)
{
	double moving_object_half_lane_width = ROUTE_PLANNER_GET_LANE_WIDTH(lanes[lane_index].traffic_restrictions[moving_object->history[0].index]) / 2.0;
	carmen_ackerman_traj_point_t moving_object_pose = moving_object->history[0].pose;
	for (unsigned int i = 0; i < lanes.size(); i++)
	{
		if (i != (unsigned int) lane_index)
		{
			for (unsigned int j = 0; j < lanes[i].moving_objects.size(); j++)
			{
				int other_moving_object_index;
				carmen_ackerman_traj_point_t other_moving_object = get_other_moving_object_pose_and_index(lanes[i].moving_objects[j],
						other_moving_object_index, lanes[i], moving_object->history[0].timestamp);
				double olther_moving_object_half_lane_width = ROUTE_PLANNER_GET_LANE_WIDTH(lanes[i].traffic_restrictions[other_moving_object_index]) / 2.0;
				if (DIST2D(moving_object_pose, other_moving_object) < (moving_object_half_lane_width + olther_moving_object_half_lane_width))
					return (&(lanes[i].moving_objects[j]));
			}
		}
	}

	return (NULL);
}


void
copy_history(moving_object_t *to, moving_object_t *from)
{
	double delta_angle = carmen_normalize_theta(from->history[0].pose.theta - to->history[0].pose.theta);
	to->v = from->v * cos(delta_angle);

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
				to->average_v.add_sample(to->history[i].pose.v);
			}
		}
	}
}


void
add_detected_obstacle(vector<lane_t> &lanes, int lane_index, int first_pose_index, carmen_map_t *occupancy_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	moving_object_t moving_object;
	memset(&(moving_object.history), 0, sizeof(moving_object.history));

	moving_object.id = next_mo_id++;
	if (next_mo_id > 999)
		next_mo_id = 0;

	bool found = add_object_history_sample(&moving_object, first_pose_index, lane, occupancy_map, timestamp, true);

	moving_object_t *moving_object_from_another_lane;
	if ((moving_object_from_another_lane = get_near_moving_object_from_another_lane(&moving_object, lanes, lane_index)) != NULL)
		copy_history(&moving_object, moving_object_from_another_lane);

	if (found)
		update_object_statistics(&moving_object);

	lane->moving_objects.push_back(moving_object);
}


void
detect_new_objects(vector<lane_t> &lanes, int lane_index, carmen_map_t *occupancy_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	for (int i = 0; i < (lane->size - 1); i++)
		if (!lane->examined[i] && obstacle_detected(lane, i, occupancy_map))
			add_detected_obstacle(lanes, lane_index, i, occupancy_map, timestamp);
}


void
fill_in_moving_objects_message_element(int k, carmen_moving_objects_point_clouds_message *message, box_model_t *box,
		vector <carmen_position_t> moving_object_points)
{
	message->point_clouds[k].r = 0.0;
	message->point_clouds[k].g = 0.0;
	message->point_clouds[k].b = 1.0;
	message->point_clouds[k].linear_velocity = box->v;
	message->point_clouds[k].orientation = box->theta;
	message->point_clouds[k].object_pose.x = box->x;
	message->point_clouds[k].object_pose.y = box->y;
	message->point_clouds[k].object_pose.z = 0.0;
	message->point_clouds[k].height = box->average_width;
	message->point_clouds[k].length = box->average_length;
	message->point_clouds[k].width = box->average_width;
	message->point_clouds[k].geometric_model = box->c;
	message->point_clouds[k].num_associated = box->id;
	object_model_features_t &model_features = message->point_clouds[k].model_features;
	model_features.model_id = box->c;

	switch (box->c)
	{
		case BUS:
			model_features.model_name = (char *) ("Bus");
			model_features.red = 1.0;
			model_features.green = 0.0;
			model_features.blue = 0.0;
			break;
		case CAR:
			model_features.model_name = (char *) ("Car");
			model_features.red = 0.5;
			model_features.green = 1.0;
			model_features.blue = 0.0;
			break;
		case BIKE:
			model_features.model_name = (char *) ("Bike");
			model_features.red = 0.5;
			model_features.green = 0.5;
			model_features.blue = 0.5;
			break;
		case PEDESTRIAN:
			model_features.model_name = (char *) ("Pedestrian");
			model_features.red = 0.0;
			model_features.green = 1.0;
			model_features.blue = 1.0;
			break;
	}
	model_features.geometry.length = box->average_length;
	model_features.geometry.width = box->average_width;
	model_features.geometry.height = box->average_width;

	if (moving_object_points.size() > 0)
	{
		message->point_clouds[k].point_size = moving_object_points.size();
		message->point_clouds[k].points = (carmen_vector_3D_t *) malloc(message->point_clouds[k].point_size * sizeof(carmen_vector_3D_t));
		for (unsigned int i = 0; i < moving_object_points.size(); i++)
			message->point_clouds[k].points[i] = {moving_object_points[i].x, moving_object_points[i].y, 0.0};
	}
	else
	{
		message->point_clouds[k].point_size = 0;
		message->point_clouds[k].points = NULL;
	}
}


carmen_moving_objects_point_clouds_message *
fill_in_moving_objects_point_clouds_message(vector<lane_t> lanes, double timestamp)
{
//	static carmen_moving_objects_point_clouds_message static_message;
//	static bool first_time = true;
//	if (first_time)
//	{
//		static_message.point_clouds = NULL;
//		first_time = false;
//	}
//
//	carmen_moving_objects_point_clouds_message *message = &static_message;
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
			box_model_t box;
			box.c = 'C'; 		// car
			box.average_length = lanes[i].moving_objects[j].average_length.arithmetic_mean();
			box.average_width = lanes[i].moving_objects[j].average_width.arithmetic_mean();
			box.id = lanes[i].moving_objects[j].id;
			box.x = lanes[i].moving_objects[j].pose.x;
			box.y = lanes[i].moving_objects[j].pose.y;
			box.v = lanes[i].moving_objects[j].v;
			box.theta = lanes[i].moving_objects[j].pose.theta;

			fill_in_moving_objects_message_element(mo_num, message, &box, lanes[i].moving_objects[j].history[0].moving_object_points);
			mo_num++;
		}
	}

	return (message);
}


void
remove_lanes_absent_from_road_network(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
	printf("remove_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
			road_network->number_of_nearby_lanes, (int) lanes.size());

    for (unsigned int i = 0; i < lanes.size(); i++)
    {
    	int lane_id = lanes[i].lane_id;
    	bool not_found = true;
		for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
		{
			if (lane_id == road_network->nearby_lanes_ids[i])
			{
				not_found = false;
				break;
			}
		}
		if (not_found)
			lanes.erase(lanes.begin() + i);
    }
}


void
include_new_lanes_in_road_network(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
	printf("include_new_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
			road_network->number_of_nearby_lanes, (int) lanes.size());

	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
    	int lane_id = road_network->nearby_lanes_ids[i];
    	bool not_found = true;
        for (unsigned int j = 0; j < lanes.size(); j++)
        {
        	printf("%d %d, ", lane_id, lanes[j].lane_id);
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
	printf("\n");
	fflush(stdout);
}


void
update_lanes(vector<lane_t> &lanes, carmen_route_planner_road_network_message *road_network)
{
	printf("update_lanes - road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()\n",
			road_network->number_of_nearby_lanes, (int) lanes.size());

	if (road_network->number_of_nearby_lanes != (int) lanes.size())
		printf("Error: road_network->number_of_nearby_lanes (%d) != lanes.size() (%d) in update_lanes()",
				road_network->number_of_nearby_lanes, (int) lanes.size());

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
	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
		printf("lane_id %d, size %d\n", road_network->nearby_lanes_ids[i], road_network->nearby_lanes_sizes[i]);

	printf("\n");
}


void
obstacle_distance_mapper_remove_moving_objects_from_occupancy_map(carmen_map_t *occupancy_map, carmen_moving_objects_point_clouds_message *moving_objects)
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
			printf("id %d, lane %d, index %d, valid %d, w_s %d, l_s %d, l %0.2lf, w %0.2lf, v_valid %d, v_s %d, v %0.2lf\n",
					lanes[i].moving_objects[j].id,
					i,
					get_last_valid_index(lanes[i].moving_objects[j].history),
					lanes[i].moving_objects[j].history[0].valid,
					lanes[i].moving_objects[j].average_width.num_samples(),
					lanes[i].moving_objects[j].average_length.num_samples(),
					lanes[i].moving_objects[j].average_length.arithmetic_mean(),
					lanes[i].moving_objects[j].average_width.arithmetic_mean(),
					lanes[i].moving_objects[j].history[0].v_valid,
					lanes[i].moving_objects[j].average_v.num_samples(),
					lanes[i].moving_objects[j].v);
		}
	}
	printf("\n");
}


static bool
near(moving_object_t mo_j, moving_object_t mo_k)
{
	for (unsigned int i = 0; i < mo_j.history[0].moving_object_points.size(); i++)
	{
		carmen_position_t point_j = mo_j.history[0].moving_object_points[i];
		for (unsigned int j = 0; j < mo_k.history[0].moving_object_points.size(); j++)
		{
			carmen_position_t point_k = mo_k.history[0].moving_object_points[j];
			if (DIST2D(point_j, point_k) < 1.0)
				return (true);
		}
	}
	return (false);
}


static void
merge(moving_object_t *mo_l, moving_object_t *mo_m, double map_resolution)
{
	int num_samples_l = get_num_valid_samples(mo_l->history);
	int num_samples_m = get_num_valid_samples(mo_m->history);

	moving_object_t *mo_j;
	moving_object_t *mo_k;
	if (num_samples_l > num_samples_m)
	{
		mo_j = mo_l;
		mo_k = mo_m;
	}
	else
	{
		mo_j = mo_m;
		mo_k = mo_l;
	}

	if (mo_j->history[0].valid && mo_k->history[0].valid)
	{
		int size_j = mo_j->history[0].moving_object_points.size();
		int size_k = mo_k->history[0].moving_object_points.size();
		mo_j->history[0].index = (mo_j->history[0].index * size_j + mo_k->history[0].index * size_k) / (size_j + size_k);

		mo_j->history[0].moving_object_points.insert(mo_j->history[0].moving_object_points.end(), mo_k->history[0].moving_object_points.begin(), mo_k->history[0].moving_object_points.end());
		get_object_history_sample_mass_center(&(mo_j->history[0]), mo_j->history[0].moving_object_points);

		mo_j->average_length.remove_sample(mo_j->history[0].length);
		mo_j->average_width.remove_sample(mo_j->history[0].width);
		compute_object_history_sample_width_and_length(&(mo_j->history[0]), map_resolution);
		mo_j->average_length.add_sample(mo_j->history[0].length);
		mo_j->average_width.add_sample(mo_j->history[0].width);

		if (mo_j->history[0].v_valid && mo_k->history[0].v_valid)
		{
			mo_j->average_v.remove_sample(mo_j->history[0].pose.v);
			mo_j->average_v.add_sample((mo_j->history[0].pose.v + mo_k->history[0].pose.v) / 2.0);
		}
		else if (mo_k->history[0].v_valid)
			mo_j->average_v.add_sample(mo_k->history[0].pose.v);
	}
	else if (!mo_j->history[0].valid && mo_k->history[0].valid)
	{
		mo_j->history[0] = mo_k->history[0];
		mo_j->average_length.add_sample(mo_j->history[0].length);
		mo_j->average_width.add_sample(mo_j->history[0].width);
		if (mo_k->history[0].v_valid)
			mo_j->average_v.add_sample(mo_k->history[0].pose.v);
	}
}


void
merge_old(moving_object_t *mo_l, moving_object_t *mo_m, double map_resolution)
{
	int num_samples_l = get_num_valid_samples(mo_l->history);
	int num_samples_m = get_num_valid_samples(mo_m->history);

	moving_object_t *mo_j;
	moving_object_t *mo_k;
	if (num_samples_l > num_samples_m)
	{
		mo_j = mo_l;
		mo_k = mo_m;
	}
	else
	{
		mo_j = mo_m;
		mo_k = mo_l;
	}

	mo_j->average_v.clear();
	mo_j->average_length.clear();
	mo_j->average_width.clear();
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (mo_j->history[i].valid && mo_k->history[i].valid)
		{
			int size_j = mo_j->history[i].moving_object_points.size();
			int size_k = mo_k->history[i].moving_object_points.size();
			mo_j->history[i].index = (mo_j->history[i].index * size_j + mo_k->history[i].index * size_k) / (size_j + size_k);

			mo_j->history[i].moving_object_points.insert(mo_j->history[i].moving_object_points.end(), mo_k->history[i].moving_object_points.begin(), mo_k->history[i].moving_object_points.end());
			get_object_history_sample_mass_center(&(mo_j->history[i]), mo_j->history[i].moving_object_points);
			compute_object_history_sample_width_and_length(&(mo_j->history[i]), map_resolution);

			mo_j->average_length.add_sample(mo_j->history[i].length);
			mo_j->average_width.add_sample(mo_j->history[i].width);

			if (mo_j->history[i].v_valid && mo_k->history[i].v_valid)
				mo_j->average_v.add_sample((mo_j->history[i].pose.v + mo_k->history[i].pose.v) / 2.0);
			else if (mo_j->history[i].v_valid)
				mo_j->average_v.add_sample(mo_j->history[i].pose.v);
			else if (mo_k->history[i].v_valid)
				mo_j->average_v.add_sample(mo_k->history[i].pose.v);
		}
		else if (!mo_j->history[i].valid && mo_k->history[i].valid)
		{
			mo_j->average_length.add_sample(mo_k->history[i].length);
			mo_j->average_width.add_sample(mo_k->history[i].width);

			mo_j->history[i] =  mo_k->history[i];
			if (mo_k->history[i].v_valid)
				mo_j->average_v.add_sample(mo_k->history[i].pose.v);
		}
	}
}


void
merge_nearby_objects(vector<lane_t> &lanes, double map_resolution)
{
	for (unsigned int i = 0; i < lanes.size(); i++)
	{
		for (unsigned int j = 0; j < lanes[i].moving_objects.size(); j++)
		{
			for (unsigned int k = j + 1; k < lanes[i].moving_objects.size(); k++)
			{
				if (near(lanes[i].moving_objects[j], lanes[i].moving_objects[k]))
				{
					merge(&(lanes[i].moving_objects[j]), &(lanes[i].moving_objects[k]), map_resolution);
					lanes[i].moving_objects.erase(lanes[i].moving_objects.begin() + k);
				}
			}
		}
	}
}


carmen_moving_objects_point_clouds_message *
obstacle_distance_mapper_datmo(carmen_route_planner_road_network_message *road_network,
		carmen_map_t &occupancy_map, double timestamp)
{
	if (!road_network)
		return (NULL);

//	print_road_network(road_network);

	static vector<lane_t> lanes;

	remove_lanes_absent_from_road_network(lanes, road_network);
	include_new_lanes_in_road_network(lanes, road_network);
	update_lanes(lanes, road_network);

	for (unsigned int i = 0; i < lanes.size(); i++)
		track_moving_objects(&(lanes[i]), &occupancy_map, timestamp);

	for (unsigned int lane_index = 0; lane_index < lanes.size(); lane_index++)
		detect_new_objects(lanes, lane_index, &occupancy_map, timestamp);

	merge_nearby_objects(lanes, occupancy_map.config.resolution);

	print_mo(lanes, timestamp);

	carmen_moving_objects_point_clouds_message *mo = fill_in_moving_objects_point_clouds_message(lanes, timestamp);

	return (mo);
}

