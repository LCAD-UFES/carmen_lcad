#include <vector>
#include <carmen/carmen.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/collision_detection.h>
#include "behavior_selector.h"

//
// No codigo abaixo, a deteccao e o traqueamento de objetos moveis eh feita apenas nas lanes do road map contido em set_of_paths
// e preechido pelo route planner.
//
// INICIALIZACAO
//
//  Para realizar a deteccao e o traqueamento, este codigo mantem uma lista de lanes que eh composta de uma lista de objetos moveis
//  encontrados em cada lane.
//
//  Primeiramente, a lista de lanes eh atualizada com as lanes no set_of_paths recebido. Lanes jah presentes na lista de lanes
//  sao atualizadas com o conteudo de set_of_paths. Lanes presentes em set_of_paths e nao presentes na lista sao incluidas, e
//  lanes na lista nao presentes em set_of_paths sao removidas da lista.
//
// TRAQUEAMENTO
//
//  Apos a inicializacao, a lista de listas eh examinada para o traqueamento dos objetos jah detectados em cada lane em ciclos anteriores.
//  Este traqueamento eh feito fazendo uma predicao de para onde os objetos de cada lane se moveram e examinando se
//  os objetos estao lah. Um vetor do tamanho de cada lane eh mantido para marcar as posicoes da lane jah visitadas e, deste modo,
//  durante a deteccao (abaixo), evitar a redeteccao por engano. Estatisticas dos objetos sao computadas
//  neste processo de traqueamento. Objetos não reencontrados no processo recebem incremento de contagem para remocao.
//
// DETECCAO
//
//  A deteccao de objetos em uma lane eh feita examinando em sequencia cada pose nao examinada no processo de traqueamento e checando se,
//  nesta pose, existe um obstaculo ateh a uma distancia igual aa metada da largura da lane.
//
//  Detectado um objeto, posicoes subsequentes na lane sao examinadas para delimitar o tamanho e eventuais outras caracteristicas do objeto.
//  Listas de objetos moveis de outras lanes são tambem examinadas para ver se o objeto nao estah mudando para esta lane.
//  Se este for o caso, as estatisticas acumuladas na outra lane sao copiada para esta.
//
// FINALIZACAO
//
//  Objetos com contagem elevada para remoccao sao removidos da lista de listas.
//  Os objetos moveis traqueados sao devolvidos para o sistema em uma mensagem carmen_moving_objects_point_clouds_message.
//

#define MOVING_OBJECT_HISTORY_SIZE 20

using namespace std;

typedef struct
{
	bool valid;
	carmen_ackerman_traj_point_t pose;
	int index;
	double width;
	double length;
	double timestamp;
} moving_object_history_t;

typedef struct
{
	int id;
	moving_object_history_t history[MOVING_OBJECT_HISTORY_SIZE];
	double v;
	double width;
	double length;
	carmen_ackerman_traj_point_t pose;
	double pose_std; // Standard deviation in meters
	int non_detection_count;
} moving_object_t;

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
	double width;
	double length;
} box_model_t;


int next_mo_id = 0;


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
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0; i--)
		moving_object->history[i + 1] = moving_object->history[i];

	moving_object->history[0].valid = false;
}


bool
obstacle_detected(lane_t *lane, int i, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	carmen_point_t global_point = {lane->lane_points[i].x, lane->lane_points[i].y, lane->lane_points[i].theta};
	double distance_to_object = carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&global_point, distance_map);
	if (distance_to_object < (ROUTE_PLANNER_GET_LANE_WIDTH(lane->traffic_restrictions[i]) / 2.0))
		return (true);
	else
		return (false);
}


int
predict_object_pose_index(moving_object_t *moving_object, lane_t *lane, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int index = moving_object->history[1].index;
	int delta_index = 0;
	bool within_up_limit = true;
	bool within_down_limit = true;
	int found_index = -1;
	while (within_up_limit || within_down_limit)
	{
		int increased_index = index + delta_index;
		if (increased_index >= lane->size)
			within_up_limit = false;
		int decreased_index = index - delta_index;
		if (decreased_index < 0)
			within_down_limit = false;

		if (within_up_limit)
		{
			if (!lane->examined[increased_index] && obstacle_detected(lane, increased_index, distance_map))
			{
				lane->examined[increased_index] = true;
				found_index = increased_index;
				break;
			}
			lane->examined[increased_index] = true;
		}
		if (within_down_limit)
		{
			if (!lane->examined[decreased_index] && obstacle_detected(lane, decreased_index, distance_map))
			{
				lane->examined[decreased_index] = true;
				found_index = decreased_index;
				break;
			}
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
update_object_statistics(moving_object_t *moving_object)
{
	double num_valid, v_valid, length, width, v;
	num_valid = v_valid = length = width = v = 0.0;
	int reference_i = -1;
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE; i++)
	{
		if (moving_object->history[i].valid)
		{
			length += moving_object->history[i].length;
			width += moving_object->history[i].width;
			num_valid += 1.0;

			if (reference_i == -1)
				reference_i = i;

			double delta_t = moving_object->history[reference_i].timestamp - moving_object->history[i].timestamp;
			double dist = DIST2D(moving_object->history[reference_i].pose, moving_object->history[i].pose);
			if (delta_t > 0.01)
			{
				// distance in the direction of the lane: https://en.wikipedia.org/wiki/Vector_projection
				double angle = ANGLE2D(moving_object->history[i].pose, moving_object->history[reference_i].pose);
				double distance = dist * cos(angle - moving_object->history[reference_i].pose.theta);
				moving_object->history[reference_i].pose.v = distance / delta_t;
				v += moving_object->history[reference_i].pose.v;
				v_valid += 1.0;

				reference_i = i;
			}
			else
				moving_object->history[i].pose.v = 0.0;
		}
	}

	printf("**id %d, v %lf, x %lf, y %lf, count %d\n",
				moving_object->id,
				moving_object->v, moving_object->pose.x, moving_object->pose.y,
				moving_object->non_detection_count);

	printf("num_valid %lf, v_valid %lf\n", moving_object->history[0].pose.x, moving_object->history[0].pose.y);

	moving_object->non_detection_count = 0;
	moving_object->pose = moving_object->history[0].pose;

	printf("***id %d, v %lf, x %lf, y %lf, count %d\n",
				moving_object->id,
				moving_object->v, moving_object->pose.x, moving_object->pose.y,
				moving_object->non_detection_count);

	moving_object->pose_std = 10.0;
	moving_object->length = length / num_valid;
	moving_object->width = width / num_valid;
	if (v_valid != 0.0)
		moving_object->v = moving_object->v + 0.1 * ((v / v_valid) - moving_object->v);
	else
		moving_object->v = 0.0;
}


void
add_object_history_sample(moving_object_t *moving_object, int first_pose_index, lane_t *lane,
		carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp, bool forward)
{
	int i = first_pose_index;
	if (forward)
	{
		while ((i < lane->size) && obstacle_detected(lane, i, distance_map))
		{
			lane->examined[i] = true;
			i++;
		}
		i--;
	}
	else
	{
		while ((i >= 0) && obstacle_detected(lane, i, distance_map))
		{
			lane->examined[i] = true;
			i--;
		}
		i++;
	}
	int object_index = (first_pose_index + i) / 2;
	moving_object->history[0].pose = lane->lane_points[object_index];
	moving_object->history[0].index = object_index;
	double length = DIST2D(lane->lane_points[i], lane->lane_points[first_pose_index]);
	if (length < 2.0)
		length = 2.0;
	moving_object->history[0].length = length;
	moving_object->history[0].width = 2.0;
	moving_object->history[0].valid = true;
	moving_object->history[0].timestamp = timestamp;
}


bool
update(moving_object_t *moving_object, lane_t *lane, int index, carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp)
{
	int last_valid_history = get_last_valid_history(moving_object);

	if (last_valid_history == -1)
		return (false);

	if (index == moving_object->history[last_valid_history].index)
	{
		while ((index >= 0) && obstacle_detected(lane, index, distance_map))
			index--;
		index++;
		add_object_history_sample(moving_object, index, lane, distance_map, timestamp, true);
	}
	else if (index > moving_object->history[last_valid_history].index)
		add_object_history_sample(moving_object, index, lane, distance_map, timestamp, true);
	else
		add_object_history_sample(moving_object, index, lane, distance_map, timestamp, false);

	update_object_statistics(moving_object);

	return (true);
}


bool
track(moving_object_t *moving_object, lane_t *lane, carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp)
{
	int index = predict_object_pose_index(moving_object, lane, distance_map);
	if (index != -1)
		return (update(moving_object, lane, index, distance_map, timestamp));
	else
		return (false);
}


void
track_moving_objects(lane_t *lane, carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp)
{
    for (unsigned int i = 0; i < lane->moving_objects.size(); i++)
    {
    	moving_object_t *moving_object = &(lane->moving_objects[i]);
    	shift_history(moving_object);
    	bool found = track(moving_object, lane, distance_map, timestamp);
    	if (!found)
    	{
    		moving_object->non_detection_count++;
//    		printf("non_detection_count %d\n", moving_object->non_detection_count);
    		if (moving_object->non_detection_count > 10)
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
	for (int i = 1; i < MOVING_OBJECT_HISTORY_SIZE; i++) // A history[0] nao deve ser copiada pois jah contem a info mais recente
		to->history[i] = from->history[i];
}


void
add_detected_obstacle(vector<lane_t> &lanes, int lane_index, int first_pose_index, carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	moving_object_t moving_object;
	memset(&(moving_object.history), 0, sizeof(moving_object.history));

	moving_object.id = next_mo_id++;
	if (next_mo_id > 999)
		next_mo_id = 0;

	add_object_history_sample(&moving_object, first_pose_index, lane, distance_map, timestamp, true);

	moving_object_t *moving_object_from_another_lane;
	if ((moving_object_from_another_lane = get_near_moving_object_from_another_lane(&moving_object, lanes, lane_index)) != NULL)
		copy_history(&moving_object, moving_object_from_another_lane);

	update_object_statistics(&moving_object);

	lane->moving_objects.push_back(moving_object);
}


void
detect_new_objects(vector<lane_t> &lanes, int lane_index, carmen_obstacle_distance_mapper_map_message *distance_map, double timestamp)
{
	lane_t *lane = &(lanes[lane_index]);
	for (int i = 0; i < lane->size; i++)
		if (!lane->examined[i] && obstacle_detected(lane, i, distance_map))
			add_detected_obstacle(lanes, lane_index, i, distance_map, timestamp);
}


void
fill_in_moving_objects_message_element(int k, carmen_moving_objects_point_clouds_message *message, box_model_t *box)
{
	message->point_clouds[k].r = 0.0;
	message->point_clouds[k].g = 0.0;
	message->point_clouds[k].b = 1.0;
	message->point_clouds[k].linear_velocity = box->v;
	message->point_clouds[k].orientation = box->theta;
	message->point_clouds[k].object_pose.x = box->x;
	message->point_clouds[k].object_pose.y = box->y;
	message->point_clouds[k].object_pose.z = 0.0;
	message->point_clouds[k].height = box->width;
	message->point_clouds[k].length = box->length;
	message->point_clouds[k].width = box->width;
	message->point_clouds[k].geometric_model = box->c;
	message->point_clouds[k].point_size = 0;
	message->point_clouds[k].points = NULL;
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
	model_features.geometry.length = box->length;
	model_features.geometry.width = box->width;
	model_features.geometry.height = box->width;
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
			box.length = lanes[i].moving_objects[j].length;
			box.width = lanes[i].moving_objects[j].width;
			box.id = lanes[i].moving_objects[j].id;
			box.x = lanes[i].moving_objects[j].pose.x;
			box.y = lanes[i].moving_objects[j].pose.y;
			box.v = lanes[i].moving_objects[j].v;
			printf("  id %d, v %lf, x %lf, y %lf, count %d\n", box.id, box.v, box.x, box.y, lanes[i].moving_objects[j].non_detection_count);
			fflush(stdout);
			box.theta = lanes[i].moving_objects[j].pose.theta;

			fill_in_moving_objects_message_element(mo_num, message, &box);
			mo_num++;
		}
	}

	return (message);
}


void
remove_lanes_absent_from_road_network(vector<lane_t> &lanes, carmen_frenet_path_planner_set_of_paths *set_of_paths)
{
    for (unsigned int i = 0; i < lanes.size(); i++)
    {
    	int lane_id = lanes[i].lane_id;
    	bool not_found = true;
		for (int i = 0; i < set_of_paths->number_of_nearby_lanes; i++)
		{
			if (lane_id == set_of_paths->nearby_lanes_ids[i])
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
include_new_lanes_in_road_network(vector<lane_t> &lanes, carmen_frenet_path_planner_set_of_paths *set_of_paths)
{
	for (int i = 0; i < set_of_paths->number_of_nearby_lanes; i++)
	{
    	int lane_id = set_of_paths->nearby_lanes_ids[i];
    	bool not_found = true;
        for (unsigned int j = 0; j < lanes.size(); j++)
        {
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
			new_lane.size = set_of_paths->nearby_lanes_sizes[i];
			new_lane.lane_points = &(set_of_paths->nearby_lanes[set_of_paths->nearby_lanes_indexes[i]]);
			new_lane.traffic_restrictions = &(set_of_paths->traffic_restrictions[set_of_paths->nearby_lanes_indexes[i]]);
			vector<bool> examined(new_lane.size, false);
			new_lane.examined = examined;

			lanes.push_back(new_lane);
		}
    }
}


void
update_lanes(vector<lane_t> &lanes, carmen_frenet_path_planner_set_of_paths *set_of_paths)
{
	for (int i = 0; i < set_of_paths->number_of_nearby_lanes; i++)
	{
    	lane_t *lane = get_lane(lanes, set_of_paths->nearby_lanes_ids[i]);
    	lane->size = set_of_paths->nearby_lanes_sizes[i];
    	lane->lane_points = &(set_of_paths->nearby_lanes[set_of_paths->nearby_lanes_indexes[i]]);
		vector<bool> examined(lane->size, false);
		lane->examined = examined;
    	lane->traffic_restrictions = &(set_of_paths->traffic_restrictions[set_of_paths->nearby_lanes_indexes[i]]);
    }
}


carmen_moving_objects_point_clouds_message *
behavior_selector_moving_objects_tracking(carmen_frenet_path_planner_set_of_paths *set_of_paths,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if (1)//(!set_of_paths)
		return (NULL);

	static vector<lane_t> lanes;

	remove_lanes_absent_from_road_network(lanes, set_of_paths);
	include_new_lanes_in_road_network(lanes, set_of_paths);
	update_lanes(lanes, set_of_paths);

	if (lanes[0].moving_objects.size() > 0)
		printf("* id %d, v %lf, x %lf, y %lf, count %d\n",
				lanes[0].moving_objects[0].id,
				lanes[0].moving_objects[0].v, lanes[0].moving_objects[0].pose.x, lanes[0].moving_objects[0].pose.y,
				lanes[0].moving_objects[0].non_detection_count);

	for (unsigned int i = 0; i < lanes.size(); i++)
		track_moving_objects(&(lanes[i]), distance_map, distance_map->timestamp);

	for (unsigned int i = 0; i < lanes.size(); i++)
		detect_new_objects(lanes, i, distance_map, distance_map->timestamp);

	carmen_moving_objects_point_clouds_message *mo = fill_in_moving_objects_point_clouds_message(lanes, distance_map->timestamp);

	return (mo);
}

