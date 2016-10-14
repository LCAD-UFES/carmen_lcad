/*********************************************************
	---  Moving Objects Simulator Module ---
**********************************************************/

#include "moving_objects_simulator.h"

static carmen_localize_ackerman_initialize_message localize_ackerman_init_message;
static carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;

char* input_filename;
FILE* input;
int ok_to_publish = 0;

std::vector<object_model_features_t> object_models;
int num_of_models;

std::vector<timestamp_moving_objects> timestamp_moving_objects_list;
std::map<int,moving_objects_by_id_t> moving_objects_by_id_map;

int current_vector_index = 0;

double previous_timestamp = 0.0;
double delta_time = 0.1;
int first = 1;
int start_pos = 0;

carmen_point_t initial_pose, actual_pose;

#define MIN_DIST 10.0
#define PUBLISH_BY_ID

void
find_start_position()
{

	start_pos = 1;
	double x_pos, y_pos, x_pos2, y_pos2;
	double dist = 0.0;

	x_pos = actual_pose.x;
	y_pos = actual_pose.y;

	current_vector_index = timestamp_moving_objects_list.size() - 1;

	x_pos2 = timestamp_moving_objects_list[current_vector_index].x_car;
	y_pos2 = timestamp_moving_objects_list[current_vector_index].y_car;

	dist = euclidean_distance(x_pos,y_pos,x_pos2,y_pos2);

	while((dist > 14.0) && (current_vector_index > 0))
	{
		current_vector_index--;
		x_pos2 = timestamp_moving_objects_list[current_vector_index].x_car;
		y_pos2 = timestamp_moving_objects_list[current_vector_index].y_car;
		dist = euclidean_distance(x_pos, y_pos, x_pos2, y_pos2);
	}
	ok_to_publish = 1;
}


void
update_publishing_flag()
{

	double x_pos, y_pos, dist;


	std::map<int,moving_objects_by_id_t>::iterator it;
	for(it = moving_objects_by_id_map.begin(); it != moving_objects_by_id_map.end(); it++)
	{
		for(int i = 0; i < (int) it->second.objects.size(); i++)
		{
			x_pos = it->second.objects[i].pos_x_obj - it->second.objects[i].pos_x_iara + it->second.objects[i].x_global_pos;
			y_pos = it->second.objects[i].pos_y_obj - it->second.objects[i].pos_y_iara + it->second.objects[i].y_global_pos;

			dist = euclidean_distance(actual_pose.x, actual_pose.y, x_pos, y_pos);

			if(dist < MIN_DIST && it->second.publishing == 0)
			{
				it->second.publishing = 1;
				it->second.index = 0;
				break;
			}
		}
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void static
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("moving_objects_simulator: disconnected.\n");

		exit(0);
	}
}


void
localize_ackerman_init_handler(carmen_localize_ackerman_initialize_message *localize_ackerman_init_message)
{

	if(localize_ackerman_init_message->num_modes > 0)
	{
		initial_pose.x = localize_ackerman_init_message->mean[0].x;
		initial_pose.y = localize_ackerman_init_message->mean[0].y;
	}
	else
		return;

}


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	actual_pose.x = msg->globalpos.x;
	actual_pose.y = msg->globalpos.y;

#ifdef PUBLISH_BY_ID
	update_publishing_flag();
#else

	double dist = 0.0;
	if(start_pos == 0)
	{
		dist = euclidean_distance(initial_pose.x, initial_pose.y, actual_pose.x, actual_pose.y);
		if(dist > 20.0)
		{
			find_start_position();
		}
	}
#endif
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
publish_moving_objects_by_timestamp()
{

	if(ok_to_publish && (current_vector_index < (int) timestamp_moving_objects_list.size()))
	{
		first = 0;
		moving_objects_point_clouds_message.num_point_clouds = timestamp_moving_objects_list[current_vector_index].objects.size();
		moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));
		carmen_test_alloc(moving_objects_point_clouds_message.point_clouds);

		previous_timestamp = timestamp_moving_objects_list[current_vector_index].timestamp;

		for(int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++)
		{
			int geometric_model = 0;
			int idMod = -1;
			if(strcmp("Car", timestamp_moving_objects_list[current_vector_index].objects[i].tipo) == 0)
			{
				geometric_model = 0;
				idMod = 0;
			}
			if(strcmp("Bike", timestamp_moving_objects_list[current_vector_index].objects[i].tipo) == 0)
			{
				geometric_model = 11;
				idMod = 1;
			}
			if(strcmp("Truck", timestamp_moving_objects_list[current_vector_index].objects[i].tipo) == 0)
			{
				geometric_model = 21;
				idMod = 2;
			}
			if(strcmp("Bus", timestamp_moving_objects_list[current_vector_index].objects[i].tipo) == 0)
			{
				geometric_model = 31;
				idMod = 3;
			}
			if(strcmp("Pedestrian", timestamp_moving_objects_list[current_vector_index].objects[i].tipo) == 0)
			{
				geometric_model = 41;
				idMod = 4;
			}


			moving_objects_point_clouds_message.point_clouds[i].r = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].g = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].b = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].point_size = 1;
			moving_objects_point_clouds_message.point_clouds[i].linear_velocity = timestamp_moving_objects_list[current_vector_index].objects[i].velocity_obj;
			moving_objects_point_clouds_message.point_clouds[i].orientation = carmen_normalize_theta(timestamp_moving_objects_list[current_vector_index].objects[i].orientation_obj);
			moving_objects_point_clouds_message.point_clouds[i].object_pose.x = (timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_iara) + timestamp_moving_objects_list[current_vector_index].x_car;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.y = (timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_iara) + timestamp_moving_objects_list[current_vector_index].y_car;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.z = 0.0;
			moving_objects_point_clouds_message.point_clouds[i].height = timestamp_moving_objects_list[current_vector_index].objects[i].height;
			moving_objects_point_clouds_message.point_clouds[i].length = timestamp_moving_objects_list[current_vector_index].objects[i].length;
			moving_objects_point_clouds_message.point_clouds[i].width= timestamp_moving_objects_list[current_vector_index].objects[i].width;
			moving_objects_point_clouds_message.point_clouds[i].geometric_model = geometric_model;
			moving_objects_point_clouds_message.point_clouds[i].model_features = get_obj_model_features(idMod);
			moving_objects_point_clouds_message.point_clouds[i].num_associated = timestamp_moving_objects_list[current_vector_index].objects[i].id;

			moving_objects_point_clouds_message.point_clouds[i].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
			moving_objects_point_clouds_message.point_clouds[i].points[0].x = (timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_iara) + timestamp_moving_objects_list[current_vector_index].x_car;
			moving_objects_point_clouds_message.point_clouds[i].points[0].y = (timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_iara) + timestamp_moving_objects_list[current_vector_index].y_car;
			moving_objects_point_clouds_message.point_clouds[i].points[0].z = 0.0;

			moving_objects_point_clouds_message.timestamp = carmen_get_time();
		}

		carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

		free(moving_objects_point_clouds_message.point_clouds);

		current_vector_index++;
	}

	return;
}


void
publish_moving_objects_by_id()
{

	int num_of_objects = 0;
	int i = 0;

	std::map<int,moving_objects_by_id_t>::iterator it;
	for(it = moving_objects_by_id_map.begin(); it != moving_objects_by_id_map.end(); it++)
	{
		if(it->second.publishing == 1 && it->second.index < (int) it->second.objects.size())
		{
			num_of_objects++;
		}
	}

	if(num_of_objects == 0){
		return;
	}

	// alocação da mensagem
	moving_objects_point_clouds_message.num_point_clouds = num_of_objects;
	moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));
	carmen_test_alloc(moving_objects_point_clouds_message.point_clouds);

	for(it = moving_objects_by_id_map.begin(); it != moving_objects_by_id_map.end(); it++)
	{
		if(it->second.publishing == 1 && it->second.index < (int) it->second.objects.size())
		{
			int geometric_model = 0;
			int idMod = -1;
			if(strcmp("Car", it->second.objects[it->second.index].tipo) == 0)
			{
				geometric_model = 0;
				idMod = 0;
			}
			if(strcmp("Bike", it->second.objects[it->second.index].tipo) == 0)
			{
				geometric_model = 11;
				idMod = 1;
			}
			if(strcmp("Truck", it->second.objects[it->second.index].tipo) == 0)
			{
				geometric_model = 21;
				idMod = 2;
			}
			if(strcmp("Bus", it->second.objects[it->second.index].tipo) == 0)
			{
				geometric_model = 31;
				idMod = 3;
			}
			if(strcmp("Pedestrian", it->second.objects[it->second.index].tipo) == 0)
			{
				geometric_model = 41;
				idMod = 4;
			}

			moving_objects_point_clouds_message.point_clouds[i].r = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].g = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].b = 1.0;
			moving_objects_point_clouds_message.point_clouds[i].point_size = 1;
			moving_objects_point_clouds_message.point_clouds[i].linear_velocity = it->second.objects[it->second.index].velocity_obj;
			moving_objects_point_clouds_message.point_clouds[i].orientation = carmen_normalize_theta(it->second.objects[it->second.index].orientation_obj);
			moving_objects_point_clouds_message.point_clouds[i].object_pose.x = (it->second.objects[it->second.index].pos_x_obj - it->second.objects[it->second.index].pos_x_iara) + it->second.objects[it->second.index].x_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.y = (it->second.objects[it->second.index].pos_y_obj - it->second.objects[it->second.index].pos_y_iara) + it->second.objects[it->second.index].y_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.z = 0.0;
			moving_objects_point_clouds_message.point_clouds[i].height = it->second.objects[it->second.index].height;
			moving_objects_point_clouds_message.point_clouds[i].length = it->second.objects[it->second.index].length;
			moving_objects_point_clouds_message.point_clouds[i].width= it->second.objects[it->second.index].width;
			moving_objects_point_clouds_message.point_clouds[i].geometric_model = geometric_model;
			moving_objects_point_clouds_message.point_clouds[i].model_features = get_obj_model_features(idMod);
			moving_objects_point_clouds_message.point_clouds[i].num_associated = it->second.objects[it->second.index].id;

			moving_objects_point_clouds_message.point_clouds[i].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
			moving_objects_point_clouds_message.point_clouds[i].points[0].x = (it->second.objects[it->second.index].pos_x_obj - it->second.objects[it->second.index].pos_x_iara) + it->second.objects[it->second.index].x_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].points[0].y = (it->second.objects[it->second.index].pos_y_obj - it->second.objects[it->second.index].pos_y_iara) + it->second.objects[it->second.index].y_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].points[0].z = 0.0;

			moving_objects_point_clouds_message.timestamp = carmen_get_time();

			it->second.index++;
			i++;
		}
	}

	carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

	free(moving_objects_point_clouds_message.point_clouds);

	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
void
initialize_objects_by_timestamp()
{

	moving_object_data moving_object;
	timestamp_moving_objects moving_objects_by_timestamp;

	double last_timestamp = 0;

	input = fopen(input_filename,"r");

	while(!feof(input))
	{
		//reads the parameters from the file
		fscanf(input,"%lf %lf %lf %d %[^ ] %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
				&moving_object.x_global_pos,
				&moving_object.y_global_pos,
				&moving_object.timestamp,
				&moving_object.id,
				moving_object.tipo,
				&moving_object.oclusion,
				&moving_object.alpha,
				&moving_object.height,
				&moving_object.width,
				&moving_object.length,
				&moving_object.pos_x_obj,
				&moving_object.pos_y_obj,
				&moving_object.l10,
				&moving_object.orientation_obj,
				&moving_object.l12,
				&moving_object.pos_x_iara,
				&moving_object.pos_y_iara,
				&moving_object.l15,
				&moving_object.orientation_iara,
				&moving_object.velocity_obj);

		if(moving_object.timestamp == last_timestamp)
		{
			moving_objects_by_timestamp.objects.push_back(moving_object);
		}
		else
		{
			timestamp_moving_objects_list.push_back(moving_objects_by_timestamp);
			moving_objects_by_timestamp.timestamp = moving_object.timestamp;
			moving_objects_by_timestamp.x_car = moving_object.x_global_pos;
			moving_objects_by_timestamp.y_car = moving_object.y_global_pos;
			moving_objects_by_timestamp.objects.clear();
			moving_objects_by_timestamp.objects.push_back(moving_object);
		}

		last_timestamp = moving_object.timestamp;
	}
	timestamp_moving_objects_list.push_back(moving_objects_by_timestamp);

	fclose(input);

	return;
}


void
intialize_objects_by_ids()
{
	moving_object_data moving_object;
	moving_objects_by_id_t moving_objects_by_id;

	input = fopen(input_filename,"r");

	while(!feof(input))
	{
		//reads the parameters from the file
		fscanf(input,"%lf %lf %lf %d %[^ ] %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
				&moving_object.x_global_pos,
				&moving_object.y_global_pos,
				&moving_object.timestamp,
				&moving_object.id,
				moving_object.tipo,
				&moving_object.oclusion,
				&moving_object.alpha,
				&moving_object.height,
				&moving_object.width,
				&moving_object.length,
				&moving_object.pos_x_obj,
				&moving_object.pos_y_obj,
				&moving_object.l10,
				&moving_object.orientation_obj,
				&moving_object.l12,
				&moving_object.pos_x_iara,
				&moving_object.pos_y_iara,
				&moving_object.l15,
				&moving_object.orientation_iara,
				&moving_object.velocity_obj);

		moving_objects_by_id_map[moving_object.id].objects.push_back(moving_object);
		moving_objects_by_id_map[moving_object.id].index = 0;
		moving_objects_by_id_map[moving_object.id].publishing = 0;
	}

	fclose(input);

	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char **argv)
{

	if (argc > 1)
		input_filename = argv[1];
	else
	{
		printf("Invalid filename\n");
		return 1;
	}

	set_object_models(object_models);

	num_of_models = object_models.size();

#ifdef PUBLISH_BY_ID
	intialize_objects_by_ids();
#else
	initialize_objects_by_timestamp();
#endif


	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Define messages that your module publishes */
	carmen_moving_objects_point_clouds_define_messages();

	memset(&localize_ackerman_init_message, 0, sizeof(carmen_localize_ackerman_initialize_message));
	/* Subscribe to sensor and filter messages */
	carmen_localize_ackerman_subscribe_initialize_message(&localize_ackerman_init_message, (carmen_handler_t) localize_ackerman_init_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

#ifdef PUBLISH_BY_ID
	carmen_ipc_addPeriodicTimer(0.1, (TIMER_HANDLER_TYPE) publish_moving_objects_by_id, NULL);
#else
	carmen_ipc_addPeriodicTimer(0.1, (TIMER_HANDLER_TYPE) publish_moving_objects_by_timestamp, NULL);
#endif
	carmen_ipc_dispatch();

	return (0);
}
