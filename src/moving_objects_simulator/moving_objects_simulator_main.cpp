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
int current_vector_index = 0;

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
	double x_pos, y_pos, x_pos2, y_pos2;
	double dist = 0.0;

	if(localize_ackerman_init_message->num_modes > 0)
	{
		x_pos = localize_ackerman_init_message->mean[0].x;
		y_pos = localize_ackerman_init_message->mean[0].y;
	}
	else
		return;

	current_vector_index = 0;

	x_pos2 = timestamp_moving_objects_list[current_vector_index].x_car;
	y_pos2 = timestamp_moving_objects_list[current_vector_index].y_car;

	dist = euclidean_distance(x_pos,y_pos,x_pos2,y_pos2);

	printf("%d dist: %lf\n",current_vector_index, dist);
	while((dist > 50.0) && (current_vector_index < timestamp_moving_objects_list.size()))
	{
		current_vector_index++;
		x_pos2 = timestamp_moving_objects_list[current_vector_index].x_car;
		y_pos2 = timestamp_moving_objects_list[current_vector_index].y_car;
		dist = euclidean_distance(x_pos, y_pos, x_pos2, y_pos2);
		printf("%d dist: %lf\n",current_vector_index, dist);
	}
	ok_to_publish = 1;
}


void
publish_moving_objects()
{

	if(ok_to_publish && (current_vector_index < timestamp_moving_objects_list.size()))
	{

		moving_objects_point_clouds_message.num_point_clouds = timestamp_moving_objects_list[current_vector_index].objects.size();
		moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));
		carmen_test_alloc(moving_objects_point_clouds_message.point_clouds);

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
			moving_objects_point_clouds_message.point_clouds[i].object_pose.x = timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_iara + timestamp_moving_objects_list[current_vector_index].objects[i].x_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.y = timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_iara + timestamp_moving_objects_list[current_vector_index].objects[i].y_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.z = 0.0;
			moving_objects_point_clouds_message.point_clouds[i].height = timestamp_moving_objects_list[current_vector_index].objects[i].height;
			moving_objects_point_clouds_message.point_clouds[i].length = timestamp_moving_objects_list[current_vector_index].objects[i].length;
			moving_objects_point_clouds_message.point_clouds[i].width= timestamp_moving_objects_list[current_vector_index].objects[i].width;
			moving_objects_point_clouds_message.point_clouds[i].geometric_model = geometric_model;
			moving_objects_point_clouds_message.point_clouds[i].model_features = get_obj_model_features(idMod);
			moving_objects_point_clouds_message.point_clouds[i].num_associated = timestamp_moving_objects_list[current_vector_index].objects[i].id;

			moving_objects_point_clouds_message.point_clouds[i].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
			moving_objects_point_clouds_message.point_clouds[i].points[0].x = timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_x_iara + timestamp_moving_objects_list[current_vector_index].objects[i].x_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].points[0].y = timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_obj - timestamp_moving_objects_list[current_vector_index].objects[i].pos_y_iara + timestamp_moving_objects_list[current_vector_index].objects[i].y_global_pos;
			moving_objects_point_clouds_message.point_clouds[i].points[0].z = 0.0;

			moving_objects_point_clouds_message.timestamp = carmen_get_time();
		}

		carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

		free(moving_objects_point_clouds_message.point_clouds);

		current_vector_index++;
	}

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

	initialize_objects_by_timestamp();

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

	carmen_ipc_addPeriodicTimer(0.1, (TIMER_HANDLER_TYPE) publish_moving_objects, NULL);

	carmen_ipc_dispatch();

	return (0);
}
