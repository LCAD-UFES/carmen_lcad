/*********************************************************
	---  Moving Objects Simulator Module ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <vector>


static carmen_localize_ackerman_initialize_message localize_ackerman_init_message;
static carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;

char* input_filename;
static FILE* input;
static int ok_to_publish = 0;
std::vector<object_model_features_t> object_models;
int num_of_models;


double
euclidean_distance(double x1, double y1, double x2, double y2)
{
	double dist = sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );

	return dist;
}


void
clear_obj_model_features(object_model_features_t &obj_model)
{
	obj_model.model_id = -1;
	obj_model.model_name = (char *)"";
	obj_model.geometry.width = 0.0;
	obj_model.geometry.length = 0.0;
	obj_model.geometry.height = 0.0;
	obj_model.red = 0.0;
	obj_model.green = 0.0;
	obj_model.blue = 0.0;
}


void
set_model(object_model_features_t &obj_model, int model_id, char *model_type, double width, double length, double height,
		double red, double green, double blue)
{
	obj_model.model_id = model_id;
	obj_model.model_name = model_type;
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
}


object_model_features_t
get_obj_model_features(int model_id)
{
	object_model_features_t obj_model;

	if (model_id >= 0 && model_id < int(object_models.size()))
		obj_model = object_models[model_id];
	else
		clear_obj_model_features(obj_model);

	return obj_model;
}


void
set_object_models(std::vector<object_model_features_t> &obj_models)
{
	object_model_features_t obj_class;

	/* 0) sedan */
	set_model(obj_class, 0, (char *)"car", 1.8, 4.4, 1.4, 1.0, 0.0, 0.8);
	obj_models.push_back(obj_class);

	/* 11) bike/motorbike 1 */
	set_model(obj_class, 11, (char *)"bike", 0.7, 2.2, 1.4, 0.0, 1.0, 1.0);
	obj_models.push_back(obj_class);

	/* 21) small truck */
	set_model(obj_class, 21, (char *)"truck", 2.2, 6.8, 2.6, 0.5, 0.5, 1.0);
	obj_models.push_back(obj_class);

	/* 31) bus (average estimative) */
	set_model(obj_class, 31, (char *)"bus", 2.9, 12.6, 3.5, 1.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	/* 41) pedestrian 1 */
	set_model(obj_class, 41, (char *)"pedestrian", 0.6, 0.6, 1.7, 0.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
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
	double x_pos, y_pos, x_pos2, y_pos2;
	double dist = 0.0;
	char lixo[256];
	if(localize_ackerman_init_message->num_modes > 0)
	{
		x_pos = localize_ackerman_init_message->mean[0].x;
		y_pos = localize_ackerman_init_message->mean[0].y;
	}
	else
		return;

	if (input != NULL)
	{
		fclose(input);
	}
	input = fopen(input_filename,"r");

	fscanf(input,"%lf %lf %[^\n]\n", &x_pos2, &y_pos2, lixo);

	dist = euclidean_distance(x_pos, y_pos, x_pos2, y_pos2);
	while(dist > 50.0 && !feof(input))
	{
		fscanf(input,"%lf %lf %[^\n]\n", &x_pos2, &y_pos2, lixo);
		dist = euclidean_distance(x_pos, y_pos, x_pos2, y_pos2);
	}
	ok_to_publish = 1;
}

void
publish_moving_objects()
{

	if(ok_to_publish)
	{
		double x_global_pos;
		double y_global_pos;
		double timestamp, new_timestamp;
		int id;
		char tipo[10];
		int oclusion;
		double alpha;
		double height;
		double width;
		double length;
		double pos_x_obj;
		double pos_y_obj;
		double l10;
		double orientation_obj;
		int l12;
		double pos_x_iara;
		double pos_y_iara;
		double l15;
		double orientation_iara;
		double velocity_obj;




		fscanf(input,"%lf %lf %lf %d %[^ ] %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
				&x_global_pos, &y_global_pos, &timestamp,
				&id, tipo, &oclusion, &alpha, &height, &width, &length, &pos_x_obj, &pos_y_obj, &l10,
				&orientation_obj, &l12, &pos_x_iara, &pos_y_iara, &l15, &orientation_iara, &velocity_obj);
		//printf("%lf %lf %lf %d %s %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
//				x_global_pos, y_global_pos, timestamp,
//				id, tipo, oclusion, alpha, height, width, length, pos_x_odom, pos_y_odom, l10,
//				orientation_obj, l12, pos_x, pos_y, l15, orientation_iara, velocity_obj);

		int geometric_model = 0;
		int idMod;
		if(strcmp("Car", tipo) == 0)
		{
			geometric_model = 0;
			idMod = 0;
		}
		if(strcmp("Bike", tipo) == 0)
		{
			geometric_model = 11;
			idMod = 1;
		}
		if(strcmp("Truck", tipo) == 0)
		{
			geometric_model = 21;
			idMod = 2;
		}
		if(strcmp("Bus", tipo) == 0)
		{
			geometric_model = 31;
			idMod = 3;
		}
		if(strcmp("Pedestrian", tipo) == 0)
		{
			geometric_model = 41;
			idMod = 4;
		}

		moving_objects_point_clouds_message.num_point_clouds = 1;
		moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(1 * sizeof(t_point_cloud_struct)));
		carmen_test_alloc(moving_objects_point_clouds_message.point_clouds);


		moving_objects_point_clouds_message.point_clouds[0].r = 1.0;
		moving_objects_point_clouds_message.point_clouds[0].g = 1.0;
		moving_objects_point_clouds_message.point_clouds[0].b = 1.0;
		moving_objects_point_clouds_message.point_clouds[0].point_size = 1;
		moving_objects_point_clouds_message.point_clouds[0].linear_velocity = velocity_obj;
		moving_objects_point_clouds_message.point_clouds[0].orientation = carmen_normalize_theta(orientation_obj);
		moving_objects_point_clouds_message.point_clouds[0].object_pose.x = pos_x_obj - pos_x_iara + x_global_pos;// + it->car_global_pose.position.x;//it->centroid[0] + it->car_global_pose.position.x;
		moving_objects_point_clouds_message.point_clouds[0].object_pose.y = pos_y_obj - pos_y_iara + y_global_pos;// + it->car_global_pose.position.y;//it->centroid[1] + it->car_global_pose.position.y;
		moving_objects_point_clouds_message.point_clouds[0].object_pose.z = 0.0;
		moving_objects_point_clouds_message.point_clouds[0].height = height;
		moving_objects_point_clouds_message.point_clouds[0].length = length;
		moving_objects_point_clouds_message.point_clouds[0].width= width;
		moving_objects_point_clouds_message.point_clouds[0].geometric_model = geometric_model;
		moving_objects_point_clouds_message.point_clouds[0].model_features = get_obj_model_features(idMod);
		moving_objects_point_clouds_message.point_clouds[0].num_associated = id;

		moving_objects_point_clouds_message.point_clouds[0].points = (carmen_vector_3D_t *) malloc(1 * sizeof(carmen_vector_3D_t));
		moving_objects_point_clouds_message.point_clouds[0].points[0].x = pos_x_obj - pos_x_iara + x_global_pos;
		moving_objects_point_clouds_message.point_clouds[0].points[0].y = pos_y_obj - pos_y_iara + y_global_pos;
		moving_objects_point_clouds_message.point_clouds[0].points[0].z = 0.0;

		moving_objects_point_clouds_message.timestamp = carmen_get_time();

		carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

		free(moving_objects_point_clouds_message.point_clouds);
//		do
//		{
//			fscanf(input,"%lf %lf %lf %d %[^ ] %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
//				&x_global_pos, &y_global_pos, &new_timestamp,
//				&id, tipo, &oclusion, &alpha, &height, &width, &length, &pos_x_odom, &pos_y_odom, &l10,
//				&orientation_obj, &l12, &pos_x, &pos_y, &l15, &orientation_iara, &velocity_obj);
//		}
//		while(new_timestamp == timestamp);

	}
	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
static int
read_parameters(int argc, char **argv)
{
	if (argc > 1)
	{
		printf("%s\n",argv[1]);
	}
	return 0;
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

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv);

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
