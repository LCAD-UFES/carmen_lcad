/*********************************************************
	---  Moving Objects Module ---
**********************************************************/

#include "moving_objects_messages.h"
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <carmen/laser_messages.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/localize_ackerman_motion.h>
#include <carmen/localize_ackerman_velodyne.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/stereo_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_messages.h>

#include <Eigen/Core>
#include <tf.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "moving_objects.h"


#define NUM_VELODYNE_POINT_CLOUDS	5
int num_velodyne_point_clouds = NUM_VELODYNE_POINT_CLOUDS;

carmen_pose_3D_t car_global_pose;
carmen_pose_3D_t car_fused_pose;
carmen_vector_3D_t car_fused_velocity;
double car_fused_time;
double car_phi;

moving_objects_input_data_t moving_objects_input;

static carmen_pose_3D_t velodyne_pose;
static carmen_pose_3D_t sensor_board_1_pose;
rotation_matrix *sensor_board_1_to_car_matrix;

static int localize_initialized = 0;
sensor_parameters_t *spherical_sensor_params;
sensor_data_t *spherical_sensor_data;
double max_range = 0.0;

int number_of_sensors;

double robot_wheel_radius;
carmen_robot_ackerman_config_t car_config;

static carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
int last_num_points;
double safe_range_above_sensors;
double highest_sensor = 0.0;
carmen_vector_3D_t *carmen_3d_point_clouds;
int** indices_clusters = NULL;

static int first_velodyne_message_flag = 1;
static int first_offline_map_message_flag = 0;

static carmen_map_p offline_grid_map = NULL;

int frame = 1;

int number_of_threads = 1;


#ifdef AJUSTE
double sigma = 3.0;			// desvio padrão dos pesos das partículas (0.1 funciona apenas no baseline)
double alpha_1 = 2.0; 		// desvio padrão da velocidade padrão 0.2
double alpha_2 = 0.12; 		// desvio padrão de theta padrão 0.01
FILE * parametro;
static int cont = 0;
static int rodada = 1;
#endif


///////////////////////////////////////////////////////////////////////////////////////////////


int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *)malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


moving_objects_input_data_t
bundle_moving_objects_input_data()
{
	moving_objects_input_data_t l_moving_objects_input;

	l_moving_objects_input.car_config                   = car_config;
	l_moving_objects_input.car_fused_pose               = car_fused_pose;
	l_moving_objects_input.car_fused_time               = car_fused_time;
	l_moving_objects_input.car_fused_velocity           = car_fused_velocity;
	l_moving_objects_input.car_phi                      = car_phi;
	l_moving_objects_input.car_global_pose              = car_global_pose;
	l_moving_objects_input.highest_sensor               = highest_sensor;
	l_moving_objects_input.num_velodyne_point_clouds    = num_velodyne_point_clouds;
	l_moving_objects_input.number_of_sensors            = number_of_sensors;
	l_moving_objects_input.robot_wheel_radius           = robot_wheel_radius;
	l_moving_objects_input.safe_range_above_sensors     = safe_range_above_sensors;
	l_moving_objects_input.sensor_board_1_pose          = sensor_board_1_pose;
	l_moving_objects_input.sensor_board_1_to_car_matrix = sensor_board_1_to_car_matrix;
	l_moving_objects_input.velodyne_pose                = velodyne_pose;
	l_moving_objects_input.first_offline_map_message    = first_offline_map_message_flag;
	l_moving_objects_input.indices_clusters             = indices_clusters;

	return l_moving_objects_input;
}


static carmen_vector_3D_t
get_position_offset(void)
{
    carmen_vector_3D_t zero;

    zero.x = 0;
    zero.y = 0;
    zero.z = 0;

    return zero;
}


Eigen::Vector3f
get_rgb_from_color_palette_and_association (int num_color, list<color_palette_and_association_data_t> color_palette_and_association)
{
	Eigen::Vector3f	color_palette;

	for (list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
			it != color_palette_and_association.end(); it++)
	{
		if (num_color == it->num_color)
		{
			color_palette[0] = it->color_palette[0];
			color_palette[1] = it->color_palette[1];
			color_palette[2] = it->color_palette[2];
			break;
		}
	}

	return color_palette;
}


void
initialize_objects_maps()
{
	offline_grid_map = (carmen_map_t *) malloc (sizeof(carmen_map_t));
	offline_grid_map->complete_map = NULL;
}


void
initialize_map(carmen_map_t *map, int gridmap_size_x, int gridmap_size_y, double gridmap_resolution)
{
	int i, j;

	map->config.resolution = gridmap_resolution;
	map->config.x_size = gridmap_size_x;
	map->config.y_size = gridmap_size_y;
	map->config.map_name = NULL;

	map->complete_map = (double*) malloc(sizeof(double) * gridmap_size_x * gridmap_size_y);
	carmen_test_alloc(map->complete_map);
	map->map = (double**) malloc(sizeof(double*) * gridmap_size_x);
	carmen_test_alloc(map->map);

	for (i = 0; i < gridmap_size_x; i++)
	{
		map->map[i] = &map->complete_map[i * gridmap_size_y];

		//initializing map with unknown
		for (j = 0; j < gridmap_size_y; j++)
		{
			map->map[i][j] = -1.0;
		}
	}
	return;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
static void
carmen_map_server_offline_map_handler(carmen_map_server_offline_map_message *message)
{
	printf("entrei!\n");
	if (message != NULL)
	{
		if (first_offline_map_message_flag == 0)
		{
			first_offline_map_message_flag = 1;
		}
	}
	printf("first_offline_map_message_flag: %d\n", first_offline_map_message_flag);

	if (offline_grid_map == NULL)
	{
		initialize_objects_maps();
		initialize_map(offline_grid_map, message->config.x_size, message->config.y_size, message->config.resolution);

		offline_grid_map->config = message->config;
		memcpy(offline_grid_map->complete_map, message->complete_map, message->config.x_size * message->config.y_size * sizeof(double));
	}
	else
	{
		offline_grid_map->config = message->config;

		memcpy(offline_grid_map->complete_map, message->complete_map, message->config.x_size * message->config.y_size * sizeof(double));
	}
}


void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *localize_ackerman_message)
{

	car_fused_pose = localize_ackerman_message->pose;
	car_fused_pose.position = sub_vectors(car_fused_pose.position, get_position_offset());
	car_fused_pose.orientation.yaw = localize_ackerman_message->globalpos.theta;

	car_fused_velocity = localize_ackerman_message->velocity;
	car_fused_time = localize_ackerman_message->timestamp;
	car_phi = localize_ackerman_message->phi;

	car_fused_pose.position.z        = 0.0;
	car_fused_pose.orientation.pitch = 0.0;
	car_fused_pose.orientation.roll  = 0.0;

	if (first_offline_map_message_flag)
	{
		car_global_pose = car_fused_pose;
		first_offline_map_message_flag = -1;
	}

	localize_initialized = 1;
}


void
init_allocation_moving_objects_point_clouds_message(int num_point_clouds)
{
	moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(num_point_clouds * sizeof(t_point_cloud_struct)));
	carmen_test_alloc(moving_objects_point_clouds_message.point_clouds);
}


void
init_allocation_moving_objects_points_for_point_cloud_i(int index_point_cloud, int num_points)
{
	moving_objects_point_clouds_message.point_clouds[index_point_cloud].points = (carmen_vector_3D_t *) (malloc(num_points * sizeof(carmen_vector_3D_t)));
	carmen_test_alloc(moving_objects_point_clouds_message.point_clouds[index_point_cloud].points);
}


void
deallocation_moving_objects_point_clouds_message()
{
	int i;

	for (i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++)
	{
		free(moving_objects_point_clouds_message.point_clouds[i].points);
//		free(moving_objects_point_clouds_message.point_clouds[i].particulas);
	}
	free(moving_objects_point_clouds_message.point_clouds);
}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	int i, j;
	int num_points;
	int num_point_clouds;
	Eigen::Vector3f	color_palette;
	std::list<object_point_cloud_data_t> list_point_clouds;
	std::list<color_palette_and_association_data_t> l_color_palette_and_association;


#ifdef AJUSTE
	// todo modificações para testar os parâmetros primeira mensagem

	if (cont == 0)
	{
		char filename[256];

		sprintf(filename,"parametros/%d_%02d_sig-%.2f_alf1-%.2f_alf2-%.2f.txt",rodada,cont+1,sigma,alpha_1,alpha_2);
		parametro = fopen(filename,"w");
		cont++;
	}

	fflush(parametro);
	if (velodyne_message->timestamp == 1379356494.718246)
	{
		if(parametro != NULL){
			fclose(parametro);
		}
		printf("Fim!\n");

		rodada++;
		cont = 0;
//		if (alpha_2 < 0.18)
//		{
//			if (cont > 0)
//				alpha_2 += 0.03;
//		}
//		else
//		{
//			alpha_2 = 0.12;
////			if(alpha_1 < 2.0)
////			{
////				alpha_1 += 0.5;
////			}
////			else
////			{
////				alpha_1 = 0.5;
//				sigma += 2.0;
//
//				if(sigma > 9.0)
//				{
//					sigma = 5.0;
//					rodada++;
//					cont = 0;
//				}
//
////			}
//		}

		//sprintf(filename,"parametros/final-22.txt",cont+16,sigma,alpha_1,alpha_2);
	}
#endif

	if (localize_initialized)
	{
		num_points = velodyne_message->number_of_32_laser_shots * spherical_sensor_params[0].vertical_resolution;
		// printf("num_velodyne_points: %d\n", num_points);

		if (first_velodyne_message_flag)
		{
			carmen_3d_point_clouds = (carmen_vector_3D_t *) malloc(velodyne_message->number_of_32_laser_shots*spherical_sensor_params[0].vertical_resolution*sizeof(carmen_vector_3D_t));
			carmen_test_alloc(carmen_3d_point_clouds);
			first_velodyne_message_flag = 0;
		}

		if (spherical_sensor_data[0].points == NULL)
			return;

		moving_objects_input = bundle_moving_objects_input_data();

		detect_and_follow_moving_objects(list_point_clouds, velodyne_message, &spherical_sensor_params[0],
				&spherical_sensor_data[0], &car_fused_velocity, car_phi, moving_objects_input, carmen_3d_point_clouds,
				offline_grid_map);

		num_point_clouds = list_point_clouds.size();

		/*** PRINT FRAME ID, TIMESTAMP AND NUMBER OF POINT CLOUDS SEGMENTED IN THE SCENE ***/
		printf("%d ", frame);
		printf("%.10f\n", velodyne_message->timestamp);
		printf("clouds: %d\n\n", num_point_clouds);
		frame++;

		if (num_point_clouds == 0)
			return;

		l_color_palette_and_association = get_color_palette_and_association();

		init_allocation_moving_objects_point_clouds_message(num_point_clouds);

		i = 0;
		for (std::list<object_point_cloud_data_t>::const_iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++, i++)
		{
			num_points = it->point_cloud.size();
			init_allocation_moving_objects_points_for_point_cloud_i(i, num_points);
		}

		moving_objects_point_clouds_message.num_point_clouds = num_point_clouds;

		i = 0;
		for (std::list<object_point_cloud_data_t>::const_iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); ++it)
		{
			j = 0;

			color_palette = get_rgb_from_color_palette_and_association(it->num_color_associate, l_color_palette_and_association);

			moving_objects_point_clouds_message.point_clouds[i].r = ((double)color_palette[0]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].g = ((double)color_palette[1]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].b = ((double)color_palette[2]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].point_size = it->point_cloud.size();
			moving_objects_point_clouds_message.point_clouds[i].linear_velocity = it->linear_velocity;
			moving_objects_point_clouds_message.point_clouds[i].orientation = it->orientation;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.x = it->object_pose.position.x;// + it->car_global_pose.position.x;//it->centroid[0] + it->car_global_pose.position.x;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.y = it->object_pose.position.y;// + it->car_global_pose.position.y;//it->centroid[1] + it->car_global_pose.position.y;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.z = it->object_pose.position.z + it->car_global_pose.position.z;//it->centroid[2] + it->car_global_pose.position.z;
			moving_objects_point_clouds_message.point_clouds[i].height = it->geometry.height;
			moving_objects_point_clouds_message.point_clouds[i].length = it->geometry.length;
			moving_objects_point_clouds_message.point_clouds[i].width= it->geometry.width;
			moving_objects_point_clouds_message.point_clouds[i].geometric_model = it->geometric_model;
			moving_objects_point_clouds_message.point_clouds[i].model_features = it->model_features;
			moving_objects_point_clouds_message.point_clouds[i].num_associated = it->num_color_associate;

			for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
			{
				moving_objects_point_clouds_message.point_clouds[i].points[j].x = pit->x + it->car_global_pose.position.x;
				moving_objects_point_clouds_message.point_clouds[i].points[j].y = pit->y + it->car_global_pose.position.y;
				moving_objects_point_clouds_message.point_clouds[i].points[j].z = pit->z + it->car_global_pose.position.z;
				j++;
			}

			//fixme para a visualização das partículas
//			moving_objects_point_clouds_message.point_clouds[i].particulas = (particle_print_t*) malloc(400 * sizeof(particle_print_t));
//			if(it->particle_set.size() > 0) {
//				for(int k = 0; k < 400; k++){
//					moving_objects_point_clouds_message.point_clouds[i].particulas[k].class_id = it->particle_set[k].class_id;
//					moving_objects_point_clouds_message.point_clouds[i].particulas[k].geometry = it->particle_set[k].model_features.geometry;
//					moving_objects_point_clouds_message.point_clouds[i].particulas[k].pose = it->particle_set[k].pose;
//					moving_objects_point_clouds_message.point_clouds[i].particulas[k].velocity = it->particle_set[k].velocity;
//				}
//			}


			i++;
		}
		printf("num of tracked objects: %d\n", num_point_clouds);
		moving_objects_point_clouds_message.timestamp = velodyne_message->timestamp;
		/*** PUBLISH MESSAGE ***/
		carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);
		deallocation_moving_objects_point_clouds_message();
	}

}

// todo para a base KITTI
static void
velodyne_variable_scan_message_handler(carmen_velodyne_variable_scan_message *velodyne_message)
{

	int i, j;
	int num_points;
	int num_point_clouds;
	Eigen::Vector3f	color_palette;
	std::list<object_point_cloud_data_t> list_point_clouds;
	std::list<color_palette_and_association_data_t> l_color_palette_and_association;

	if (localize_initialized)
	{
		num_points = velodyne_message->number_of_shots * spherical_sensor_params[8].vertical_resolution;

		if (first_velodyne_message_flag)
		{
			carmen_3d_point_clouds = (carmen_vector_3D_t *) malloc(velodyne_message->number_of_shots*spherical_sensor_params[8].vertical_resolution*sizeof(carmen_vector_3D_t));
			carmen_test_alloc(carmen_3d_point_clouds);
			first_velodyne_message_flag = 0;
		}

		if (spherical_sensor_data[8].points == NULL)
			return;

		moving_objects_input = bundle_moving_objects_input_data();

		 detect_and_follow_moving_objects_variable_scan(list_point_clouds, velodyne_message, &spherical_sensor_params[8],
				&spherical_sensor_data[8], &car_fused_velocity, car_phi, moving_objects_input, carmen_3d_point_clouds,
				offline_grid_map);

		num_point_clouds = list_point_clouds.size();

		/*** PRINT NUMBER OF POINT CLOUDS SEGMENTED IN THE SCENE ***/
//		printf("frame: %d \n", frame);
//		printf("num_point_clouds: %d \n", num_point_clouds);
//		printf("current timestamp: %.10f \n", velodyne_message->timestamp);
		frame++;

		if (num_point_clouds == 0)
			return;

		l_color_palette_and_association = get_color_palette_and_association();

		init_allocation_moving_objects_point_clouds_message(num_point_clouds);

		i = 0;
		for (std::list<object_point_cloud_data_t>::const_iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++, i++)
		{
			num_points = it->point_cloud.size();
			init_allocation_moving_objects_points_for_point_cloud_i(i, num_points);
		}

		moving_objects_point_clouds_message.num_point_clouds = num_point_clouds;

		i = 0;
		for (std::list<object_point_cloud_data_t>::const_iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); ++it)
		{
			j = 0;

			color_palette = get_rgb_from_color_palette_and_association(it->num_color_associate, l_color_palette_and_association);

			moving_objects_point_clouds_message.point_clouds[i].r = ((double)color_palette[0]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].g = ((double)color_palette[1]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].b = ((double)color_palette[2]) / 255.0;
			moving_objects_point_clouds_message.point_clouds[i].point_size      = it->point_cloud.size();
			moving_objects_point_clouds_message.point_clouds[i].linear_velocity = it->linear_velocity;
			moving_objects_point_clouds_message.point_clouds[i].orientation     = it->orientation;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.x   = it->object_pose.position.x;// + it->car_global_pose.position.x;//it->centroid[0] + it->car_global_pose.position.x;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.y   = it->object_pose.position.y;// + it->car_global_pose.position.y;//it->centroid[1] + it->car_global_pose.position.y;
			moving_objects_point_clouds_message.point_clouds[i].object_pose.z   = it->object_pose.position.z + it->car_global_pose.position.z;//it->centroid[2] + it->car_global_pose.position.z;
			moving_objects_point_clouds_message.point_clouds[i].height          = it->geometry.height;
			moving_objects_point_clouds_message.point_clouds[i].length          = it->geometry.length;
			moving_objects_point_clouds_message.point_clouds[i].width           = it->geometry.width;
			moving_objects_point_clouds_message.point_clouds[i].geometric_model = it->geometric_model;
			moving_objects_point_clouds_message.point_clouds[i].model_features  = it->model_features;
			moving_objects_point_clouds_message.point_clouds[i].num_associated  = it->num_color_associate;

			for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
			{
				moving_objects_point_clouds_message.point_clouds[i].points[j].x = pit->x + it->car_global_pose.position.x;
				moving_objects_point_clouds_message.point_clouds[i].points[j].y = pit->y + it->car_global_pose.position.y;
				moving_objects_point_clouds_message.point_clouds[i].points[j].z = pit->z + it->car_global_pose.position.z;
				j++;
			}

			i++;
		}

		moving_objects_point_clouds_message.timestamp = velodyne_message->timestamp;
		carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);
		deallocation_moving_objects_point_clouds_message();
	}

}



void static
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("moving_objects: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out, carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


	carmen_test_alloc(velodyne_points);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
	*robot_pose_out = robot_pose;
	*robot_velocity_out = robot_velocity;
	*robot_timestamp_out = robot_timestamp;
}

double *
get_variable_velodyne_correction()
{
	double *vert_angle = (double*) calloc(64,sizeof(double));
	carmen_test_alloc(vert_angle);

	// angulo vertical minimo
	double vmin = carmen_radians_to_degrees(-0.45);
	// angulo vertical maximo
	double vmax = carmen_radians_to_degrees(0.111);
	// intervalo angular vertical
	double vstep = (vmax - vmin) / 64.0;

	for(int i = 0; i < 64; i++) {
		vert_angle[i] = vmin + vstep * i;
	}

	return vert_angle;
}

void
print_correction(double * correction, int res)
{
	for(int i = 0; i < res; i++)
		printf("%f ", correction[i]);
}


static void
get_sensors_param(int argc, char **argv)
{
	int i, j;
	int flipped;
	int horizontal_resolution;
	char stereo_velodyne_string[256];

	int stereo_velodyne_vertical_roi_ini;
	int stereo_velodyne_vertical_roi_end;

	int stereo_velodyne_horizontal_roi_ini;
	int stereo_velodyne_horizontal_roi_end;

	int roi_ini, roi_end;


	spherical_sensor_params[0].pose = velodyne_pose;
	spherical_sensor_params[0].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, spherical_sensor_params[0].pose.position, sensor_board_1_to_car_matrix);

	spherical_sensor_params[0].height = spherical_sensor_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (spherical_sensor_params[0].height > highest_sensor)
		highest_sensor = spherical_sensor_params[0].height;

	if (spherical_sensor_params[0].alive && !strcmp(spherical_sensor_params[0].name,"velodyne"))
	{
		spherical_sensor_params[0].ray_order = carmen_velodyne_get_ray_order();
		spherical_sensor_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		// spherical_sensor_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		// spherical_sensor_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
				{spherical_sensor_params[0].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &spherical_sensor_params[0].vertical_resolution, 0, NULL},
				{(char *) "localize_ackerman", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max, 0, NULL},
				{spherical_sensor_params[0].name, (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].time_spent_by_each_scan, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&spherical_sensor_data[0].points, &spherical_sensor_data[0].intensity, &spherical_sensor_data[0].robot_pose,
				&spherical_sensor_data[0].robot_velocity, &spherical_sensor_data[0].robot_timestamp, &spherical_sensor_data[0].robot_phi);
		spherical_sensor_params[0].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[0].pose.orientation);
		spherical_sensor_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[0], spherical_sensor_params[0].vertical_resolution, number_of_threads);

		if (max_range < spherical_sensor_params[0].range_max)
		{
			max_range = spherical_sensor_params[0].range_max;
		}
		spherical_sensor_params[0].current_range_max = spherical_sensor_params[0].range_max;
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (spherical_sensor_params[i].alive)
		{
			spherical_sensor_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);

			spherical_sensor_params[i].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, spherical_sensor_params[i].pose.position, sensor_board_1_to_car_matrix);
			spherical_sensor_params[i].height = spherical_sensor_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (spherical_sensor_params[i].height > highest_sensor)
				highest_sensor = spherical_sensor_params[i].height;

			sprintf(stereo_velodyne_string, "%s%d", "stereo", i);


			carmen_param_t param_list[] =
			{
					{spherical_sensor_params[i].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &spherical_sensor_params[i].vertical_resolution, 0, NULL},
					{spherical_sensor_params[i].name, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
					{spherical_sensor_params[i].name, (char *) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
					{spherical_sensor_params[i].name, (char *) "range_max", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].range_max, 0, NULL},
					{spherical_sensor_params[i].name, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
					{spherical_sensor_params[i].name, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
					{spherical_sensor_params[i].name, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
					{spherical_sensor_params[i].name, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }

			};


			carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

			if (flipped)
			{
				spherical_sensor_params[i].vertical_resolution = horizontal_resolution;
				roi_ini = stereo_velodyne_horizontal_roi_ini;
				roi_end = stereo_velodyne_horizontal_roi_end;
			}
			else
			{
				roi_ini = stereo_velodyne_vertical_roi_ini;
				roi_end = stereo_velodyne_vertical_roi_end;
			}

			if (spherical_sensor_params[i].vertical_resolution > (roi_end - roi_ini))
			{
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
			}

			if (max_range < spherical_sensor_params[i].range_max)
			{
				max_range = spherical_sensor_params[i].range_max;
			}

			spherical_sensor_params[i].current_range_max = spherical_sensor_params[i].range_max;

			spherical_sensor_params[i].range_max_factor = 1.0;
			spherical_sensor_params[i].ray_order = generates_ray_order(spherical_sensor_params[i].vertical_resolution);

			spherical_sensor_params[i].vertical_correction = get_variable_velodyne_correction();
//			spherical_sensor_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, spherical_sensor_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);

			init_velodyne_points(&spherical_sensor_data[i].points, &spherical_sensor_data[i].intensity, &spherical_sensor_data[i].robot_pose,
					&spherical_sensor_data[i].robot_velocity, &spherical_sensor_data[i].robot_timestamp, &spherical_sensor_data[i].robot_phi);
			spherical_sensor_params[i].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[i].pose.orientation);
			spherical_sensor_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[i], spherical_sensor_params[i].vertical_resolution, number_of_threads);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			// if(i == 8) {
			// 	spherical_sensor_params[i].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
			// 	spherical_sensor_params[i].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

			// } else {
			// 	spherical_sensor_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			// 	spherical_sensor_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			// 	for (j = 0; j < 50; j++)
			// 		spherical_sensor_params[i].delta_difference_stddev[j] = 1.0;
			// }

		}
	}
}


void
get_alive_sensors(int argc, char **argv)
{
	int i;

	spherical_sensor_params = (sensor_parameters_t *)calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(spherical_sensor_params);

	spherical_sensor_data = (sensor_data_t *)calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(spherical_sensor_data);

	carmen_param_t param_list[] =
	{
			{(char *) "localize_ackerman", (char *) "velodyne", CARMEN_PARAM_ONOFF, &spherical_sensor_params[0].alive, 0, NULL},
			//{(char *) "localize_ackerman", (char *) "stereo_velodyne1", CARMEN_PARAM_ONOFF, &spherical_sensor_params[1].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne2", CARMEN_PARAM_ONOFF, &spherical_sensor_params[2].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne3", CARMEN_PARAM_ONOFF, &spherical_sensor_params[3].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne4", CARMEN_PARAM_ONOFF, &spherical_sensor_params[4].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne5", CARMEN_PARAM_ONOFF, &spherical_sensor_params[5].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne6", CARMEN_PARAM_ONOFF, &spherical_sensor_params[6].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne7", CARMEN_PARAM_ONOFF, &spherical_sensor_params[7].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne8", CARMEN_PARAM_ONOFF, &spherical_sensor_params[8].alive, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne9", CARMEN_PARAM_ONOFF, &spherical_sensor_params[9].alive, 0, NULL},

			{(char *) "localize_ackerman", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_occ, 0, NULL},
			//{(char *) "localize_ackerman", (char *) "stereo_velodyne1_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_occ, 0, NULL},


			{(char *) "localize_ackerman", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_free, 0, NULL},
			//{(char *) "localize_ackerman", (char *) "stereo_velodyne1_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_free, 0, NULL},

			{(char *) "localize_ackerman", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_l0, 0, NULL},
			//{(char *) "localize_ackerman", (char *) "stereo_velodyne1_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_l0, 0, NULL},

			{(char *) "localize_ackerman", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unexpeted_delta_range_sigma, 0, NULL},
			//{(char *) "localize_ackerman", (char *) "stereo_velodyne1_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", (char *) "stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].unexpeted_delta_range_sigma, 0, NULL},

			{(char *) "localize_ackerman",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max_factor, 0, NULL}


	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{

		spherical_sensor_data[i].ray_position_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));
		spherical_sensor_data[i].maxed = (int**)calloc(number_of_threads ,sizeof(int*));
		spherical_sensor_data[i].obstacle_height = (double**)calloc(number_of_threads ,sizeof(double*));
		spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target = (double**)calloc(number_of_threads ,sizeof(double*));
		spherical_sensor_data[i].point_cloud_index = 0;
		spherical_sensor_data[i].points = NULL;
		spherical_sensor_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));;
		spherical_sensor_data[i].ray_size_in_the_floor = (double**)calloc(number_of_threads ,sizeof(double*));
		spherical_sensor_data[i].processed_intensity = (double**)calloc(number_of_threads ,sizeof(double*));
		spherical_sensor_data[i].ray_hit_the_robot = (int**)calloc(number_of_threads ,sizeof(int*));
		spherical_sensor_data[i].ray_that_hit_the_nearest_target = (int*)calloc(number_of_threads ,sizeof(int));

		spherical_sensor_params[i].name = NULL;
		spherical_sensor_params[i].ray_order = NULL;
		spherical_sensor_params[i].sensor_to_support_matrix = NULL;
		spherical_sensor_params[i].vertical_correction = NULL;
		spherical_sensor_params[i].vertical_resolution = 0;

		for (int j = 0; j < number_of_threads; j++)
		{
			spherical_sensor_data[i].ray_position_in_the_floor[j] = NULL;
			spherical_sensor_data[i].maxed[j] = NULL;
			spherical_sensor_data[i].obstacle_height[j] = NULL;
			spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
			spherical_sensor_data[i].ray_origin_in_the_floor[j] = NULL;
			spherical_sensor_data[i].ray_size_in_the_floor[j] = NULL;
			spherical_sensor_data[i].processed_intensity[j] = NULL;
			spherical_sensor_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (spherical_sensor_params[i].alive)
		{
			spherical_sensor_params[i].name = (char *)calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(spherical_sensor_params[i].name, param_list[i].variable);
		}
	}
}


static int 
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "sensor_board_1",  (char *) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char *) "sensor_board_1",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char *) "sensor_board_1",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},


			{(char *) "velodyne", (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *) "velodyne", (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *) "velodyne", (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *) "velodyne", (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *) "velodyne", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &car_config.length, 0, NULL},
			{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &car_config.width, 0, NULL},
			{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &car_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *) "robot", (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &car_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *) "robot", (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},

			{(char *) "model", 		(char *) "predictive_planner_obstacles_safe_distance", 	CARMEN_PARAM_DOUBLE, &car_config.model_predictive_planner_obstacles_safe_distance, 1, NULL},
			{(char *) "obstacle", 	(char *) "avoider_obstacles_safe_distance", 			CARMEN_PARAM_DOUBLE, &car_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},

			{(char *) "moving_objects",  (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
			{(char *) "localize_ackerman",  (char *) "number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	get_alive_sensors(argc, argv);

	get_sensors_param(argc, argv);


	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char **argv)
{

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Initialize vector of readings laser */


	/* Initialize vector of static points */


	/* Initialize all the relevant parameters */
	read_parameters(argc, argv);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Define messages that your module publishes */
	carmen_moving_objects_point_clouds_define_messages();

	/* Subscribe to sensor and filter messages */
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
				(carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) carmen_map_server_offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL,
    		(carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_stereo_velodyne_subscribe_scan_message(8, NULL,
    		(carmen_handler_t)velodyne_variable_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

//    carmen_localize_ackerman_subscribe_globalpos_message(NULL,
//			(carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);


	carmen_ipc_dispatch();

	return (0);
}
