#include <carmen/carmen.h>
#include <carmen/rotation_geometry.h>
#include <carmen/fused_odometry_interface.h>

#include "icp_processor.h"

struct point_cloud
{
	carmen_vector_3D_t* points;
	carmen_vector_3D_t* point_color;
	carmen_vector_3D_t car_position;
	int num_points;
	double timestamp;
};
typedef struct point_cloud point_cloud;


static point_cloud *velodyne_points;
static int last_velodyne_position;
static int velodyne_initialized;
static int velodyne_size;

static carmen_pose_3D_t car_fused_pose;
static carmen_vector_3D_t car_fused_velocity;
static double car_fused_time;
static carmen_vector_3D_t gps_position_at_turn_on;

static carmen_pose_3D_t velodyne_pose;

static icp_processor* icp;

static void
carmen_fused_odometry_message_handler(carmen_fused_odometry_message* odometry_message)
{
	car_fused_pose = odometry_message->pose;
	car_fused_velocity = odometry_message->velocity;
	car_fused_time = odometry_message->timestamp;
	gps_position_at_turn_on = odometry_message->gps_position_at_turn_on;
}

carmen_vector_3D_t
get_point_position_global_reference(carmen_vector_3D_t car_position, carmen_vector_3D_t car_reference, rotation_matrix* car_to_global_matrix)
{

	//rotation_matrix* r_matrix = create_rotation_matrix(car_fused_pose.orientation);

	carmen_vector_3D_t global_reference = multiply_matrix_vector(car_to_global_matrix, car_reference);

	carmen_vector_3D_t point = add_vectors(global_reference, car_position);

	//destroy_rotation_matrix(r_matrix);

	return point;
}


static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range, rotation_matrix* velodyne_to_car_matrix)
{
	double cos_rot_angle = cos(rot_angle);
	double sin_rot_angle = sin(rot_angle);

	double cos_vert_angle = cos(vert_angle);
	double sin_vert_angle = sin(vert_angle);

	double xy_distance = range * cos_vert_angle;

	carmen_vector_3D_t velodyne_reference;

	velodyne_reference.x = (xy_distance * sin_rot_angle);
	velodyne_reference.y = (xy_distance * cos_rot_angle);
	velodyne_reference.z = (range * sin_vert_angle);

	//rotation_matrix* velodyne_to_car_matrix = create_rotation_matrix(velodyne_pose.orientation);
	carmen_vector_3D_t car_reference = multiply_matrix_vector(velodyne_to_car_matrix, velodyne_reference);
	carmen_vector_3D_t reading = add_vectors(car_reference, velodyne_pose.position);
	//destroy_rotation_matrix(velodyne_to_car_matrix);

	return reading;
}


static void
create_cloud_from_velodyne_message(carmen_velodyne_partial_scan_message* velodyne_message)
{
	static double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0,
	-24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
	-13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };

	static double last_timestamp = 0.0;

	//if(!odometry_initialized)
	//	return;

	if(last_timestamp == 0.0)
	{
		last_timestamp = velodyne_message->timestamp;
		return;
	}

	velodyne_initialized = 1;

	last_velodyne_position++;
	if (last_velodyne_position >= velodyne_size)
	{
		last_velodyne_position = 0;
	}

	int num_points = velodyne_message->number_of_32_laser_shots*(32-11);

	if(velodyne_points[last_velodyne_position].num_points < num_points)
	{
		velodyne_points[last_velodyne_position].points = (carmen_vector_3D_t*)realloc(velodyne_points[last_velodyne_position].points, num_points*sizeof(carmen_vector_3D_t));
	}
	velodyne_points[last_velodyne_position].num_points = num_points;

	velodyne_points[last_velodyne_position].point_color = NULL;
	velodyne_points[last_velodyne_position].car_position = car_fused_pose.position;
	velodyne_points[last_velodyne_position].timestamp = velodyne_message->timestamp;

	rotation_matrix* velodyne_to_car_matrix = create_rotation_matrix(velodyne_pose.orientation);
	rotation_matrix* r_matrix_car_to_global = create_rotation_matrix(car_fused_pose.orientation);

	int i;
	for(i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		int k = 0;
		int j;
		for(j = 0; j < 32; j++)		
		{
			if(vertical_correction[j] < -16.0)
			{
				continue;
			}

			carmen_vector_3D_t point_position = get_velodyne_point_car_reference(	carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle),
																					carmen_degrees_to_radians(vertical_correction[j]),
																					velodyne_message->partial_scan[i].distance[j]/500.0,
																					velodyne_to_car_matrix);

			double shot_time = last_timestamp + ((velodyne_message->timestamp - last_timestamp)*((double)i)/((double)velodyne_message->number_of_32_laser_shots));
			carmen_vector_3D_t car_interpolated_position = carmen_get_interpolated_robot_position_at_time(car_fused_pose, car_fused_velocity, car_fused_time, shot_time, r_matrix_car_to_global);


			velodyne_points[last_velodyne_position].points[i*(32-11) + k] = get_point_position_global_reference(car_interpolated_position, point_position, r_matrix_car_to_global);
			k++;
		}
	}

	destroy_rotation_matrix(velodyne_to_car_matrix);
	destroy_rotation_matrix(r_matrix_car_to_global);

	last_timestamp = velodyne_message->timestamp;
}


static void
fill_past_cloud(void)
{
	int k = last_velodyne_position - 1;
	if(k<0)
		k += velodyne_size;

	set_icp_cloud_1(icp, velodyne_points[k].points, velodyne_points[k].num_points, 1);

	int i;
	for(i=1; i<2; i++)
	{
		int k = last_velodyne_position - i - 1;
		if(k<0)
			k += velodyne_size;

		add_to_cloud_1(icp, velodyne_points[k].points, velodyne_points[k].num_points, 1);
	}
}

static void
fill_current_cloud(void)
{
	int k = last_velodyne_position;

	set_icp_cloud_2(icp, velodyne_points[k].points, velodyne_points[k].num_points, 1);
}

static Eigen::Matrix4f
create_eigen_matrix_from_carmen(carmen_vector_3D_t displacement)
{
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0,3) = displacement.x;
	transform(1,3) = displacement.y;
	transform(2,3) = displacement.z;

	return transform;
}

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message* velodyne_message)
{
	//printf("Scan\n");

	static int init = 0;
	static carmen_pose_3D_t last_fused_pose;

	create_cloud_from_velodyne_message(velodyne_message);

	last_fused_pose = car_fused_pose;

	if(init < 22)
	{
		init++;
		return;
	}

	Eigen::Matrix4f transform_guess = create_eigen_matrix_from_carmen(car_fused_velocity);
	
	fill_past_cloud();
	fill_current_cloud();
	get_cloud_transformation(icp,transform_guess);
	

}

static void
init_velodyne(void)
{
	velodyne_initialized = 0; // Only considered initialized when first message is received

	velodyne_size = 50;
	velodyne_points = (point_cloud*)malloc(velodyne_size * sizeof(point_cloud));

	int i;
	for (i = 0; i < velodyne_size; i++)
	{
		velodyne_points[i].points = NULL;
		velodyne_points[i].point_color = NULL;
		velodyne_points[i].num_points = 0;
		velodyne_points[i].timestamp = carmen_get_time();
	}

	last_velodyne_position = 0;
}

static void
init_carmem_params(int argc, char** argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{"velodyne", "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{"velodyne", "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{"velodyne", "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{"velodyne", "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{"velodyne", "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{"velodyne", "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL}

		};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}

static void
init_stuff(int argc, char** argv)
{
	init_carmem_params(argc, argv);
	init_velodyne();
	icp = create_icp_processor();
}

static void destroy_stuff(void)
{
	destroy_icp_processor(icp);
}

static void subscribe_messages(void)
{
	carmen_velodyne_subscribe_partial_scan_message(	NULL,
													(carmen_handler_t)velodyne_partial_scan_message_handler,
													CARMEN_SUBSCRIBE_LATEST);

	carmen_fused_odometry_subscribe_fused_odometry_message(	NULL,
								       						(carmen_handler_t)carmen_fused_odometry_message_handler,
								       						CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	init_stuff(argc, argv);
	subscribe_messages();
	carmen_ipc_dispatch();
	carmen_ipc_disconnect();

	destroy_stuff();

	return 0;
}
