#include <carmen/carmen.h>
#include <tf.h>

#include "tf_helper.h"


static char* car_tf_name = (char*)"/car";
static char* velodyne_tf_name = (char*)"/velodyne";
static char* board_tf_name = (char*)"/board";

static carmen_pose_3D_t velodyne_pose;
static carmen_pose_3D_t sensor_board_pose;

static tf::Transformer transformer(false);


static tf::Vector3
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return tf_vector;
}


static carmen_vector_3D_t
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();
	
	return carmen_vector;
}


static tf::Quaternion
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


static carmen_orientation_3D_t 
get_carmen_orientation_from_tf_transform(tf::Transform transform)
{
	carmen_orientation_3D_t orientation;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(orientation.yaw, orientation.pitch, orientation.roll);
	
	return orientation;
}


static carmen_pose_3D_t 
tf_transform_to_carmen_pose_3D(tf::Transform transform)
{
	carmen_pose_3D_t pose;

	pose.position = tf_vector3_to_carmen_vector3(transform.getOrigin());
	pose.orientation = get_carmen_orientation_from_tf_transform(transform);
	
	return pose;
}


static void
initialize_carmen_parameters(int argc, char** argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
		{(char*)"velodyne", (char*)"x",			CARMEN_PARAM_DOUBLE, &velodyne_pose.position.x,		0, NULL},
		{(char*)"velodyne", (char*)"y",			CARMEN_PARAM_DOUBLE, &velodyne_pose.position.y,		0, NULL},
		{(char*)"velodyne", (char*)"z",			CARMEN_PARAM_DOUBLE, &velodyne_pose.position.z,		0, NULL},
		{(char*)"velodyne", (char*)"roll",		CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.roll,	0, NULL},
		{(char*)"velodyne", (char*)"pitch",		CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.pitch,	0, NULL},
		{(char*)"velodyne", (char*)"yaw",		CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.yaw,	0, NULL},
		{(char*)"sensor_board_1", (char*)"x",		CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.x,	0, NULL},
		{(char*)"sensor_board_1", (char*)"y",		CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.y,	0, NULL},
		{(char*)"sensor_board_1", (char*)"z",		CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.z,	0, NULL},
		{(char*)"sensor_board_1", (char*)"roll",	CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.roll,0, NULL},
		{(char*)"sensor_board_1", (char*)"pitch",	CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.pitch,0, NULL},
		{(char*)"sensor_board_1", (char*)"yaw",		CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.yaw,0, NULL},

	};
	
	num_items = sizeof(param_list) / sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);
}


static void
initialize_transformations(void)
{
	tf::Time::init();

	tf::Transform velodyne_position_on_board;	
	velodyne_position_on_board.setOrigin(carmen_vector3_to_tf_vector3(velodyne_pose.position));
	velodyne_position_on_board.setRotation(carmen_rotation_to_tf_quaternion(velodyne_pose.orientation));
	tf::StampedTransform board_to_velodyne_transform(velodyne_position_on_board, tf::Time(0), board_tf_name, velodyne_tf_name);
	transformer.setTransform(board_to_velodyne_transform, "board_to_velodyne_transform");

	tf::Transform board_position_on_car;	
	board_position_on_car.setOrigin(carmen_vector3_to_tf_vector3(sensor_board_pose.position));
	board_position_on_car.setRotation(carmen_rotation_to_tf_quaternion(sensor_board_pose.orientation));
	tf::StampedTransform car_to_board_transform(board_position_on_car, tf::Time(0), car_tf_name, board_tf_name);
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");
}


carmen_pose_3D_t
get_velodyne_pose_in_relation_to_car_helper(int argc, char** argv)
{	
	static int initialized = 0;

	if(initialized == 0)
	{
		initialize_carmen_parameters(argc, argv);
		initialize_transformations();

		initialized = 1;
	}

	tf::StampedTransform car_to_velodyne;
	transformer.lookupTransform(car_tf_name, velodyne_tf_name, tf::Time(0), car_to_velodyne);
	
	carmen_pose_3D_t velodyne_pose = tf_transform_to_carmen_pose_3D(car_to_velodyne);

	return velodyne_pose;
}

