#include <string.h>
#include <carmen/carmen.h>
#include "xyz_pointcloud_lidar_interface.h"

void
carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(int sensor_id, char message_name[])
{
	if (sensor_id == -1)
		strcpy(message_name, CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME);
	else
		sprintf(message_name, "%s%d", CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME, sensor_id);
}

IPC_RETURN_TYPE
carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(carmen_xyz_pointcloud_lidar_message *message, int sensor_id)
{
	IPC_RETURN_TYPE err;

	static char message_name[64];
	carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(sensor_id, message_name);
	err = IPC_publishData(message_name, message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);

	return err;
}

void
carmen_subscribe_xyz_pointcloud_message(carmen_xyz_pointcloud_lidar_message *message,
										 carmen_handler_t handler,
										 carmen_subscribe_t subscribe_how,
										 int sensor_id)
{
	static char message_name[64];
	carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(sensor_id, message_name);
	
	carmen_subscribe_message(message_name,
			(char*) CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_FMT,
			message, sizeof(carmen_xyz_pointcloud_lidar_message),
			handler, subscribe_how);
}

void
carmen_unsubscribe_xyz_pointcloud_message(carmen_handler_t handler, int sensor_id)
{
	static char message_name[64];
	carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(sensor_id, message_name);
	carmen_unsubscribe_message(message_name, handler);
}


void
load_xyz_lidar_config(int argc, char** argv, int xyz_lidar_id, carmen_xyz_lidar_config **xyz_lidar_config)
{
    char xyz_lidar_string[256];

 	carmen_xyz_lidar_config *xyz_lidar_config_p = *xyz_lidar_config;
	
 	sprintf(xyz_lidar_string, "xyz_lidar%d", xyz_lidar_id);        // Geather the lidar id

    carmen_param_t param_list[] =
    {
		{xyz_lidar_string, (char *) "model", CARMEN_PARAM_STRING, &xyz_lidar_config_p->model, 0, NULL},
		{xyz_lidar_string, (char *) "ip", CARMEN_PARAM_STRING, &xyz_lidar_config_p->ip, 0, NULL},
		{xyz_lidar_string, (char *) "port", CARMEN_PARAM_STRING, &xyz_lidar_config_p->port, 0, NULL},
		{xyz_lidar_string, (char *) "max_range", CARMEN_PARAM_DOUBLE, &xyz_lidar_config_p->max_range, 1, NULL},
		{xyz_lidar_string, (char *) "x", CARMEN_PARAM_DOUBLE, &(xyz_lidar_config_p->pose.position.x), 1, NULL},
		{xyz_lidar_string, (char *) "y", CARMEN_PARAM_DOUBLE, &(xyz_lidar_config_p->pose.position.y), 1, NULL},
		{xyz_lidar_string, (char *) "z", CARMEN_PARAM_DOUBLE, &xyz_lidar_config_p->pose.position.z, 1, NULL},
		{xyz_lidar_string, (char *) "roll", CARMEN_PARAM_DOUBLE, &xyz_lidar_config_p->pose.orientation.roll, 1, NULL},
		{xyz_lidar_string, (char *) "pitch", CARMEN_PARAM_DOUBLE, &xyz_lidar_config_p->pose.orientation.pitch, 1, NULL},
		{xyz_lidar_string, (char *) "yaw", CARMEN_PARAM_DOUBLE, &xyz_lidar_config_p->pose.orientation.yaw, 1, NULL},
#ifdef USE_REAR_BULLBAR
		{xyz_lidar_string, (char *) "sensor_reference", CARMEN_PARAM_INT, &xyz_lidar_config_p->sensor_reference, 0, NULL},
#endif
 	};
 	carmen_param_install_params(argc, argv, param_list, (sizeof(param_list) / sizeof(param_list[0])));

 	// DO NOT ERASE very useful for debug
 	// printf("Model: %s Port: %s Shot size: %d Min Sensing: %d Max Sensing: %d Range division: %d Time: %lf\n", lidar_config_p->model, lidar_config_p->port, lidar_config_p->shot_size, lidar_config_p->min_sensing, lidar_config_p->max_sensing, lidar_config_p->range_division_factor, lidar_config_p->time_between_shots);
 	// printf("X: %lf Y: %lf Z: %lf R: %lf P: %lf Y: %lf\n", lidar_config_p->pose.position.x, lidar_config_p->pose.position.y, lidar_config_p->pose.position.z, lidar_config_p->pose.orientation.roll, lidar_config_p->pose.orientation.pitch, lidar_config_p->pose.orientation.yaw);
 	// for (int i = 0; i < lidar_config_p->shot_size; i++)
 	// 	printf("%lf ", lidar_config_p->vertical_angles[i]); printf("\n");
 	// for (int i = 0; i < lidar_config_p->shot_size; i++)
 	// 	printf("%d ", lidar_config_p->ray_order[i]); printf("\n");
}



