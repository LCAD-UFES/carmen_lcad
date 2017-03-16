#include "udatmo_interface.h"

#include <carmen/global_graphics.h>
#include <carmen/moving_objects_interface.h>

#include "udatmo_memory.h"


carmen_udatmo_moving_obstacles_message *carmen_udatmo_new_moving_obstacles_message(size_t size)
{
	carmen_udatmo_moving_obstacles_message *message = CREATE(carmen_udatmo_moving_obstacles_message);
	carmen_udatmo_init_moving_obstacles_message(message, size);
	return message;
}


void carmen_udatmo_init_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, size_t size)
{
	// The value of carmen_get_host() is retrieved through getenv(), which may change
	// the returned buffer on later calls. Therefore it's safer to create a copy of
	// the returned string.
	//
	// See: http://www.cplusplus.com/reference/cstdlib/getenv/
	//
	// "The pointer returned [by a getenv() call] points to an internal memory block,
	// whose content or validity may be altered by further calls to getenv
	// (but not by other library functions)."
	static char *host = NULL;
	if (host == NULL)
	{
		char *host_temp = carmen_get_host();
		host = (char*) malloc(strlen(host_temp) * sizeof(char));
		strcpy(host, host_temp);
	}

	message->host = host;
	message->timestamp = 0;
	message->num_obstacles = size;

	int bytes = sizeof(carmen_udatmo_moving_obstacle) * size;
	message->obstacles = (carmen_udatmo_moving_obstacle*) malloc(bytes);
	memset(message->obstacles, 0, bytes);
}


void carmen_udatmo_clear_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message)
{
	message->host = NULL;
	message->timestamp = 0;
	message->num_obstacles = 0;
	DELETE(message->obstacles);
}


void carmen_udatmo_publish_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME);
}


void carmen_udatmo_subscribe_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message,
													  void (*handler)(carmen_udatmo_moving_obstacles_message*),
													  carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME,
							 (char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_FMT,
							 message, sizeof(carmen_udatmo_moving_obstacles_message),
							 (carmen_handler_t) handler,
							 subscribe_how);
}


void carmen_udatmo_unsubscribe_moving_obstacles_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME, handler);
}


void carmen_udatmo_fill_virtual_laser_message(carmen_udatmo_moving_obstacles_message *message, int offset, carmen_mapper_virtual_laser_message *out)
{
	int n = message->num_obstacles;
	carmen_udatmo_moving_obstacle *obstacles = message->obstacles;
	if (n == 0 || obstacles[0].rddf_index == -1)
		return;

	for (int i = 0; i < n; i++)
	{
		int k = i + offset;
		out->colors[k] = CARMEN_BLUE;
		out->positions[k].x = obstacles[i].x;
		out->positions[k].y = obstacles[i].y;
	}
}


void carmen_udatmo_display_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, carmen_robot_ackerman_config_t *robot_config)
{
	int n = message->num_obstacles;
	carmen_udatmo_moving_obstacle *obstacle = message->obstacles;
	if (n == 0 || obstacle->rddf_index == -1)
		return;

	static carmen_moving_objects_point_clouds_message *out = NULL;
	if (out == NULL)
	{
		out = CREATE(carmen_moving_objects_point_clouds_message);
		memset(out, 0, sizeof(carmen_moving_objects_point_clouds_message));
	}

	out->timestamp = carmen_get_time();
	out->host = carmen_get_host();
	RESIZE(out->point_clouds, t_point_cloud_struct, n);
	out->num_point_clouds = n;

	t_point_cloud_struct *point = out->point_clouds;
	double length = robot_config->length;
	double width = robot_config->width;

	for (int i = 0; i < n; i++, point++, obstacle++)
	{
		memset(point, 0, sizeof(t_point_cloud_struct));
		point->object_pose.x = obstacle->x;
		point->object_pose.y = obstacle->y;
		point->orientation = obstacle->theta;
		point->length = length;
		point->width = width;
	}

	carmen_moving_objects_point_clouds_publish_message(out);
}
