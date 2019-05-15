

#include <string.h>
#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>

int velodyne_vertical_correction_size = 32;

double velodyne_vertical_correction[32] =
{
		-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
		-20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
		-9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
		2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000
};

int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};


double velodyne_delta_difference_mean[50] =
{
		0.447845973, 0.8118254059, 0.2744132381, 0.8101127149,	0.82030982,	0.8016078725, 0.804877796, 0.7965554561, 0.7756627352,
		0.7697838698, 0.7813010135, 0.7775437988, 0.7175587957, 0.6593800389,	0.6529979871, 0.667852627, 0.6166590995, 0.6905564045,
		0.6779342328, 0.6052055679, 0.6404078803, 0.6579618571,	0.676596366, 0.6282102062, 0.5981359107, 0.5980909959, 0.5619362,
		0.6075080093, 0.6257750262,	0.6222210152, 0.6036784737,	0.6119039759, 0.5791022794,	0.5770027206, 0.5712085, 0.5068878571,
		0.5560903937, 0.5723009259,	0.4718930851, 0.5584450515,	0.5151185185, 0.5402702817,	0.5213501429, 0.4925393443,	0.5237356923,
		0.5267628788, 0.568367561,	0.4769748837, 0.4926688372,	0.5306957143
};

double velodyne_delta_difference_stddev[50] =
{
		0.297237444, 0.2102265115, 0.2468379952, 0.233222978, 0.2371104492, 0.2690028057, 0.2671220475, 0.2635474672, 0.2716227463,
		0.2808910258, 0.2763067042, 0.2773640452, 0.3051253804, 0.3333721101, 0.3472227053, 0.3414741073, 0.3509250867, 0.3130839467,
		0.3307547077, 0.3508115872, 0.3336553474, 0.3209124199, 0.331129849, 0.3542726143, 0.3506590909, 0.3347541008, 0.3548054823,
		0.3435922836, 0.347695954, 0.3308590111, 0.3720740884, 0.3233566445, 0.3477545351, 0.3511283494, 0.336196184, 0.3472701782,
		0.3333434712, 0.3204128562, 0.34184086, 0.3423745322, 0.3524176509, 0.3035344607, 0.3457419408, 0.3194585803, 0.3388110989,
		0.3051421052, 0.3083028964, 0.3277629732, 0.3151197249, 0.3330088828
};



double*
carmen_velodyne_get_vertical_correction()
{
	double *vertical_correction = (double *)malloc(velodyne_vertical_correction_size * sizeof(double));
	memcpy(vertical_correction, velodyne_vertical_correction, velodyne_vertical_correction_size * sizeof(double));
	return vertical_correction;
}


int*
carmen_velodyne_get_ray_order()
{
	int *ray_order = (int *)malloc(velodyne_vertical_correction_size * sizeof(int));
	memcpy(ray_order, velodyne_ray_order, velodyne_vertical_correction_size * sizeof(int));
	return ray_order;
}

double*
carmen_velodyne_get_delta_difference_mean()
{
	double *delta_difference_mean = (double *)malloc(50 * sizeof(double));
	memcpy(delta_difference_mean, velodyne_delta_difference_mean, 50 * sizeof(double));
	return delta_difference_mean;
}


double*
carmen_velodyne_get_delta_difference_stddev()
{
	double *delta_difference_stddev = (double *)malloc(50 * sizeof(double));
	memcpy(delta_difference_stddev, velodyne_delta_difference_stddev, 50 * sizeof(double));
	return delta_difference_stddev;
}


void
carmen_velodyne_subscribe_partial_scan_message(carmen_velodyne_partial_scan_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME,
			(char*) CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT,
			message, sizeof(carmen_velodyne_partial_scan_message),
			handler, subscribe_how);
}


void
carmen_velodyne_unsubscribe_partial_scan_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*) CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, handler);
}


void
carmen_velodyne_create_variable_velodyne_message_name(int sensor_id, char message_name[])
{
	if (sensor_id == -1)
		strcpy(message_name, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME);
	else
		sprintf(message_name, "%s%d", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME, sensor_id);
}


void
carmen_velodyne_subscribe_variable_scan_message(carmen_velodyne_variable_scan_message *message,
										 carmen_handler_t handler,
										 carmen_subscribe_t subscribe_how,
										 int sensor_id)
{
	static char message_name[64];
	carmen_velodyne_create_variable_velodyne_message_name(sensor_id, message_name);
	
	carmen_subscribe_message(message_name,
			(char*) CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT,
			message, sizeof(carmen_velodyne_variable_scan_message),
			handler, subscribe_how);
}


void
carmen_velodyne_unsubscribe_variable_scan_message(carmen_handler_t handler, int sensor_id)
{
	static char message_name[64];
	carmen_velodyne_create_variable_velodyne_message_name(sensor_id, message_name);
	carmen_unsubscribe_message(message_name, handler);
}


IPC_RETURN_TYPE
carmen_velodyne_publish_variable_scan_message(carmen_velodyne_variable_scan_message *message, int sensor_id)
{
	IPC_RETURN_TYPE err;

	static char message_name[64];
	carmen_velodyne_create_variable_velodyne_message_name(sensor_id, message_name);
	err = IPC_publishData(message_name, message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);

	return err;
}

void
carmen_velodyne_subscribe_gps_message(carmen_velodyne_gps_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_VELODYNE_GPS_MESSAGE_NAME,
			(char*) CARMEN_VELODYNE_GPS_MESSAGE_FMT,
			message, sizeof(carmen_velodyne_gps_message),
			handler, subscribe_how);
}


void
carmen_velodyne_unsubscribe_gps_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*) CARMEN_VELODYNE_GPS_MESSAGE_NAME, handler);
}


IPC_RETURN_TYPE
carmen_velodyne_publish_gps_message(carmen_velodyne_gps_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VELODYNE_GPS_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_GPS_MESSAGE_NAME);

	return err;
}


void
carmen_velodyne_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_GPS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_GPS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_GPS_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_NAME);

	char message_name[64];

	for (int i = 0; i <= 9; i++)
	{
		carmen_velodyne_create_variable_velodyne_message_name(i, message_name);
		err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", message_name);
	}
}


double
carmen_velodyne_estimate_shot_time(double sensor_last_timestamp, double sensor_timestamp, int shot_index, int number_of_shots)
{
	return sensor_last_timestamp + (sensor_timestamp - sensor_last_timestamp) * ((double) shot_index / (double) number_of_shots);
}


void
carmen_velodyne_partial_scan_update_points(carmen_velodyne_partial_scan_message *velodyne_message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp)
{
	points->timestamp = timestamp;

	for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		for (int j = 0; j < vertical_resolution; j++)
		{
			double angle = carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);
			double range = (double) (velodyne_message->partial_scan[i].distance[ray_order[j]]) / 500.0;

			if (range <= 0.0)  // @@@ Aparentemente o Velodyne diz range zero quando de range_max
				range = range_max;

			intensity[i * vertical_resolution + j] = velodyne_message->partial_scan[i].intensity[ray_order[j]];
			points->sphere_points[i * vertical_resolution + j].horizontal_angle = carmen_normalize_theta(-angle);
			points->sphere_points[i * vertical_resolution + j].vertical_angle = carmen_degrees_to_radians(vertical_correction[j]);
			points->sphere_points[i * vertical_resolution + j].length = range;
		}
	}
}


void
carmen_velodyne_partial_scan_update_points_with_remission_check(carmen_velodyne_partial_scan_message *velodyne_message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp, int use_remission)
{
	points->timestamp = timestamp;

	for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		for (int j = 0; j < vertical_resolution; j++)
		{
			double angle = carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle);
			double range = (double) (velodyne_message->partial_scan[i].distance[ray_order[j]]) / 500.0;

			if (range <= 0.0)  // @@@ Aparentemente o Velodyne diz range zero quando de range_max
				range = range_max;

			points->sphere_points[i * vertical_resolution + j].horizontal_angle = carmen_normalize_theta(-angle);
			points->sphere_points[i * vertical_resolution + j].vertical_angle = carmen_degrees_to_radians(vertical_correction[j]);
			points->sphere_points[i * vertical_resolution + j].length = range;
		}
	}

	if (use_remission)
	{
		for (int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
			for (int j = 0; j < vertical_resolution; j++)
				intensity[i * vertical_resolution + j] = velodyne_message->partial_scan[i].intensity[ray_order[j]];
	}
}


void
carmen_velodyne_variable_scan_update_points(carmen_velodyne_variable_scan_message *message,
		int vertical_resolution, spherical_point_cloud *points, unsigned char *intensity,
		int *ray_order, double *vertical_correction, float range_max, double timestamp)
{
	points->timestamp = timestamp;

	for (int i = 0; i < message->number_of_shots; i++)
	{
		for (int j = 0; j < vertical_resolution; j++)
		{
			double angle = carmen_degrees_to_radians(message->partial_scan[i].angle);
			double range = (double) (message->partial_scan[i].distance[ray_order[j]]) / 500.0;

			if (range <= 0.0)  // O Velodyne diz range zero quando ocorre range_max
				range = range_max;

			intensity[i * vertical_resolution + j] = message->partial_scan[i].intensity[j];
			points->sphere_points[i * vertical_resolution + j].horizontal_angle = carmen_normalize_theta(-angle);
			points->sphere_points[i * vertical_resolution + j].vertical_angle = carmen_degrees_to_radians(vertical_correction[j]);
			points->sphere_points[i * vertical_resolution + j].length = range;
		}
	}
}



carmen_velodyne_variable_scan_message*
carmen_velodyne_alloc_variable_velodyne_message_and_shots(int n_shots)
{
	carmen_velodyne_variable_scan_message *message =
			(carmen_velodyne_variable_scan_message *)
				calloc(1, sizeof(carmen_velodyne_variable_scan_message));

	message->partial_scan = (carmen_velodyne_shot*) calloc (n_shots, sizeof(carmen_velodyne_shot));
	return message;
}


void
carmen_velodyne_alloc_shot(carmen_velodyne_shot *shot, int shot_size)
{
	shot->distance = (unsigned short*) calloc (shot_size, sizeof(unsigned short));
	shot->intensity = (unsigned char*) calloc (shot_size, sizeof(unsigned char));
}


carmen_velodyne_variable_scan_message*
carmen_velodyne_copy_variable_velodyne_message(
		carmen_velodyne_variable_scan_message *message)
{
	carmen_velodyne_variable_scan_message *copy =
			carmen_velodyne_alloc_variable_velodyne_message_and_shots(message->number_of_shots);

	copy->host = message->host;
	copy->timestamp = message->timestamp;
	copy->number_of_shots = message->number_of_shots;

	for (int i = 0; i < message->number_of_shots; i++)
	{
		copy->partial_scan[i].angle = message->partial_scan[i].angle;
		copy->partial_scan[i].shot_size = message->partial_scan[i].shot_size;

		carmen_velodyne_alloc_shot(&(copy->partial_scan[i]), message->partial_scan[i].shot_size);

		memcpy(copy->partial_scan[i].distance, message->partial_scan[i].distance, message->partial_scan[i].shot_size * sizeof(unsigned short));
		memcpy(copy->partial_scan[i].intensity, message->partial_scan[i].intensity, message->partial_scan[i].shot_size * sizeof(unsigned char));
	}

	return copy;
}


void
carmen_velodyne_free_variable_velodyne_message(
		carmen_velodyne_variable_scan_message *message)
{
	for (int i = 0; i < message->number_of_shots; i++)
	{
		free(message->partial_scan[i].distance);
		free(message->partial_scan[i].intensity);
	}

	free(message->partial_scan);
	free(message);
}
