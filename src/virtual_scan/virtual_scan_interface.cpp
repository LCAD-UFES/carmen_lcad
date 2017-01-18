#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/virtual_scan_interface.h>

/** \brief Minimum ray length. Values below this limit are assumed to be spurious. */
static const double d_min = 3.0;

/** \brief Maximum ray length. Values beyond this limit are assumed to be spurious. */
static double d_max;

carmen_virtual_scan_message *carmen_virtual_scan_new_message(void)
{
	carmen_virtual_scan_message *message = (carmen_virtual_scan_message*) malloc(sizeof(carmen_virtual_scan_message));
	carmen_virtual_scan_init_message(message);
	return message;
}

void carmen_virtual_scan_init_message(carmen_virtual_scan_message *message)
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
	char *host = carmen_get_host();
	message->host = (char*) malloc(strlen(host) * sizeof(char));
	strcpy(message->host, host);

	message->timestamp = 0;
	message->num_rays = 0;
	message->ranges = NULL;
	message->angles = NULL;
	message->intensity = NULL;
}

#define FREE(BUFFER) if (BUFFER != NULL) {free(BUFFER); BUFFER = NULL;}

void carmen_virtual_scan_clear_message(carmen_virtual_scan_message *message)
{
	message->timestamp = 0;
	message->num_rays = 0;
	FREE(message->host);
	FREE(message->angles);
	FREE(message->ranges);
	FREE(message->intensity);
}

inline double distance(const carmen_velodyne_32_laser_shot &column, int j)
{
	static const int column_indexes[] =
	{
		0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
		1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31
	};

	const int k = column_indexes[j];
	const double d = column.distance[k];
	return d / 500.0;
}

inline bool valid_range(double d)
{
	return (d_min < d && d < d_max);
}

inline double vertical_angle(const double *angles, int j)
{
	return carmen_degrees_to_radians(angles[j]);
}

#define RESIZE(BUFFER, TYPE, SIZE) BUFFER = (TYPE*) realloc(BUFFER, SIZE * sizeof(TYPE))

void carmen_virtual_scan_set_message(carmen_virtual_scan_message *message, carmen_velodyne_partial_scan_message *velodyne_message)
{
	static const double *vertical_angles = NULL;
	if (vertical_angles == NULL)
		vertical_angles = carmen_velodyne_get_vertical_correction();

	// TODO: read from .ini file.
	// Height of the velodyne sensor, measured from the ground.
	static const double h_car = 1.725;

	// Range of vertical scan rays to process.
	// the 22nd ray is at angle 0.0 (i.e. parallel to the ground).
	// The next rays record objects above the car (trees, etc).
	static const int n = 23;

	int m = velodyne_message->number_of_32_laser_shots;

	message->num_rays = m;
	RESIZE(message->angles, double, m);
	RESIZE(message->ranges, double, m);
	RESIZE(message->intensity, double, m);

	message->timestamp = velodyne_message->timestamp;

	for (int i = 0; i < m; i++)
	{
		const carmen_velodyne_32_laser_shot &column = velodyne_message->partial_scan[i];

		// Velodyne angles grow clockwise.
		message->angles[i] = carmen_normalize_theta(carmen_degrees_to_radians(-column.angle));
		message->ranges[i] = 0.0;
		message->intensity[i] = 0.0;

		double d = d_max;

		for (int j = 0; j < n; j++)
		{
			double d1 = distance(column, j);
			double d2 = distance(column, j + 1);
			if (!valid_range(d1) || !valid_range(d2))
				continue;

			double t1 = vertical_angle(vertical_angles, j);
			double t2 = vertical_angle(vertical_angles, j + 1);

			double d1_xy = d1 * cos(t1);
			double d2_xy = d2 * cos(t2);

			double delta_xy = d2_xy - d1_xy;
			double tan_next = tan(atan(h_car / d1_xy) - carmen_normalize_theta(t2 - t1));
			double delta_expected = (h_car - d1_xy * tan_next) / tan_next;
			double delta_ratio = delta_xy / delta_expected;

			if (delta_xy < delta_expected && delta_ratio < 0.5 && valid_range(d2_xy) && d2_xy < d)
				d = d2_xy;
		}

		if (d < d_max)
		{
			message->intensity[i] = 1.0;
			message->ranges[i] = d;
		}
	}
}

void carmen_virtual_scan_subscribe_message(carmen_virtual_scan_message *message,
										   carmen_handler_t handler,
										   carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_VIRTUAL_SCAN_MESSAGE_NAME,
							 (char*) CARMEN_VIRTUAL_SCAN_MESSAGE_FMT,
							 message, sizeof(carmen_virtual_scan_message),
							 handler,
							 subscribe_how);
}

void carmen_virtual_scan_publish_message(carmen_virtual_scan_message *message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VIRTUAL_SCAN_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VIRTUAL_SCAN_MESSAGE_NAME);
}

void carmen_virtual_scan_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*) CARMEN_VIRTUAL_SCAN_MESSAGE_NAME, handler);
}

void carmen_virtual_scan_define_messages(void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_VIRTUAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VIRTUAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VIRTUAL_SCAN_MESSAGE_NAME);
}

void carmen_virtual_scan_install_params(int argc, char *argv[])
{
	carmen_param_t laser_param_list[] =
	{
		{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, &d_max, 0, NULL}
	};

	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
}

static void carmen_velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static carmen_virtual_scan_message *message = NULL;
	if (message == NULL)
		message = carmen_virtual_scan_new_message();

	carmen_virtual_scan_set_message(message, velodyne_message);

	carmen_virtual_scan_publish_message(message);
}

void carmen_virtual_scan_subscribe_messages(void)
{
	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) carmen_velodyne_handler, CARMEN_SUBSCRIBE_LATEST);
}
