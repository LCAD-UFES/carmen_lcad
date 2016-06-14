/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/
#include <err.h>
#include <sickldmrs.h>
#include <carmen/carmen.h>
#include <carmen/laser_ldmrs_interface.h>

volatile int done = 0;
static int last_frame = -1;
static char *laser_ldmrs_port = 0;
static char *laser_ldmrs_address = 0;

/*********************************************************
		   --- Publishers ---
 **********************************************************/

void publish_heartbeats()
{
	carmen_publish_heartbeat("laser_ldmrs");
}

/*********************************************************
		   --- Handlers ---
 **********************************************************/

static void shutdown_module(int signo)
{
	(void) signo;
	done = 1;
	printf("laser_ldmrs: disconnected.\n");
}

static int carmen_laser_ldmrs_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{"laser_ldmrs", "address", CARMEN_PARAM_STRING, &laser_ldmrs_address, 0, NULL},
			{"laser_ldmrs", "port", CARMEN_PARAM_STRING, &laser_ldmrs_port, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

static void carmen_laser_ldmrs_copy_message(struct sickldmrs_scan *scan, carmen_laser_ldmrs_message *message)
{
	int i;

	message->scan_number = scan->scan_number;
	message->scanner_status = scan->scanner_status;
	message->sync_phase_offset = scan->sync_phase_offset;
//	message->scan_start_time = scan->scan_start_time;
//	message->scan_end_time = scan->scan_end_time;
	message->angle_ticks_per_rotation = scan->angle_ticks_per_rotation;
	message->start_angle = scan->start_angle;
	message->end_angle = scan->end_angle;
	message->mount_yaw = scan->mount_yaw;
	message->mount_pitch = scan->mount_pitch;
	message->mount_roll = scan->mount_roll;
	message->mount_x = scan->mount_x;
	message->mount_y = scan->mount_y;
	message->mount_z = scan->mount_z;
	message->flags = scan->flags;
	if(message->scan_points != scan->scan_points)
	{
		message->scan_points = scan->scan_points;
		message->points = (carmen_laser_ldmrs_point *)realloc(message->points, message->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(message->points);
	}
	for(i = 0; i < message->scan_points; i++)
	{
		message->points[i].layer = scan->points[i].layer;
		message->points[i].echo = scan->points[i].echo;
		message->points[i].flags = scan->points[i].flags;
		message->points[i].horizontal_angle = scan->points[i].horizontal_angle;
		message->points[i].radial_distance = scan->points[i].radial_distance;
		message->points[i].pulse_width = scan->points[i].pulse_width;
	}
}

int main(int argc, char **argv)
{
	static carmen_laser_ldmrs_message message;
	struct sickldmrs_device *dev;
	int rc;

	message.scan_points = 0;

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	/* Read application specific parameters (Optional) */
	carmen_laser_ldmrs_read_parameters(argc, argv);

	/* Define published messages by your module */
	carmen_laser_define_ldmrs_messages();

	carmen_ipc_addPeriodicTimer(10, publish_heartbeats, NULL);

	dev = sickldmrs_init(laser_ldmrs_address, laser_ldmrs_port, true);
	if (dev != NULL)
	{
		//dev->debug = 1;
		if ((rc = sickldmrs_get_status(dev, -1)) < 0)
			errx(2, "sickldmrs_get_status: %s\n", strerror(-rc));
		if ((rc = sickldmrs_config_output(dev, 0x00ee, -1)) < 0)
			errx(2, "sickldmrs_config_output: %s\n", strerror(rc));
		/* scan frequency -> 25 Hz */
		if ((rc = sickldmrs_set_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, 6400, -1)) < 0)
			errx(2, "sickldmrs_set_parameter: %s", strerror(rc));
		if ((rc = sickldmrs_get_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, -1)) < 0)
			errx(2, "sickldmrs_get_parameter: %s", strerror(rc));

		while (!done)
		{
			carmen_ipc_sleep(1./25.);	/* 25Hz */

			sickldmrs_lock(dev);
			if (dev->scan != NULL && dev->scan->scan_number != last_frame)
			{
				//sickldmrs_print_scan(dev->scan); //DEBUG
				carmen_laser_ldmrs_copy_message(dev->scan, &message);
				carmen_laser_publish_ldmrs(&message);
				last_frame = dev->scan->scan_number;
			}
			sickldmrs_unlock(dev);
		}
		sickldmrs_end(dev);
	}

	carmen_ipc_disconnect();

	return 0;
}
