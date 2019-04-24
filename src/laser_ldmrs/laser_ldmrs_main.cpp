/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/
#include <err.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sickldmrs-private.h>
#include <sickldmrs.h>

#include <sickldmrs.h>
#include <carmen/carmen.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/stereo_velodyne_interface.h>

#include "laser_ldmrs_utils.h"

volatile int done = 0;
static int last_frame = -1;

static char *laser_ldmrs_port = 0;
static char *laser_ldmrs_address = 0;
static double axle_distance = 0.0;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
shutdown_module(int signo)
{
	if ((signo == SIGINT) || (signo == SIGTERM))
	{
		carmen_ipc_disconnect();
		printf("laser_ldmrs: disconnected.\n");
		done++;
	}
	usleep(400000);
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////


static int
carmen_laser_ldmrs_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "laser_ldmrs", 	(char*)	"address", 	CARMEN_PARAM_STRING, &laser_ldmrs_address, 0, NULL},
			{(char *) "laser_ldmrs", 	(char*)	"port", 	CARMEN_PARAM_STRING, &laser_ldmrs_port, 0, NULL},
			{(char *) "robot", 			(char*) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &axle_distance, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


struct sickldmrs_device *
setup_ldmrs_laser(char *address, char *port)
{
	int rc;
	struct sickldmrs_device *dev;

	dev = sickldmrs_init(address, port, true);

	if (dev == NULL)
	{
		printf("Could not initialize the Sick LDMRS Lidar\n");
		exit(2);
	}

	dev->debug = 1;
	if ((rc = sickldmrs_get_status(dev, -1)) < 0)
		errx(2, "sickldmrs_get_status: %s\n", strerror(-rc));

	if ((rc = sickldmrs_config_output(dev, 0x00ee, -1)) < 0)
		errx(2, "sickldmrs_config_output: %s\n", strerror(rc));

	/* scan frequency -> 25 Hz */
	if ((rc = sickldmrs_set_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, 6400, -1)) < 0)
		errx(2, "sickldmrs_set_parameter: %s", strerror(rc));

	if ((rc = sickldmrs_get_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, -1)) < 0)
		errx(2, "sickldmrs_get_parameter: %s", strerror(rc));

	usleep(40000);
	dev->priv->done = 1;
	dev->debug = 0;

	return (dev);
}


int
main(int argc, char **argv)
{
	static carmen_laser_ldmrs_new_message message;
	struct sickldmrs_device *dev;
	char *address, *port;

	address = argv[1];
	port = argv[2];

	dev = setup_ldmrs_laser(address, port);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);
	carmen_laser_ldmrs_read_parameters(argc, argv);
	carmen_laser_define_ldmrs_new_messages();

	while (!done)
	{
		int rv = sickldmrs_receive_frame(dev, -1);
		if (rv != 0 && errno == ETIMEDOUT)
			continue;
		if (dev->scan != NULL && dev->scan->scan_number != last_frame)
		{
			last_frame = dev->scan->scan_number;
			message.timestamp = carmen_get_time();
			carmen_laser_ldmrs_new_copy_laser_scan_to_message(&message, dev->scan);
			if (dev->scan->scan_points > 0)
				carmen_laser_publish_ldmrs_new(&message);
		}
	}
	sickldmrs_end(dev);
	printf("bye\n");
	return 0;
}
