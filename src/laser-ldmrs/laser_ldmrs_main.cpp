/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/
#include <err.h>
#include <sickldmrs.h>
#include <vpLaserScan.h>
#include <carmen/carmen.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/base_ackerman_messages.h>

volatile int done = 0;
//static int last_frame = -1;
static char *laser_ldmrs_port = 0;
static char *laser_ldmrs_address = 0;
vpSickLDMRS laser;

/*********************************************************
		   --- Publishers ---
 **********************************************************/

void publish_heartbeats(void *clientData,
		   unsigned long currentTime,
		   unsigned long scheduledTime)
{
	(void)clientData;
	(void)currentTime;
	(void)scheduledTime;
	carmen_publish_heartbeat((char*)"laser_ldmrs");
}

/*********************************************************
		   --- Handlers ---
 **********************************************************/

static void shutdown_module(int signo)
{
	(void) signo;
	done = 1;
	carmen_ipc_disconnect();
	printf("laser_ldmrs: disconnected.\n");
	exit(0);
}

static int carmen_laser_ldmrs_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char*)"laser_ldmrs", (char*)"address", CARMEN_PARAM_STRING, &laser_ldmrs_address, 0, NULL},
			{(char*)"laser_ldmrs", (char*)"port", CARMEN_PARAM_STRING, &laser_ldmrs_port, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *odometry_message)
{
	short velocity_cms = short (odometry_message->v * 100.0);
	short phi_mrad = short (odometry_message->phi * 1000);
	short yaw_rate = 0;

	laser.sendEgoMotionData(velocity_cms, phi_mrad, yaw_rate);
}

/*
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

	// Connect to IPC Server
	carmen_ipc_initialize(argc, argv);

	// Check the param server version
	carmen_param_check_version(argv[0]);

	// Register shutdown cleaner handler
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	// Read application specific parameters (Optional)
	carmen_laser_ldmrs_read_parameters(argc, argv);

	// Define published messages by your module
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
		// scan frequency -> 25 Hz
		if ((rc = sickldmrs_set_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, 6400, -1)) < 0)
			errx(2, "sickldmrs_set_parameter: %s", strerror(rc));
		if ((rc = sickldmrs_get_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, -1)) < 0)
			errx(2, "sickldmrs_get_parameter: %s", strerror(rc));

		while (!done)
		{
			carmen_ipc_sleep(1./25.);	// 25Hz

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
*/

static void carmen_laser_ldmrs_copy_message(vpLaserScan laserscan[4], carmen_laser_ldmrs_message *message)
{
	message->scan_number = laserscan[0].getMeasurementId();
	message->scan_start_time = laserscan[0].getStartTimestamp();
	message->scan_end_time = laserscan[0].getEndTimestamp();
	message->angle_ticks_per_rotation = laserscan[0].getNumSteps();
	message->start_angle = laserscan[0].getStartAngle();
	message->end_angle = laserscan[0].getStopAngle();

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	//printf("size1: %d, size2: %d, size3: %d, size4: %d\n",sizeLayer1,sizeLayer2,sizeLayer3,sizeLayer4);

	if(message->scan_points != laserscan[0].getNumPoints())
	{
		message->scan_points = laserscan[0].getNumPoints();
		message->arraypoints = (carmen_laser_ldmrs_point *)realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(message->arraypoints);
	}

	for(int i = 0; i < sizeLayer1; i++)
	{
		message->arraypoints[i].horizontal_angle = pointsInLayer1[i].getHAngle();
		message->arraypoints[i].vertical_angle = pointsInLayer1[i].getVAngle();
		message->arraypoints[i].radial_distance = pointsInLayer1[i].getRadialDist();
	}

	for(int i = 0; i < sizeLayer2; i++)
	{
		message->arraypoints[i + sizeLayer1].horizontal_angle = pointsInLayer2[i].getHAngle();
		message->arraypoints[i + sizeLayer1].vertical_angle = pointsInLayer2[i].getVAngle();
		message->arraypoints[i + sizeLayer1].radial_distance = pointsInLayer2[i].getRadialDist();
	}

	for(int i = 0; i < sizeLayer3; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2].horizontal_angle = pointsInLayer3[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].vertical_angle = pointsInLayer3[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].radial_distance = pointsInLayer3[i].getRadialDist();
	}

	for(int i = 0; i < sizeLayer4; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].horizontal_angle = pointsInLayer4[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].vertical_angle = pointsInLayer4[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].radial_distance = pointsInLayer4[i].getRadialDist();
	}
}

static void carmen_laser_ldmrs_objects_build_message(vpLaserObjectData *objectData, carmen_laser_ldmrs_objects_message *message)
{
	message->num_objects = objectData->getNumObjects();
	std::vector<vpObject> objectsList = objectData->getObjectList();
	message->objects_list = (carmen_laser_ldmrs_object *) malloc(message->num_objects * sizeof(carmen_laser_ldmrs_object));
	for(int i = 0; i < message->num_objects; i++){
		message->objects_list[i].id = objectsList[i].getObjectId();
		message->objects_list[i].x = 0.01 * objectsList[i].getObjectBoxCenter().x_pos;
		message->objects_list[i].y = 0.01 * objectsList[i].getObjectBoxCenter().y_pos;
		int xv = objectsList[i].getAbsoluteVelocity().x_pos;
		int yv = objectsList[i].getAbsoluteVelocity().y_pos;
		double velocity = 0.01 * sqrt(xv*xv + yv*yv);
		message->objects_list[i].velocity = velocity;
		message->objects_list[i].orientation = (((double) objectsList[i].getObjectBoxOrientation())/32.0)*M_PI/180;
		message->objects_list[i].lenght = 0.01 * objectsList[i].getObjectBoxSize().x_size;
		message->objects_list[i].width = 0.01 * objectsList[i].getObjectBoxSize().y_size;
	}
}

int main(int argc, char **argv)
{
	static carmen_laser_ldmrs_message message;
	static carmen_laser_ldmrs_objects_message objectsMessage;

	message.scan_points = 0;

	// Connect to IPC Server
	carmen_ipc_initialize(argc, argv);

	// Check the param server version
	carmen_param_check_version(argv[0]);

	// Register shutdown cleaner handler
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	// Read application specific parameters (Optional)
	carmen_laser_ldmrs_read_parameters(argc, argv);

	// Define published messages by your module
	carmen_laser_define_ldmrs_messages();

	carmen_ipc_addPeriodicTimer(10, publish_heartbeats, NULL);

	// Subscribe to odometry messages
	carmen_base_ackerman_subscribe_odometry_message(NULL,
    		(carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

	laser.setIpAddress(laser_ldmrs_address);
	laser.setPort(atoi(laser_ldmrs_port));
	laser.setup();

	vpLaserScan laserscan[4];
	//vpLaserObjectData objectData;

	for ( ; ; ) {

		vpLaserObjectData objectData;
		if(laser.tracking(&objectData) == true)
		{
			carmen_laser_ldmrs_objects_build_message(&objectData, &objectsMessage);

			carmen_laser_publish_ldmrs_objects(&objectsMessage);
		}

		// Get the measured points in the four layers
		if (laser.measure(laserscan) == false)
			continue;

		carmen_laser_ldmrs_copy_message(laserscan, &message);
		// Prints all the measured points
		/*
		for (int layer=0; layer<4; layer++) {
			std::vector<vpScanPoint> pointsInLayer = laserscan[layer].getScanPoints();
			vpScanPoint p;

			for (unsigned int i=0; i < pointsInLayer.size(); i++) {
				std::cout << pointsInLayer[i] << std::endl;
			}
		}
		*/
		carmen_laser_publish_ldmrs(&message);
	}
	carmen_ipc_disconnect();

	return 0;
}
