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

static char *laser_ldmrs_port = 0;
static char *laser_ldmrs_address = 0;
static double axle_distance = 0.0;

static carmen_base_ackerman_odometry_message odometry_message;
vpSickLDMRS laser;



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_variable_velodyne_message(vpLaserScan laserscan[4])
{
	carmen_velodyne_variable_scan_message velodyne_message;

	double start_angle_deg = carmen_radians_to_degrees(2*M_PI*laserscan[0].getStartAngle() /(double) laserscan[0].getNumSteps());
	double end_angle_deg = carmen_radians_to_degrees(2*M_PI*laserscan[0].getStopAngle() /(double) laserscan[0].getNumSteps());
	int number_of_shots = (start_angle_deg - end_angle_deg) / 0.5;


	velodyne_message.number_of_shots = number_of_shots;
	velodyne_message.partial_scan = (carmen_velodyne_shot *) malloc (velodyne_message.number_of_shots * sizeof(carmen_velodyne_shot));
	velodyne_message.timestamp = carmen_get_time();
	velodyne_message.host = carmen_get_host();


	// alocacao
	for (int i = 0; i < velodyne_message.number_of_shots; i++)
	{
		velodyne_message.partial_scan[i].shot_size = 4;
		velodyne_message.partial_scan[i].distance = (unsigned short*) calloc (4, sizeof(unsigned short));
		velodyne_message.partial_scan[i].intensity = (unsigned char*) calloc (4, sizeof(unsigned char));
		velodyne_message.partial_scan[i].angle = start_angle_deg + 0.5 * i;
	}


	if(laserscan[0].getNumPoints() == 0) {
		return;
	}

	int num_of_points = laserscan[0].getNumPoints();

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	if(num_of_points > (sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4)){
		num_of_points = sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4;
	}

//	if(message->scan_points != num_of_points)
//	{
//		message->scan_points = num_of_points;
//		message->arraypoints = (carmen_laser_ldmrs_point *)realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_point));
//		carmen_test_alloc(message->arraypoints);
//	}
//
//	int i;
//	for(i = 0; i < sizeLayer1; i++)
//	{
//		message->arraypoints[i].horizontal_angle = pointsInLayer1[i].getHAngle();
//		message->arraypoints[i].vertical_angle = pointsInLayer1[i].getVAngle();
//		message->arraypoints[i].radial_distance = pointsInLayer1[i].getRadialDist();
//		message->arraypoints[i].flags = pointsInLayer1[i].getFlags();
//	}
//
//	for(i = 0; i < sizeLayer2; i++)
//	{
//		message->arraypoints[i + sizeLayer1].horizontal_angle = pointsInLayer2[i].getHAngle();
//		message->arraypoints[i + sizeLayer1].vertical_angle = pointsInLayer2[i].getVAngle();
//		message->arraypoints[i + sizeLayer1].radial_distance = pointsInLayer2[i].getRadialDist();
//		message->arraypoints[i + sizeLayer1].flags = pointsInLayer2[i].getFlags();
//	}
//
//	for(i = 0; i < sizeLayer3; i++)
//	{
//		message->arraypoints[i + sizeLayer1 + sizeLayer2].horizontal_angle = pointsInLayer3[i].getHAngle();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2].vertical_angle = pointsInLayer3[i].getVAngle();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2].radial_distance = pointsInLayer3[i].getRadialDist();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2].flags = pointsInLayer3[i].getFlags();
//
//	}
//
//	for(i = 0; i < sizeLayer4; i++)
//	{
//		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].horizontal_angle = pointsInLayer4[i].getHAngle();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].vertical_angle = pointsInLayer4[i].getVAngle();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].radial_distance = pointsInLayer4[i].getRadialDist();
//		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].flags = pointsInLayer4[i].getFlags();
//	}
}


void
publish_heartbeats(void *clientData,
		   unsigned long currentTime,
		   unsigned long scheduledTime)
{
	(void)clientData;
	(void)currentTime;
	(void)scheduledTime;
	carmen_publish_heartbeat((char*)"laser_ldmrs");
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *odometry_message)
{

	short velocity_cms = (short) (odometry_message->v * 100.0);
	short phi_mrad = (short) (odometry_message->phi * 1000);
	short yaw_rate = (short) (odometry_message->v * tan(odometry_message->phi) / axle_distance) * 10000;

	laser.sendEgoMotionData(velocity_cms, phi_mrad, yaw_rate);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("laser_ldmrs: disconnected.\n");
		exit(0);
	}

	if (signo == SIGTERM)
	{
		carmen_ipc_disconnect();
		printf("laser_ldmrs: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


static int
carmen_laser_ldmrs_read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char*)"laser_ldmrs", 	(char*)	"address", 	CARMEN_PARAM_STRING, &laser_ldmrs_address, 0, NULL},
			{(char*)"laser_ldmrs", 	(char*)	"port", 	CARMEN_PARAM_STRING, &laser_ldmrs_port, 0, NULL},
			{(char*)"robot", 		(char*) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &axle_distance, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


/*********************************************************
		   --- Build messages ---
 **********************************************************/

static void
carmen_laser_ldmrs_copy_message(vpLaserScan laserscan[4], carmen_laser_ldmrs_message *message)
{
	message->scan_number = laserscan[0].getMeasurementId();
	message->scan_start_time = laserscan[0].getStartTimestamp();
	message->scan_end_time = laserscan[0].getEndTimestamp();
	message->angle_ticks_per_rotation = laserscan[0].getNumSteps();
	message->start_angle = laserscan[0].getStartAngle();
	message->end_angle = laserscan[0].getStopAngle();

	if(laserscan[0].getNumPoints() == 0) {
		return;
	}

	int num_of_points = laserscan[0].getNumPoints();

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	if(num_of_points > (sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4)){
		num_of_points = sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4;
	}

	if(message->scan_points != num_of_points)
	{
		message->scan_points = num_of_points;
		message->arraypoints = (carmen_laser_ldmrs_point *)realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(message->arraypoints);
	}

	int i;
	for(i = 0; i < sizeLayer1; i++)
	{
		message->arraypoints[i].horizontal_angle = pointsInLayer1[i].getHAngle();
		message->arraypoints[i].vertical_angle = pointsInLayer1[i].getVAngle();
		message->arraypoints[i].radial_distance = pointsInLayer1[i].getRadialDist();
		message->arraypoints[i].flags = pointsInLayer1[i].getFlags();
	}

	for(i = 0; i < sizeLayer2; i++)
	{
		message->arraypoints[i + sizeLayer1].horizontal_angle = pointsInLayer2[i].getHAngle();
		message->arraypoints[i + sizeLayer1].vertical_angle = pointsInLayer2[i].getVAngle();
		message->arraypoints[i + sizeLayer1].radial_distance = pointsInLayer2[i].getRadialDist();
		message->arraypoints[i + sizeLayer1].flags = pointsInLayer2[i].getFlags();
	}

	for(i = 0; i < sizeLayer3; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2].horizontal_angle = pointsInLayer3[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].vertical_angle = pointsInLayer3[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].radial_distance = pointsInLayer3[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].flags = pointsInLayer3[i].getFlags();

	}

	for(i = 0; i < sizeLayer4; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].horizontal_angle = pointsInLayer4[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].vertical_angle = pointsInLayer4[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].radial_distance = pointsInLayer4[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].flags = pointsInLayer4[i].getFlags();
	}
}


static void
carmen_laser_ldmrs_objects_build_message(vpLaserObjectData *objectData, carmen_laser_ldmrs_objects_message *message)
{

	std::vector<vpObject> objectsList = objectData->getObjectList();

	if(objectData->getNumObjects() == 0) {
		return;
	}

	if(message->num_objects != objectData->getNumObjects()) {
		message->num_objects = objectData->getNumObjects();
		message->objects_list = (carmen_laser_ldmrs_object *) realloc(message->objects_list, message->num_objects * sizeof(carmen_laser_ldmrs_object));
		carmen_test_alloc(message->objects_list);
	}

	for(int i = 0; i < message->num_objects; i++)
	{

		message->objects_list[i].id = objectsList[i].getObjectId();
		message->objects_list[i].x = 0.01 * (objectsList[i].getObjectBoxCenter().x_pos);
		message->objects_list[i].y = 0.01 * (objectsList[i].getObjectBoxCenter().y_pos);

		int xv = objectsList[i].getAbsoluteVelocity().x_pos;
		int yv = objectsList[i].getAbsoluteVelocity().y_pos;
		double velocity = 0.01 * sqrt(xv*xv + yv*yv);
		message->objects_list[i].velocity = velocity;
		message->objects_list[i].orientation = (((double) objectsList[i].getObjectBoxOrientation())/32.0)*M_PI/180;
		message->objects_list[i].lenght = 0.01 * objectsList[i].getObjectBoxSize().x_size;
		message->objects_list[i].width = 0.01 * objectsList[i].getObjectBoxSize().y_size;

		message->objects_list[i].classId = objectsList[i].getClassification();
	}
}


static void
carmen_laser_ldmrs_objects_data_build_message(vpLaserObjectData *objectData, carmen_laser_ldmrs_objects_data_message *message)
{

	std::vector<vpObject> objectsList = objectData->getObjectList();

	if(objectData->getNumObjects() == 0) {
		return;
	}

	if(message->num_objects != objectData->getNumObjects()) {
		message->num_objects = objectData->getNumObjects();
		message->objects_data_list = (carmen_laser_ldmrs_object_data *) realloc(message->objects_data_list, message->num_objects * sizeof(carmen_laser_ldmrs_object_data));
		carmen_test_alloc(message->objects_data_list);
	}

	for(int i = 0; i < message->num_objects; i++)
	{
		message->objects_data_list[i].object_id = objectsList[i].getObjectId();
		message->objects_data_list[i].object_age = objectsList[i].getObjectAge();
		message->objects_data_list[i].object_prediction_age = objectsList[i].getObjectPredictionAge();
		message->objects_data_list[i].reference_point_x = 0.01 * (double) objectsList[i].getReferencePoint().x_pos;
		message->objects_data_list[i].reference_point_y = 0.01 * (double) objectsList[i].getReferencePoint().y_pos;;
		message->objects_data_list[i].reference_point_sigma_x = 0.01 * (double) objectsList[i].getReferencePointSigma().x_pos;
		message->objects_data_list[i].reference_point_sigma_y = 0.01 * (double) objectsList[i].getReferencePointSigma().y_pos;;
		message->objects_data_list[i].closest_point_x = 0.01 * (double) objectsList[i].getClosestPoint().x_pos;
		message->objects_data_list[i].closest_point_y = 0.01 * (double) objectsList[i].getClosestPoint().y_pos;;
		message->objects_data_list[i].bounding_box_center_x = 0.01 * (double) objectsList[i].getBoundingBoxCenter().x_pos;
		message->objects_data_list[i].bounding_box_center_y = 0.01 * (double) objectsList[i].getBoundingBoxCenter().y_pos;
		message->objects_data_list[i].bounding_box_length = 0.01 * (double) objectsList[i].getBoundingBoxSize().y_size;
		message->objects_data_list[i].bounding_box_width = 0.01 * (double) objectsList[i].getBoundingBoxSize().x_size;
		message->objects_data_list[i].object_box_center_x = 0.01 * (double) objectsList[i].getObjectBoxCenter().x_pos;
		message->objects_data_list[i].object_box_center_y = 0.01 * (double) objectsList[i].getObjectBoxCenter().y_pos;
		message->objects_data_list[i].object_box_lenght = 0.01 * (double) objectsList[i].getObjectBoxSize().y_size;
		message->objects_data_list[i].object_box_width = 0.01 * (double) objectsList[i].getObjectBoxSize().x_size;
		message->objects_data_list[i].object_box_orientation = carmen_normalize_theta(((double) objectsList[i].getObjectBoxOrientation()/32.0) * M_PI/180.0) ;
		message->objects_data_list[i].abs_velocity_x = 0.01 * (double) objectsList[i].getAbsoluteVelocity().x_pos;
		message->objects_data_list[i].abs_velocity_y = 0.01 * (double) objectsList[i].getAbsoluteVelocity().y_pos;
		message->objects_data_list[i].abs_velocity_sigma_x = 0.01 * (double) objectsList[i].getAbsoluteVelocitySigma().x_size;
		message->objects_data_list[i].abs_velocity_sigma_y = 0.01 * (double) objectsList[i].getAbsoluteVelocitySigma().y_size;
		message->objects_data_list[i].relative_velocity_x = 0.01 * (double) objectsList[i].getRelativeVelocity().x_pos;
		message->objects_data_list[i].relative_velocity_y = 0.01 * (double) objectsList[i].getRelativeVelocity().y_pos;
		message->objects_data_list[i].class_id = objectsList[i].getClassification();
	}
}


int
main(int argc, char **argv)
{
	static carmen_laser_ldmrs_message message;
	static carmen_laser_ldmrs_objects_message objectsMessage;
	static carmen_laser_ldmrs_objects_data_message objectsDataMessage;
	unsigned short dataType;

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

	// setup laser
	laser.setIpAddress(laser_ldmrs_address);
	laser.setPort(atoi(laser_ldmrs_port));
	laser.setup();

	memset(&odometry_message, 0, sizeof(carmen_base_ackerman_odometry_message));
	// Subscribe to odometry messages
	carmen_base_ackerman_subscribe_odometry_message(&odometry_message,
	    		(carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

	for ( ; ; )
	{
		vpLaserObjectData objectData;
		vpLaserScan laserscan[4];

		dataType = laser.readData(laserscan, &objectData);

		switch (dataType)
		{
			case vpSickLDMRS::MeasuredData:
				carmen_laser_ldmrs_copy_message(laserscan, &message);
				if (laserscan[0].getNumPoints() > 0)
					carmen_laser_publish_ldmrs(&message);
				break;
			case vpSickLDMRS::ObjectData:
				carmen_laser_ldmrs_objects_data_build_message(&objectData, &objectsDataMessage);
				//carmen_laser_ldmrs_objects_build_message(&objectData, &objectsMessage);
				if (objectData.getNumObjects() > 0)
					carmen_laser_publish_ldmrs_objects_data(&objectsDataMessage);
				break;
			case 0:
				carmen_ipc_sleep((1.0 / 12.5) / 10.0);
			default:
				break;
		}

	}

	return 0;
}
