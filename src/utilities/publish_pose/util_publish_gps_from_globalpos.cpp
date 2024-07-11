#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps.h>


double UtmZone = 24;
int UtmHemiN = 1;
carmen_vector_3D_t gps_position;


carmen_xsens_mtig_message
gps_frame_transform(carmen_xsens_mtig_message xsens_message)
{
	return xsens_message;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_xsens_gps_message(carmen_xsens_mtig_message message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_XSENS_MTIG_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_MTIG_NAME);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *message)
{
	carmen_xsens_mtig_message xsens_message;

	xsens_message.velocity = message->velocity;
	xsens_message.timestamp = message->timestamp;

	Gdc_Coord_3d gdc = carmen_Utm_Gdc2(message->globalpos.x, message->globalpos.y, 0, UtmZone, UtmHemiN);

	xsens_message.latitude = gdc.latitude;
	xsens_message.longitude = gdc.longitude;
	xsens_message.height = gdc.elevation;

	xsens_message = gps_frame_transform(xsens_message);

	publish_xsens_gps_message(xsens_message);
}


static void
publish_gps_shutdown(int signal)
{
	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from IPC. signal = %d\n", signal);
		done = 1;
	}
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) (carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
}


void
define_messages()
{

}


static void
read_parameters(int argc, char **argv)
{
	if (argc > 1)
	{
		UtmZone = (double) atof(argv[1]);
	}
	if (argc > 2)
	{
		UtmHemiN = atoi(argv[2]);
	}

	int num_items;

	carmen_param_t param_list[] =
	{
        {(char *)"gps",	(char *)"nmea_1_x", CARMEN_PARAM_DOUBLE, &gps_position.x, 1, NULL},
		{(char *)"gps",	(char *)"nmea_1_y", CARMEN_PARAM_DOUBLE, &gps_position.y, 1, NULL},
		{(char *)"gps",	(char *)"nmea_1_z", CARMEN_PARAM_DOUBLE, &gps_position.z, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}
///////////////////////////////////////////////////////////////////////////////////////////////////


void
print_parameters_publish_gps()
{
	printf("UtmZone = %lf\n", UtmZone);
	printf("UtmHemiN = %d\n", UtmHemiN);
}


int
main(int argc, char **argv)
{
	if (argc > 3)
	{
		printf("Use %s <UtmZone> <UtmHemiN>\n", argv[0]);
		exit(-1);
	}

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	define_messages();
	subscribe_messages();
	print_parameters_publish_gps();

	signal(SIGINT, publish_gps_shutdown);

	carmen_ipc_dispatch();

	return (0);
}
