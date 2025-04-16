#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps.h>


double UtmZone = 24;

// Se UtmHemiN = 1, hemisfério é norte, caso contrário é sul
int UtmHemiN = 0;
carmen_vector_3D_t gps_position;


carmen_gps_gpgga_message
gps_frame_transform(carmen_gps_gpgga_message nmea_message)
{
	return nmea_message;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_nmea_gps_message(carmen_gps_gpgga_message message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_GPS_GPGGA_MESSAGE_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_GPGGA_MESSAGE_NAME);
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
	// NMEAGGA 1 174910.100000 2016.3327512 S 4018.3876823 W 5 12 2.090000 3.001000 0.000000 -7.074000 0.000000 1 1685123350.263025 gps_driver@breno-Predator-PH315-54 230.727233
	// int nr; /**< number of the gps unit **/
	// double utc; /**< Universal Time Coordinated (UTC) **/
	// double latitude; /**< latitude as a real floating point number **/
	// double latitude_dm; /**< latitude as DDMM.MMMM format (as in the NMEA text message) **/
	// char lat_orient; /**< N or S (North or South) **/
	// double longitude; /**< longitude as a real floating point number **/
	// double longitude_dm; /**< longitude as DDMM.MMMM format (as in the NMEA text message) **/
	// char long_orient; /**< E or W (East or West) **/
	// int gps_quality; /**< GPS Quality Indicator,
	// 	         0 - fix not available,
	// 	         1 - GPS fix,
	// 	         2 - Differential GPS fix, OmniSTAR VBS
	// 			 3 - ? (it is not in the Trimble documentation in http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_GGA.html
	// 			 4 - Real-Time Kinematic, fixed integers
	// 			 5 - Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK **/
	// int num_satellites; /**< Number of satellites in view, 00-12 **/
	// double hdop; /**< Horizontal Dilution of precision **/
	// double sea_level; /**< Antenna Altitude above/below 
	// 		 mean-sea-level (geoid) **/
	// double altitude; /**< Units of antenna altitude, meters **/
	// double geo_sea_level; /**< Geoidal separation, the difference
	// 			 between the WGS-84 earth ellipsoid and
	// 			 mean-sea-level (geoid), "-" means
	// 			 mean-sea-level below ellipsoid **/
	// double geo_sep; /**< Units of geoidal separation, meters **/
	// int data_age; /**< Age of differential GPS data, time 
	// 		 in seconds since last SC104 type 1 or
	// 		 9 update, null field when DGPS is not
	// 		 used **/
	// double timestamp;
	// char *host;

	// 7757691.01 -363593.38 0.588 - Coordenada UTM portão LCAD-4

	carmen_gps_gpgga_message nmea_message = {};

	nmea_message.timestamp = message->timestamp;
	nmea_message.host = carmen_get_host();

	// Gdc_Coord_3d gdc = carmen_Utm_Gdc2(363593.38, 7757691.01, 0, UtmZone, UtmHemiN);
	Gdc_Coord_3d gdc = carmen_Utm_Gdc2(-message->globalpos.y, message->globalpos.x, 0, UtmZone, UtmHemiN);

	nmea_message.nr = 1;
	nmea_message.utc = 0.0;
	nmea_message.latitude = -gdc.latitude;
	nmea_message.latitude_dm = carmen_global_convert_double_to_degmin(nmea_message.latitude);
	nmea_message.longitude = -gdc.longitude;
	nmea_message.longitude_dm = carmen_global_convert_double_to_degmin(nmea_message.longitude);
	nmea_message.altitude = gdc.elevation;
	nmea_message.lat_orient = 'S';
	nmea_message.long_orient = 'W';

	nmea_message = gps_frame_transform(nmea_message);

	publish_nmea_gps_message(nmea_message);
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

	// TODO: usar esses valores para levar globalpos para posição do gps no veículo
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
