#include <carmen/carmen.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <carmen/velodyne_interface.h>
#include <driver2.h>

using namespace std;

velodyne_driver::velodyne_gps_t gps;
velodyne_driver::velodyne_config_t config;
velodyne_driver::VelodyneDriver* velodyne = NULL;

static carmen_velodyne_variable_scan_message velodyne_variable_scan;
static carmen_velodyne_gps_message velodyne_gps;

//static int velodyne_scan_port;
//static int velodyne_gps_port;
static int velodyne_gps_enabled;
static int velodyne_number;

static double velodyne_package_rate;
static double dist_lsb;
static double rotation_resolution;
static unsigned short rotation_max_units;
static int velodyne_num_lasers;							//vertical_resolution
static int velodyne_num_shots;
static int min_sensing_distance;
static int max_sensing_distance;
static int velodyne_msg_buffer_size;
static int velodyne_gps_buffer_size;

static double velodyne_driver_broadcast_freq_hz;
static int velodyne_udp_port; 							//velodyne_scan_port
static int velodyne_gps_udp_port;						//velodyne_gps_port

static unsigned short velodyne_upper_header_bytes;
static unsigned short velodyne_lower_header_bytes;

static double gyro_scale_factor;
static double temp_scale_factor;
static int temp_base_factor;
static double accel_scale_factor;

static double velodyne_min_frequency;
static int velodyne_max_laser_shots_per_revolution;

void assembly_velodyne_gps_message_from_gps(velodyne_driver::velodyne_gps_t gps)
{
	velodyne_gps.gyro1 = (gps.packet.gyro1 & 0x0fff) * gyro_scale_factor;
	velodyne_gps.gyro2 = (gps.packet.gyro2 & 0x0fff) * gyro_scale_factor;
	velodyne_gps.gyro3 = (gps.packet.gyro3 & 0x0fff) * gyro_scale_factor;

	velodyne_gps.temp1 = (gps.packet.temp1 & 0x0fff) * temp_scale_factor + temp_base_factor;
	velodyne_gps.temp2 = (gps.packet.temp2 & 0x0fff) * temp_scale_factor + temp_base_factor;
	velodyne_gps.temp3 = (gps.packet.temp3 & 0x0fff) * temp_scale_factor + temp_base_factor;

	velodyne_gps.accel1_x = (gps.packet.accel1_x & 0x0fff) * accel_scale_factor;
	velodyne_gps.accel2_x = (gps.packet.accel2_x & 0x0fff) * accel_scale_factor;
	velodyne_gps.accel3_x = (gps.packet.accel3_x & 0x0fff) * accel_scale_factor;

	velodyne_gps.accel1_y = (gps.packet.accel1_y & 0x0fff) * accel_scale_factor;
	velodyne_gps.accel2_y = (gps.packet.accel2_y & 0x0fff) * accel_scale_factor;
	velodyne_gps.accel3_y = (gps.packet.accel3_y & 0x0fff) * accel_scale_factor;

	char* nmea_sentence = (char*) gps.packet.nmea_sentence;
	std::vector<std::string> strs;
	boost::split(strs, nmea_sentence, boost::is_any_of(","));

	velodyne_gps.utc_time = atoi(strs[1].c_str());
	velodyne_gps.status = *strs[2].c_str();
	velodyne_gps.latitude = (double) atof(strs[3].c_str());
	velodyne_gps.latitude_hemisphere = *strs[4].c_str();
	velodyne_gps.longitude = (double) atof(strs[5].c_str());
	velodyne_gps.longitude_hemisphere = *strs[6].c_str();
	velodyne_gps.speed_over_ground = (double) atof(strs[7].c_str());
	velodyne_gps.course_over_ground = (double) atof(strs[8].c_str());
	velodyne_gps.utc_date = atoi(strs[9].c_str());
	velodyne_gps.magnetic_variation_course = (double) atof(strs[10].c_str());
	velodyne_gps.magnetic_variation_direction = (double) atof(strs[11].c_str());
	velodyne_gps.mode_indication = *strs[12].c_str();

	velodyne_gps.timestamp = gps.timestamp;
	velodyne_gps.host = carmen_get_host();

//	printf("Gyro    -> 1: %6.2f, 2: %6.2f, 3: %6.2f\n", velodyne_gps.gyro1, velodyne_gps.gyro2, velodyne_gps.gyro3);
//	printf("Temp    -> 1: %6.2f, 2: %6.2f, 3: %6.2f\n", velodyne_gps.temp1, velodyne_gps.temp2, velodyne_gps.temp3);
//	printf("Accel_X -> 1: %6.2f, 2: %6.2f, 3: %6.2f\n", velodyne_gps.accel1_x, velodyne_gps.accel2_x, velodyne_gps.accel3_x);
//	printf("Accel_Y -> 1: %6.2f, 2: %6.2f, 3: %6.2f\n", velodyne_gps.accel1_y, velodyne_gps.accel2_y, velodyne_gps.accel3_y);
}

/*********************************************************
		   --- Publishers ---
**********************************************************/

/*void publish_velodyne_partial_scan()
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, &velodyne_partial_scan);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
}*/

//void publish_velodyne_variable_scan()
//{
//	IPC_RETURN_TYPE err;
//
//	if (velodyne_number == 1)
//	{
//		err = IPC_publishData(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_NAME, &velodyne_variable_scan);
//		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE1_FMT);
//	}
//	else if (velodyne_number == 2)
//	{
//		err = IPC_publishData(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_NAME, &velodyne_variable_scan);
//		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE2_FMT);
//	}
//	else if (velodyne_number == 3)
//	{
//		err = IPC_publishData(CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_NAME, &velodyne_variable_scan);
//		carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_VARIABLE_SCAN_MESSAGE3_FMT);
//	}
//}

void publish_velodyne_gps(velodyne_driver::velodyne_gps_t gps)
{
	IPC_RETURN_TYPE err;

	assembly_velodyne_gps_message_from_gps(gps);

	err = IPC_publishData(CARMEN_VELODYNE_GPS_MESSAGE_NAME, &velodyne_gps);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_GPS_MESSAGE_FMT);
}

/*********************************************************
		   --- Handlers ---
**********************************************************/

void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("velodyne sensor: disconnected.\n");
		exit(0);
	}
}

int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list_command_line[] =
	{
		{(char *) "commandline",	(char *) "velodyne_number",		CARMEN_PARAM_INT,	&(velodyne_number),		0, NULL},
	};
	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list_command_line, sizeof(param_list_command_line) / sizeof(param_list_command_line[0]));

	char velodyne[10] = "velodyne";
	char integer_string[2];
	sprintf(integer_string, "%d", velodyne_number);
	strcat(velodyne, integer_string);

	carmen_param_t param_list[] = {
			{velodyne, (char*)"gps_enable", CARMEN_PARAM_ONOFF, &velodyne_gps_enabled, 0, NULL},

			{velodyne, (char*)"velodyne_package_rate", CARMEN_PARAM_DOUBLE, &velodyne_package_rate, 0, NULL},
			{velodyne, (char*)"dist_lsb", CARMEN_PARAM_DOUBLE, &dist_lsb, 0, NULL},
			{velodyne, (char*)"rotation_resolution", CARMEN_PARAM_DOUBLE, &rotation_resolution, 0, NULL},
			{velodyne, (char*)"rotation_max_units", CARMEN_PARAM_INT, &rotation_max_units, 0, NULL},
			{velodyne, (char*)"velodyne_num_lasers", CARMEN_PARAM_INT, &velodyne_num_lasers, 0, NULL},
			{velodyne, (char*)"velodyne_num_shots", CARMEN_PARAM_INT, &velodyne_num_shots, 0, NULL},
			{velodyne, (char*)"min_sensing_distance", CARMEN_PARAM_INT, &min_sensing_distance, 0, NULL},
			{velodyne, (char*)"max_sensing_distance", CARMEN_PARAM_INT, &max_sensing_distance, 0, NULL},
			{velodyne, (char*)"velodyne_msg_buffer_size", CARMEN_PARAM_INT, &velodyne_msg_buffer_size, 0, NULL},
			{velodyne, (char*)"velodyne_gps_buffer_size", CARMEN_PARAM_INT, &velodyne_gps_buffer_size, 0, NULL},
			{velodyne, (char*)"velodyne_driver_broadcast_freq_hz", CARMEN_PARAM_DOUBLE, &velodyne_driver_broadcast_freq_hz, 0, NULL},
			{velodyne, (char*)"velodyne_udp_port", CARMEN_PARAM_INT, &velodyne_udp_port, 0, NULL},
			{velodyne, (char*)"velodyne_gps_udp_port", CARMEN_PARAM_INT, &velodyne_gps_udp_port, 0, NULL},
			{velodyne, (char*)"velodyne_upper_header_bytes", CARMEN_PARAM_INT, &velodyne_upper_header_bytes, 0, NULL},
			{velodyne, (char*)"velodyne_lower_header_bytes", CARMEN_PARAM_INT, &velodyne_lower_header_bytes, 0, NULL},
			{velodyne, (char*)"gyro_scale_factor", CARMEN_PARAM_DOUBLE, &gyro_scale_factor, 0, NULL},
			{velodyne, (char*)"temp_scale_factor", CARMEN_PARAM_DOUBLE, &temp_scale_factor, 0, NULL},
			{velodyne, (char*)"temp_base_factor", CARMEN_PARAM_INT, &temp_base_factor, 0, NULL},
			{velodyne, (char*)"accel_scale_factor", CARMEN_PARAM_DOUBLE, &accel_scale_factor, 0, NULL},
			{velodyne, (char*)"velodyne_min_frequency", CARMEN_PARAM_DOUBLE, &velodyne_min_frequency, 0, NULL}
	};
	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	velodyne_max_laser_shots_per_revolution  = (int) ((double) (velodyne_package_rate * velodyne_num_shots) * velodyne_min_frequency + 0.5);

	return 0;
}

void freeMemory()
{
	//free(velodyne_partial_scan.partial_scan);

	for (int i=0; i<(2 * velodyne_max_laser_shots_per_revolution); i++)
	{
		free(velodyne_variable_scan.partial_scan[i].distance);
		free(velodyne_variable_scan.partial_scan[i].intensity);
	}
	free(velodyne_variable_scan.partial_scan);
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	read_parameters(argc, argv);
	carmen_velodyne_define_messages();

	velodyne_variable_scan.host = carmen_get_host();

	while (true)
	{
		if (velodyne == NULL)
		{
			velodyne = new velodyne_driver::VelodyneDriver(velodyne_variable_scan, velodyne_num_lasers, velodyne_max_laser_shots_per_revolution, velodyne_udp_port, velodyne_gps_udp_port);
			config = velodyne->getVelodyneConfig();
		}
		else
		{
			if (velodyne->pollScan(velodyne_variable_scan, velodyne_number, velodyne_udp_port, velodyne_max_laser_shots_per_revolution, velodyne_num_shots,
								   velodyne_package_rate, velodyne_num_lasers))
			{
				carmen_velodyne_publish_variable_scan_message(&velodyne_variable_scan, velodyne_number);

				if (velodyne_gps_enabled && velodyne->pollGps(velodyne_gps_udp_port))
				{
					gps = velodyne->getVelodyneGps();
					publish_velodyne_gps(gps);
				}
			}
			else // just in case velodyne crash
			{
				printf("velodyne disconect\n");
				velodyne->~VelodyneDriver();

				freeMemory();

				velodyne = NULL;
				usleep(1e6 / 2);
			}
		}
	}

	return 0;
}
