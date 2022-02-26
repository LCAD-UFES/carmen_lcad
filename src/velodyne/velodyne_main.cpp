#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>

#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <driver.h>

using namespace std;

velodyne_driver::velodyne_gps_t gps;
velodyne_driver::velodyne_config_t config;
velodyne_driver::VelodyneDriver* velodyne = NULL;

static carmen_velodyne_partial_scan_message velodyne_partial_scan;
static carmen_velodyne_gps_message velodyne_gps;
static int velodyne_scan_port;
static int velodyne_gps_port;
static int velodyne_gps_enabled;


void assembly_velodyne_gps_message_from_gps(velodyne_driver::velodyne_gps_t gps)
{
	velodyne_gps.gyro1 = (gps.packet.gyro1 & 0x0fff) * velodyne_driver::GYRO_SCALE_FACTOR;
	velodyne_gps.gyro2 = (gps.packet.gyro2 & 0x0fff) * velodyne_driver::GYRO_SCALE_FACTOR;
	velodyne_gps.gyro3 = (gps.packet.gyro3 & 0x0fff) * velodyne_driver::GYRO_SCALE_FACTOR;

	velodyne_gps.temp1 = (gps.packet.temp1 & 0x0fff) * velodyne_driver::TEMP_SCALE_FACTOR + velodyne_driver::TEMP_BASE_FACTOR;
	velodyne_gps.temp2 = (gps.packet.temp2 & 0x0fff) * velodyne_driver::TEMP_SCALE_FACTOR + velodyne_driver::TEMP_BASE_FACTOR;
	velodyne_gps.temp3 = (gps.packet.temp3 & 0x0fff) * velodyne_driver::TEMP_SCALE_FACTOR + velodyne_driver::TEMP_BASE_FACTOR;

	velodyne_gps.accel1_x = (gps.packet.accel1_x & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;
	velodyne_gps.accel2_x = (gps.packet.accel2_x & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;
	velodyne_gps.accel3_x = (gps.packet.accel3_x & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;

	velodyne_gps.accel1_y = (gps.packet.accel1_y & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;
	velodyne_gps.accel2_y = (gps.packet.accel2_y & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;
	velodyne_gps.accel3_y = (gps.packet.accel3_y & 0x0fff) * velodyne_driver::ACCEL_SCALE_FACTOR;

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

void publish_velodyne_partial_scan()
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, &velodyne_partial_scan);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
}

void publish_velodyne_gps(velodyne_driver::velodyne_gps_t gps)
{
	IPC_RETURN_TYPE err;

	assembly_velodyne_gps_message_from_gps(gps);

	err = IPC_publishData(CARMEN_VELODYNE_GPS_MESSAGE_NAME, &velodyne_gps);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_GPS_MESSAGE_FMT);
}


// void
// publish_velodyne_variable_scan()
// {
//     static bool first_time = true;
// 	static carmen_velodyne_variable_scan_message msg;

// 	if (first_time)
// 	{
// 		msg.partial_scan = (carmen_velodyne_shot *) malloc ((4000 + 1) * sizeof(carmen_velodyne_shot));
	
//         for (int i = 0 ; i <= 4000; i++)
//         {
//             msg.partial_scan[i].shot_size = 32;
//             msg.partial_scan[i].distance  = (unsigned short*) malloc (32 * sizeof(unsigned short));
//             msg.partial_scan[i].intensity = (unsigned char*)  malloc (32 * sizeof(unsigned char));
//         }
//         msg.host = carmen_get_host();
// 		first_time = false;
// 	}
// 	msg.number_of_shots = velodyne_partial_scan.number_of_32_laser_shots;
	
// 	for(int i = 0; i < velodyne_partial_scan.number_of_32_laser_shots; i++)
// 	{
// 		msg.partial_scan[i].distance = velodyne_partial_scan.partial_scan[i].distance;
// 		msg.partial_scan[i].intensity = velodyne_partial_scan.partial_scan[i].intensity;
// 		msg.partial_scan[i].angle = velodyne_partial_scan.partial_scan[i].angle;
// 	}
// 	msg.timestamp = velodyne_partial_scan.timestamp;

// 	carmen_velodyne_publish_variable_scan_message(&msg, 0);
// }


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

	carmen_param_t param_list[] = {
			{(char*)"velodyne", (char*)"scan_port", CARMEN_PARAM_INT, &velodyne_scan_port, 0, NULL},
			{(char*)"velodyne", (char*)"gps_enable", CARMEN_PARAM_ONOFF, &velodyne_gps_enabled, 0, NULL},
			{(char*)"velodyne", (char*)"gps_port", CARMEN_PARAM_INT, &velodyne_gps_port, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	read_parameters(argc, argv);
	carmen_velodyne_define_messages();

	velodyne_partial_scan.host = carmen_get_host();

	while (true)
	{
		if (velodyne == NULL)
		{
			velodyne = new velodyne_driver::VelodyneDriver(velodyne_scan_port, velodyne_gps_port, velodyne_partial_scan);
			config = velodyne->getVelodyneConfig();
		}
		else
		{
			if (velodyne->pollScan(velodyne_partial_scan))
			{
				publish_velodyne_partial_scan();

				// publish_velodyne_variable_scan();

				if (velodyne_gps_enabled && velodyne->pollGps())
				{
					gps = velodyne->getVelodyneGps();
					publish_velodyne_gps(gps);
				}
			}
			else
			{
				printf("velodyne disconected?\n");
//				velodyne->~VelodyneDriver();
//				free(velodyne_partial_scan.partial_scan);
//				velodyne = NULL;
				usleep(1e6 / 2);
			}
		}
	}

	return 0;
}
