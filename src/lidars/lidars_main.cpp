#include <carmen/carmen.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <carmen/velodyne_interface.h>
#include <driver2.h>

#include "lidar_drivers.h"

using namespace std;

velodyne_driver::velodyne_gps_t gps;
velodyne_driver::velodyne_config_t config;
velodyne_driver::VelodyneDriver* velodyne = NULL;

carmen_velodyne_variable_scan_message variable_scan_msg;


int
get_lidar_id(int argc, char **argv)
{
	if (argc != 2)
		carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \n\nUsage:\n %s <lidar_number>\n", argv[0], argv[0], argc-1, argv[0]);

	return (atoi(argv[1]));
}


void
setup_message(carmen_lidar_config lidar_config)
{
	variable_scan_msg.partial_scan = (carmen_velodyne_shot *) malloc ((INITIAL_MAX_NUM_SHOT + 1) * sizeof(carmen_velodyne_shot));
	
    for (int i = 0 ; i <= INITIAL_MAX_NUM_SHOT; i++)
	{
		variable_scan_msg.partial_scan[i].shot_size = lidar_config.shot_size;
		variable_scan_msg.partial_scan[i].distance = (unsigned short*) malloc (lidar_config.shot_size * sizeof(unsigned short));
		variable_scan_msg.partial_scan[i].intensity = (unsigned char*) malloc (lidar_config.shot_size * sizeof(unsigned char));
	}
	variable_scan_msg.host = carmen_get_host();
}


void
run_velodyne_VLP16_PUCK_driver(carmen_velodyne_variable_scan_message &msg, carmen_lidar_config lidar_config, int lidar_id)
{
	int port = atoi(lidar_config.port);

	velodyne = new velodyne_driver::VelodyneDriver(msg, lidar_config.shot_size, INITIAL_MAX_NUM_SHOT, port, 8309);
	config = velodyne->getVelodyneConfig();
	
	while (1)
	{
		if (velodyne->pollScan(msg, port, INITIAL_MAX_NUM_SHOT, 12, 754.0, lidar_config.shot_size))  // 12 is the number of shots per packet, 754.0 the velodyne package rate, see manual
		{
			carmen_velodyne_publish_variable_scan_message(&msg, lidar_id);
		}
		else
		{
			printf("Velodyne disconected!\n");
			velodyne->~VelodyneDriver();
		}
	}
}


/*********************************************************
		   --- Publishers ---
**********************************************************/



/*********************************************************
		   --- Handlers ---
**********************************************************/


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("\nLidar sensor: disconnected.\n");
		exit(0);
	}
}


/*********************************************************
		   --- Initializations ---
**********************************************************/


void
read_parameters(int argc, char** argv, int lidar_id, carmen_lidar_config &lidar_config)
{
    char *vertical_correction_string;
    char lidar_string[256];
	
	sprintf(lidar_string, "lidar%d", lidar_id);        // Geather the lidar id

    carmen_param_t param_list[] = {
			{lidar_string, (char*)"model", CARMEN_PARAM_STRING, &lidar_config.model, 0, NULL},
			{lidar_string, (char*)"ip", CARMEN_PARAM_STRING, &lidar_config.ip, 0, NULL},
			{lidar_string, (char*)"port", CARMEN_PARAM_STRING, &lidar_config.port, 0, NULL},
			{lidar_string, (char*)"shot_size", CARMEN_PARAM_INT, &lidar_config.shot_size, 0, NULL},
            {lidar_string, (char*)"min_sensing", CARMEN_PARAM_INT, &lidar_config.min_sensing, 0, NULL},
            {lidar_string, (char*)"max_sensing", CARMEN_PARAM_INT, &lidar_config.max_sensing, 0, NULL},
			{lidar_string, (char*)"range_division_factor", CARMEN_PARAM_INT, &lidar_config.range_division_factor, 0, NULL},
            {lidar_string, (char*)"time_between_shots", CARMEN_PARAM_DOUBLE, &lidar_config.time_between_shots, 0, NULL},
			{lidar_string, (char*)"x", CARMEN_PARAM_DOUBLE, &(lidar_config.pose.position.x), 0, NULL},
			{lidar_string, (char*)"y", CARMEN_PARAM_DOUBLE, &(lidar_config.pose.position.y), 0, NULL},
			{lidar_string, (char*)"z", CARMEN_PARAM_DOUBLE, &lidar_config.pose.position.z, 0, NULL},
			{lidar_string, (char*)"roll", CARMEN_PARAM_DOUBLE, &lidar_config.pose.orientation.roll, 0, NULL},
			{lidar_string, (char*)"pitch", CARMEN_PARAM_DOUBLE, &lidar_config.pose.orientation.pitch, 0, NULL},
			{lidar_string, (char*)"yaw", CARMEN_PARAM_DOUBLE, &lidar_config.pose.orientation.yaw, 0, NULL},
			{lidar_string, (char*)"vertical_angles", CARMEN_PARAM_STRING, &vertical_correction_string, 0, NULL},
	};
	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

    lidar_config.vertical_angles = (double*) malloc(lidar_config.shot_size * sizeof(double));

    for (int i = 0; i < lidar_config.shot_size; i++)
		lidar_config.vertical_angles[i] = CLF_READ_DOUBLE(&vertical_correction_string); // CLF_READ_DOUBLE takes a double number from a string
    
	//printf("Model: %s Port: %s Shot size: %d Min Sensing: %d Max Sensing: %d Range division: %d Time: %lf\n", lidar_config.model, lidar_config.port, lidar_config.shot_size, lidar_config.min_sensing, lidar_config.max_sensing, lidar_config.range_division_factor, lidar_config.time_between_shots);  printf("X: %lf Y: %lf Z: %lf R: %lf P: %lf Y: %lf\n", lidar_config.pose.position.x, lidar_config.pose.position.y, lidar_config.pose.position.z, lidar_config.pose.orientation.roll, lidar_config.pose.orientation.pitch, lidar_config.pose.orientation.yaw); for (int i = 0; i < lidar_config.shot_size; i++) printf("%lf ", lidar_config.vertical_angles[i]); printf("\n");
}


int 
main(int argc, char **argv)
{
	int lidar_id;
	carmen_lidar_config lidar_config;

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	lidar_id = get_lidar_id(argc, argv);

	read_parameters(argc, argv, lidar_id, lidar_config);

	carmen_velodyne_define_messages();

	printf("-------------------------\n  Lidar %d %s loaded!\n-------------------------\n", lidar_id, lidar_config.model);

	setup_message(lidar_config);

	if(strcmp(lidar_config.model, "RS16") == 0)
	{
		run_robosense_RSLiDAR16_driver(variable_scan_msg, lidar_config, lidar_id);
	}
	if(strcmp(lidar_config.model, "VLP16") == 0)
	{
		run_velodyne_VLP16_PUCK_driver(variable_scan_msg, lidar_config, lidar_id);
	}

	carmen_die("\nERROR: lidar%d_model %s not found!\nPlease verify the carmen.ini file.\n\n", lidar_id, lidar_config.model);

	return 0;
}
