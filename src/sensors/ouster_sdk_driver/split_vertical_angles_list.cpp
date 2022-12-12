#include <carmen/carmen.h>

#include <iostream>

#include<vector>

#include "build.h"
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <json/json.h>

char *ouster_ip = NULL;
char *host_ip = NULL;
char *num_split = NULL;
int ouster_sensor_id = 0;


using namespace ouster;

void
read_parameters(int argc, char **argv)
{

	carmen_param_t comand_line_param_list[] = {
        {(char*) "commandline", (char*) "lidar_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
		{(char*) "commandline", (char*) "num_split", CARMEN_PARAM_STRING, &num_split, 0, NULL},
//        {(char*) "commandline", (char*) "intensity_type", CARMEN_PARAM_INT, &ouster_intensity_type, 0, NULL},
	};
	carmen_param_install_params(argc, argv, comand_line_param_list, sizeof(comand_line_param_list)/sizeof(comand_line_param_list[0]));


    if (host_ip == NULL)
    {
    	host_ip = (char*) malloc (12 * sizeof(char));
    	sprintf(host_ip, "192.168.1.1");
    }

    char lidar_string[256];

    sprintf(lidar_string, "lidar%d", ouster_sensor_id);        // Geather the lidar id

    carmen_param_allow_unfound_variables(0);
     carmen_param_t param_list[] =
     {
	 		{lidar_string, (char *) "ip", CARMEN_PARAM_STRING, &ouster_ip, 0, NULL}
     };

     int num_items = sizeof(param_list) / sizeof(param_list[0]);
     carmen_param_install_params(argc, argv, param_list, num_items);
}


void
prog_usage(char *prog_name, const char *error_msg = NULL, const char *error_msg2 = NULL)
{
	if (error_msg)
		fprintf(stderr, "\n%s", error_msg);
	if (error_msg2)
		fprintf(stderr, "%s", error_msg2);

	fprintf(stderr, "\n\nUsage:   %s   -lidar_id {lidar from Carmen.ini number} -num_split {number of splits of vertical angles list} \n", prog_name);
	fprintf(stderr,   "default value: -host_ip  192.168.1.1    -num_rays_per_message 16       -intensity_type 3\n");

	exit(-1);
}

void
FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

int
main(int argc, char* argv[])
{
 if (argc > 1 && strcmp(argv[1], "-h") == 0)
		prog_usage(argv[0]);
	std::cerr << "Ouster client SDK Version " << ouster::SDK_VERSION_FULL << std::endl;
	/*
	 * The sensor client consists of the network client and a library for
	 * reading and working with data.
	 *
	 * The network client supports reading and writing a limited number of
	 * configuration parameters and receiving data without working directly with
	 * the socket APIs. See the `client.h` for more details. The minimum
	 * required parameters are the sensor hostname/ip and the data destination
	 * hostname/ip.
	 */

	//Carmen Stuffs
	carmen_ipc_initialize(argc, argv);


	carmen_param_check_version(argv[0]);
	//TODO pegar o ip etc daqui, rodar o modulo soh com o numero da mensagem / -lidar_id
	read_parameters(argc, argv);


	const std::string sensor_hostname = ouster_ip;
	const std::string data_destination = host_ip;
	int n_split = atoi(num_split);
	std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

	//TODO checar se vai precisa de porta com varios sensores conectados
	auto handle = sensor::init_client(sensor_hostname, 7502, 7503);
	if (!handle) FATAL("Failed to connect");
	std::cerr << "ok" << std::endl;

	/*
	 * Configuration and calibration parameters can be queried directly from the
	 * sensor. These are required for parsing the packet stream and calculating
	 * accurate point clouds.
	 */
	std::cerr << "Gathering metadata..." << std::endl;
	auto metadata = sensor::get_metadata(*handle);

	// Raw metadata can be parsed into a `sensor_info` struct
	sensor::sensor_info info = sensor::parse_metadata(metadata);

	std::vector<std::vector<double>> list_vertical_angles;
	for(int i = 0 ; i < n_split; i++)
	{

		std::vector<double> aux_vector;
		for (unsigned int j = i; j < info.beam_altitude_angles.size(); j = j + n_split)
			aux_vector.push_back(info.beam_altitude_angles.at(j));
		list_vertical_angles.push_back(aux_vector);
	}
	for(int i = 0 ; i < n_split; i++)
	{
		std::cout << "Lista " << i << ": ";
		for (unsigned int j = 0; j < list_vertical_angles[i].size(); j++)
		{
			std::cout<< list_vertical_angles[i].at(j) << " ";
		}
		std::cout << "\n";
	}
	return (0);
}
