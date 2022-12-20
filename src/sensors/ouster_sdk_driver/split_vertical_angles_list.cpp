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
int ouster_port = 7502;
int ouster_imu_port = 7503;

using namespace ouster;

void
read_parameters(int argc, char **argv)
{

	carmen_param_t comand_line_param_list[] = {
		{(char*) "commandline", (char*) "sensor_ip", CARMEN_PARAM_STRING, &ouster_ip, 0, NULL},
		{(char*) "commandline", (char*) "host_ip", CARMEN_PARAM_STRING, &host_ip, 0, NULL},
		{(char*) "commandline", (char*) "num_split", CARMEN_PARAM_STRING, &num_split, 0, NULL},
		{(char*) "commandline", (char*) "port", CARMEN_PARAM_INT, &ouster_port, 0, NULL},
		{(char*) "commandline", (char*) "imu_port", CARMEN_PARAM_INT, &ouster_imu_port, 0, NULL},
	};
	carmen_param_install_params(argc, argv, comand_line_param_list, sizeof(comand_line_param_list)/sizeof(comand_line_param_list[0]));

}

void
FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

int
main(int argc, char* argv[])
{
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
	auto handle = sensor::init_client(sensor_hostname, ouster_port, ouster_imu_port);
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
	if((list_vertical_angles[0].at(0) - list_vertical_angles[1].at(0)) < 0)
	{
		for(int i = 0 ; i < n_split; i++)
		{
			std::cout << "Lista " << i << ": ";
			for (unsigned int j = 0; j < list_vertical_angles[i].size(); j++)
			{
				std::cout<< list_vertical_angles[i].at(j) << " ";
			}
			std::cout << "\n";
		}
	}else
	{
		int c = 0;
		for(int i = (n_split -1) ; i >= 0; i--)
		{
			std::cout << "Lista " << c << ": ";
			for (unsigned int j = (list_vertical_angles[i].size() -1); j >= 0; j--)
			{
				std::cout<< list_vertical_angles[i].at(j) << " ";
			}
			std::cout << "\n";
			c++;
		}
	}
	return (0);
}
