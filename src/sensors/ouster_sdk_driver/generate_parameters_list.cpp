#include <carmen/carmen.h>

#include <iostream>
#include <string.h>
#include<vector>

#include "build.h"
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <json/json.h>



std::string lidar_sensorBox_model;
std::string _ray_order;
char *ouster_ip = NULL;
char *host_ip = NULL;
char *num_split = NULL;
int ouster_port;
int ouster_imu_port;
int _shot_size;
int lidar_id;
int n_split;
int model;


using namespace ouster;

void
read_parameters(int argc, char **argv)
{

	carmen_param_t comand_line_param_list[] = {
			{(char*) "commandline", (char*) "sensor_ip", CARMEN_PARAM_STRING, &ouster_ip, 0, NULL},
			{(char*) "commandline", (char*) "host_ip", CARMEN_PARAM_STRING, &host_ip, 0, NULL},
			{(char*) "commandline", (char*) "port", CARMEN_PARAM_INT, &ouster_port, 0, NULL},
			{(char*) "commandline", (char*) "imu_port", CARMEN_PARAM_INT, &ouster_imu_port, 0, NULL},
			{(char*) "commandline", (char*) "lidar_id", CARMEN_PARAM_INT, &lidar_id, 0, NULL},
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
	//TODO pegar o ip etc daqui, rodar o modulo soh com o numero da mensagem / -sensor_ip -host_ip -port -imu_purt -lidar_id
	read_parameters(argc, argv);

	const std::string sensor_hostname = ouster_ip;
	const std::string data_destination = host_ip;



	std::string vehicle_name;
	std::cout<< "Digite o nome do veículo: \n";
	std::cout<< "Veículos disponíveis: Bravo, Arocs, Atego_1730, Atego_2430, Axor, Actros_4844 e Ford_escape. \n";
	std::cin >>  vehicle_name;



	if("Bravo" == vehicle_name){
		model = 0;

	}else if("Arocs" == vehicle_name){
		model = 1;

	}else if("Atego_1730" == vehicle_name){
		model = 2;

	}else if("Atego_2430" == vehicle_name){
		model = 3;

	}else if("Axor" == vehicle_name){
		model = 4;

	}else if("Actros_4844" == vehicle_name){
		model = 5;

	}else if("Ford_escape" == vehicle_name){
		model = 6;

	}

	carmen_pose_3D models[6];

	// LIDAR 0
	if(lidar_id == 0){
		// Bravo;
		models[0].position = {0.13 ,0.0 ,0.32};
		models[0].orientation = {0.0 ,0.0 ,0.02};

		//Arocs;
		models[1].position = {2.94, 0.0 ,-1.138};
		models[1].orientation = {0.0, 0.003, 0.0};

		//Atego_1730;
		models[2].position = {2.94, 0.0 ,-1.138};
		models[2].orientation = {0.0, 0.003, 0.0};

		//Atego_2430;
		models[3].position = {2.94, 0.0 ,-1.138};
		models[3].orientation = {0.0, 0.003, 0.0};

		// Axor;
		models[4].position = {2.94, 0.0 ,-1.138};
		models[4].orientation = {0.0, 0.003, 0.0};

		// Actros_4844;
		models[5].position = {2.94, 0.0 ,-1.138};
		models[5].orientation = {0.0, 0.003, 0.0};

		// Ford_escape;
		models[6].position = {2.96 ,0.3, -0.48};
		models[6].orientation = {-0.02, 0.52, 0.0};

	}else if(lidar_id == 5){

		//carmen_pose_3D Bravo;
		models[0].position = {1.65 ,1.118 ,-1.90};
		models[0].orientation = {0.0 ,0.0 ,-0.015};

		//carmen_pose_3D Arocs;
		models[1].position = {0.71, -1.286 ,-1.725};
		models[1].orientation = {0.04,-0.001, 2.15};

		//carmen_pose_3D Atego_1730;
		models[2].position = {0.1085, 0.0 ,0.318};
		models[2].orientation = {0.0, 0.0, 0.0};

		//carmen_pose_3D Atego_2430;
		models[3].position = {0.47, -1.140 ,-1.822};
		models[3].orientation = {0.0, 0.0, -0.05};

		//carmen_pose_3D Axor;
		models[4].position = {0.1085, 0.0 ,0.318};
		models[4].orientation = {0.0, 0.0, 0.0};

		//carmen_pose_3D Actros_4844;
		models[5].position = {0.46, -1.325 ,-1.6};
		models[5].orientation = {-0.1, -0.1, -2.1};

		//carmen_pose_3D Ford_escape;
		models[6].position = {0.215 ,-0.515, 0.078};
		models[6].orientation = {0.0, 0.0, 0.0};


	}else if(lidar_id == 7 ){

		//carmen_pose_3D Bravo;
		models[0].position = {1.75 ,-1.118 ,-1.85};
		models[0].orientation = {0.0 ,0.0 ,-0.04};

		//carmen_pose_3D Arocs;
		models[1].position = {0.71, 1.286 ,-1.725};
		models[1].orientation = {0.04,-0.001, 2.15};

		//carmen_pose_3D Atego_1730;
		models[2].position = {0.6, -1.27 ,-1.4};
		models[2].orientation = {0.05, -0.08, -1.5};

		//carmen_pose_3D Atego_2430;
		models[3].position = {2.94, 0.0 ,-1.138};
		models[3].orientation = {0.0, 0.003, 0.0};

		//_pose_3D Axor;
		models[4].position = {0.6, -1.27 ,-1.4};
		models[4].orientation = {0.05, -0.08, -1.5};

		//carmen_pose_3D Actros_4844;
		models[5].position = {0.46, 1.325 ,-1.6};
		models[5].orientation = {0.04, -0.001, 2.15};

		//carmen_pose_3D Ford_escape;
		models[6].position = {0.215 ,-0.515, 0.078};
		models[6].orientation = {0.0, 0.0, 0.0};

	}


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


	// beam azimuth angles
	std::vector<double> list_azimuth_angles;
	// Raw metadata can be parsed into a `sensor_info` struct
	sensor::sensor_info info = sensor::parse_metadata(metadata);

	//get the ouster model
	lidar_sensorBox_model = info.prod_line;

	//get the shot
	size_t h = info.format.pixels_per_column;

	// check
	if (fabs(info.beam_azimuth_angles.at(0)) - fabs(info.beam_azimuth_angles.at(1)) < 0.2) //não alernado
	{
		_shot_size = h;
		n_split = 1;
		for (int i = 0 ; i < _shot_size; i++)
		{
			_ray_order += std::to_string(i) + " ";
		}

		// 4 splits
	}else if (fabs(info.beam_azimuth_angles.at(0)) - fabs(info.beam_azimuth_angles.at(2)) > 1)
	{

		_shot_size = h/4;
		n_split = 4;
		for (int i = 0 ; i < _shot_size; i++){

			_ray_order += std::to_string(i) + " ";

		}
		// 2 splits
	}else{

		_shot_size= h/2;
		n_split = 2;
		for (int i = 0 ; i < _shot_size; i++){

			_ray_order += std::to_string(i) + " ";

		}

	}

	// creating string arrays

	//
	std::string string_list_for_vertical_angles[n_split];
	std::string string_list_for_azimuth_angles;
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
			string_list_for_vertical_angles[i] += std::to_string(list_vertical_angles[i].at(j)) + " ";

		}
		std::cout<< "\n";
	}


	std::cout<< "beam_azimuth_angles: ";
	for(unsigned int i = 0 ; i < info.beam_azimuth_angles.size(); i++)
	{
		list_azimuth_angles.push_back(info.beam_azimuth_angles.at(i));
		std::cout<< list_azimuth_angles[i] << " ";
		string_list_for_azimuth_angles += std::to_string(list_azimuth_angles[i]) + " ";



	}
	std::cout<< "\n";

	// print lidar parameters

	std::cout <<"####################################################################################################### \n";
	std::cout <<"############################################### Lidar ################################################# \n";
	std::cout <<"####################################################################################################### \n";

	std::cout << std::fixed;
	for(int i = lidar_id ; i < lidar_id + n_split; i++)
	{


		// print lidar parameters

		std::cout <<"lidar"<< i<<"_model                 "<<  lidar_sensorBox_model <<"\n"; // # HDL32 # VLP16 # RS16
		std::cout <<"lidar"<< i<<"_port                  "<<  ouster_port <<"\n";
		std::cout <<"lidar"<< i<<"_imu_port              "<<  ouster_imu_port<<"\n";
		std::cout <<"lidar"<< i<<"_ip                    "<<  sensor_hostname<<"\n";
		std::cout <<"lidar"<< i<<"_shot_size             "<<  _shot_size <<"\n";
		std::cout <<"lidar"<< i<<"_min_sensing           "<<  1000<<"\n"; 			//# 2m in 2mm units
		std::cout <<"lidar"<< i<<"_max_sensing           "<<  24000<<"\n"; 			//# 1200m in 2mm units
		std::cout <<"lidar"<< i<<"_range_division_factor "<<  1000 <<"\n";
		std::cout <<"lidar"<< i<<"_max_range             "<<  75.0 <<"\n";
		std::cout <<"lidar"<< i<<"_time_between_shots    "<<  0.000048828 <<"\n";
		std::cout <<"lidar"<< i<<"_x                     "<<  models[model].position.x <<"\n";   // # 2.94 # 0.1085          # Lidar position and orientation in relation to sensor board 1
		std::cout <<"lidar"<< i<<"_y                     "<<  models[model].position.y <<"\n";
		std::cout <<"lidar"<< i<<"_z                     "<<  models[model].position.z <<"\n";
		std::cout <<"lidar"<< i<<"_roll                  "<<  models[model].orientation.roll <<"\n";
		std::cout <<"lidar"<< i<<"_pitch                  "<<  models[model].orientation.pitch <<"\n";
		std::cout <<"lidar"<< i<<"_yaw                    "<<  models[model].orientation.yaw <<"\n";
		std::cout <<"lidar"<< i<<"_ray_order              "<< _ray_order  <<"\n";
		std::cout <<"lidar"<< i<<"_vertical_angles        "<< string_list_for_vertical_angles[i-lidar_id] << "\n";
		std::cout <<"lidar"<< i<<"_sensor_reference       0\n";   //# informa em que a posicao do lidar esta referenciada. 0 para sensorboard, 1 para front_bullbar, 2 para rear_bullbar
		std::cout<<"\n";
	}

	return (0);
}
