/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "build.h"
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <json/json.h>


using namespace ouster;

#define number_of_rays_per_message 16

const size_t N_SCANS = 1;
const size_t UDP_BUF_SIZE = 65536;

char *ouster_ip = NULL;
char *host_ip = NULL;
int ouster_sensor_id = 0;
int ouster_publish_imu = 0;
int ouster_intensity_type = 1;
bool is_alternated = false;


void 
FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("\nOuster LiDAR disconnected!\n");
		exit(0);
	}
}


void
setup_message(carmen_velodyne_variable_scan_message &msg, int number_of_shots, int shot_size)
{
	msg.partial_scan = (carmen_velodyne_shot *) malloc ((number_of_shots + 1) * sizeof(carmen_velodyne_shot));

    for (int i = 0 ; i <= number_of_shots; i++)
	{
		msg.partial_scan[i].shot_size = shot_size;
		msg.partial_scan[i].distance  = (unsigned int*) malloc (shot_size * sizeof(unsigned int));
		msg.partial_scan[i].intensity = (unsigned short*)  malloc (shot_size * sizeof(unsigned short));
	}
	msg.host = carmen_get_host();
}


void 
read_parameters(int argc, char **argv)
{

	carmen_param_t comand_line_param_list[] = {
		{(char*) "commandline", (char*) "sensor_ip", CARMEN_PARAM_STRING, &ouster_ip, 0, NULL},
		{(char*) "commandline", (char*) "host_ip", CARMEN_PARAM_STRING, &host_ip, 0, NULL},
        {(char*) "commandline", (char*) "sensor_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
        {(char*) "commandline", (char*) "intensity_type", CARMEN_PARAM_INT, &ouster_intensity_type, 0, NULL}, 
		{(char*) "commandline", (char*) "publish_imu", CARMEN_PARAM_ONOFF, &ouster_publish_imu, 0, NULL}
	};
	carmen_param_install_params(argc, argv, comand_line_param_list, sizeof(comand_line_param_list)/sizeof(comand_line_param_list[0]));

    if (ouster_intensity_type < 1 || ouster_intensity_type > 3)// 1-Intensity 2-REFLECTIVITY 3-NEAR_IR
    {
        fprintf(stderr, "Invalid intensity type: %d. Filling intensity with zeros.\n", ouster_intensity_type);
        exit(0);
    }

    char lidar_string[256];

    sprintf(lidar_string, "lidar%d", ouster_sensor_id);        // Geather the lidar id

    // carmen_param_t param_list[] =
    // {
	// 		{lidar_string, (char *) "port", CARMEN_PARAM_INT, &ouster_port, 0, NULL},
	// 		{lidar_string, (char *) "imu_port", CARMEN_PARAM_INT, &ouster_imu_port, 0, NULL}
    // };

    // int num_items = sizeof(param_list) / sizeof(param_list[0]);
    // carmen_param_install_params(argc, argv, param_list, num_items);
}


int 
main(int argc, char* argv[]) 
{
    std::cerr << "Ouster client SDK Version " << ouster::CLIENT_VERSION << std::endl;
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

    signal(SIGINT, shutdown_module);
	
    carmen_param_check_version(argv[0]);
    //TODO pegar o ip etc daqui, rodar o modulo soh com o numero da mensagem / -lidar_id 
    read_parameters(argc, argv);

    const std::string sensor_hostname = ouster_ip;
    const std::string data_destination = host_ip;
    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";
    
//TODO checar se vai precisa de porta com varios sensores conectados
    auto handle = sensor::init_client(sensor_hostname, data_destination);
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

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;
 
//TODO Checar se eh o de 64 raios para publicar o modo de 4 mensagens
    std::vector<carmen_velodyne_variable_scan_message> vector_msgs;
	//checa se os raios do Lidar estão alinhados ou alternados
    if (fabs(info.beam_azimuth_angles.at(0) - info.beam_azimuth_angles.at(1)) < 0.5)
    {
    	//RAIOS ALINHADOS
    	carmen_velodyne_variable_scan_message message;
    	setup_message(message, w, h);
		vector_msgs.push_back(message);
    	is_alternated = false;

    }else
    {
    	is_alternated = true;
    	//RAIOS ALTERNADOS
    	if (h < 32)
    	{
    		std::cerr << "Esse código não trata Lidars que têm raios alternados com número de shots menor do que 32\n";
    		carmen_ipc_disconnect();
    		exit(0);
    	}
    	for (size_t i = 0; i < h / number_of_rays_per_message; i++)
		{
			carmen_velodyne_variable_scan_message message;
			setup_message(message, w, number_of_rays_per_message);
			vector_msgs.push_back(message);
		}
    }
    carmen_velodyne_define_messages();
    
    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    // azimuth_window config param reduce the amount of valid columns per scan
    // that we will receive
    int column_window_length =
        (column_window.second - column_window.first + w) % w + 1;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;
    if (is_alternated)
    {
    	std::cerr << "\n Esse LiDAR está publicando menssagens com ids ";
    	for(size_t i = 0; i < h / number_of_rays_per_message; i++)
    		std::cerr << (h / number_of_rays_per_message) * ouster_sensor_id + i << " ";
    	std::cerr << ", CHECAR SE OS PARÂMETROS COM OS IDS DESSAS MENSSAGENS ESTÃO CORRETOS NO CARMEN.INI\n" << std::endl;
    }
    else
    	std::cerr << "\n Esse LiDAR está publicando menssagens com ids " << ouster_sensor_id << std::endl;

    // A LidarScan holds lidar data for an entire rotation of the device
    std::vector<LidarScan> scans{
        N_SCANS, LidarScan{w, h, info.format.udp_profile_lidar}};

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pf = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cerr << "Capturing points... ";

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

    while (true)
    {
        for (size_t i = 0; i < N_SCANS;)
        {
            // wait until sensor data is available
            sensor::client_state st = sensor::poll_client(*handle);

            // check for error status
            if (st & sensor::CLIENT_ERROR)
                FATAL("Sensor client returned error state!");

            // check for lidar data, read a packet and add it to the current batch
            if (st & sensor::LIDAR_DATA)
            {
                if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                    FATAL("Failed to read a packet of the expected size!");

                // batcher will return "true" when the current scan is complete
                if (batch_to_scan(packet_buf.get(), scans[i]))
                {
                    // LidarScan provides access to azimuth block data and headers
                    auto n_invalid = std::count_if(
                        scans[i].headers.begin(), scans[i].headers.end(),
                        [](const LidarScan::BlockHeader &h)
                        {
                            return !(h.status & 0x01);
                        });
                    // retry until we receive a full set of valid measurements
                    // (accounting for azimuth_window settings if any)
                    if (n_invalid <= (int)w - column_window_length)
                        i++;
                }
            }

            // check if IMU data is available (but don't do anything with it)
            if (st & sensor::IMU_DATA)
            {
                sensor::read_imu_packet(*handle, packet_buf.get(), pf);
            }
        }
        // std::cerr << "ok" << std::endl;

        // /*
        //  * The example code includes functions for efficiently and accurately
        //  * computing point clouds from range measurements. LidarScan data can also
        //  * be accessed directly using the Eigen[0] linear algebra library.
        //  *
        //  * [0] http://eigen.tuxfamily.org
        //  */
        for (const LidarScan &scan : scans)
        {
            int number_of_shots = 0;
            // auto n_returns = (scan.field(sensor::RANGE) != 0).count();
            auto range = scan.field(sensor::RANGE);

            auto intensity = scan.field(sensor::INTENSITY);

            // if (ouster_intensity_type == 2)
            // intensity = scan.field(sensor::REFLECTIVITY);
            //     else if (ouster_intensity_type == 3)
            //         intensity = scan.field(sensor::NEAR_IR);

            auto measurement_id = scan.measurement_id();

			for (int m_id = column_window.first; m_id <= column_window.second; m_id++)
			{
				double shot_angle = ((2 * M_PI * measurement_id(m_id)) / w);//Calculo do angulo, no ouster os shots sao fixos
				// TODO!!!!!!!!! Investigar o motivo dessa defasagem de 180 graus na linha abaixo
				//checa se os raios do Lidar estão alinhados ou alternados
				if (fabs(info.beam_azimuth_angles.at(0) - info.beam_azimuth_angles.at(1)) < 0.5)
				{
					//RAIOS ALINAHDOS
					double shot_angle_correction = carmen_normalize_angle_degree(carmen_radians_to_degrees(shot_angle) + (info.beam_azimuth_angles.at(0)) + 180);
					// std::cerr << "\n angle " << shot_angle << " shot_angle_correction " << shot_angle_correction << std::endl;
					vector_msgs[0].partial_scan[m_id].angle = shot_angle_correction;
					vector_msgs[0].partial_scan[m_id].shot_size = h;
					//std::cout << h << std::endl;
					// std::cerr << "angle_correction " << angle_correction << std::endl;

					for (size_t ipx = 0; ipx < h; ipx++)
					{
						vector_msgs[0].partial_scan[m_id].distance[ipx] = (unsigned short) range(ipx, m_id);
						vector_msgs[0].partial_scan[m_id].intensity[ipx] = (unsigned char) intensity(ipx, m_id);
					}
				}else
				{
					//RAIOS ALTERNADOS
					for (size_t i = 0; i < h / number_of_rays_per_message; i++)
					{
						double shot_angle_correction = carmen_normalize_angle_degree(carmen_radians_to_degrees(shot_angle) +  (info.beam_azimuth_angles.at(i % (h / number_of_rays_per_message))) + 180);
						//std::cout<< info.beam_azimuth_angles.at(i % (h / number_of_rays_per_message)) << "\n";
						// std::cerr << "\n angle " << shot_angle << " shot_angle_correction " << shot_angle_correction << std::endl;
						vector_msgs[i % (h / number_of_rays_per_message)].partial_scan[m_id].angle = shot_angle_correction;

						vector_msgs[i % (h / number_of_rays_per_message)].partial_scan[m_id].shot_size = number_of_rays_per_message;
					}
					// std::cerr << "angle_correction " << angle_correction << std::endl;

					for (size_t ipx = 0; ipx < h ; ipx++)
					{
						vector_msgs[ipx % (h / number_of_rays_per_message)].partial_scan[m_id].distance[(int) (ipx / (h / number_of_rays_per_message))] = (unsigned int)range(ipx, m_id);
						vector_msgs[ipx % (h / number_of_rays_per_message)].partial_scan[m_id].intensity[(int) (ipx / (h / number_of_rays_per_message))] = (unsigned short)intensity(ipx, m_id);
						/*std::cout<< "indice msg " << ipx % (h / number_of_rays_per_message) << "\n";
						std::cout<< "indice raio " << m_id << "\n";
						std::cout<< "indice shot " << ipx << "\n";
						std::cout<< "indice distancia " << int(ipx / (h / number_of_rays_per_message)) << "\n";
						std::cout<< "terminou o laco\n";*/
					}
				}
				number_of_shots++;
			}
			if (is_alternated)
			{
				for (size_t i = 0; i < h / number_of_rays_per_message; i++)
				{
					vector_msgs[i].host = carmen_get_host();
					vector_msgs[i].timestamp = carmen_get_time();
					vector_msgs[i].number_of_shots = number_of_shots;
					carmen_velodyne_publish_variable_scan_message(&vector_msgs[i], (h / number_of_rays_per_message) * ouster_sensor_id + i);
					if (fabs(info.beam_azimuth_angles.at(0) - info.beam_azimuth_angles.at(1)) < 0.5)
						//break para preencher apenas uma menssagem para Lidars com raios alinhados
						break;
				}
			}
			else
			{
				vector_msgs[0].host = carmen_get_host();
				vector_msgs[0].timestamp = carmen_get_time();
				vector_msgs[0].number_of_shots = number_of_shots;
				carmen_velodyne_publish_variable_scan_message(&vector_msgs[0], ouster_sensor_id);
			}

        }
        // std::cerr << "Publiquei   " << std::endl;
    }

    return 0;
}
