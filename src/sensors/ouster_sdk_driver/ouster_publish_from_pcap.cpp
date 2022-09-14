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
#include "helpers.h"
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include "ouster/os_pcap.h"
#include <json/json.h>


using namespace ouster;

const size_t N_SCANS = 50;
const size_t UDP_BUF_SIZE = 65536;

char *pcap_file2 = NULL;
char *json_file2 = NULL;
int ouster_sensor_id = 0;
int ouster_publish_imu = 0;
int ouster_intensity_type = 1;

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
		{(char*) "commandline", (char*) "pcap_file", CARMEN_PARAM_STRING, &pcap_file2, 0, NULL},
		{(char*) "commandline", (char*) "json_file", CARMEN_PARAM_STRING, &json_file2, 0, NULL},
        {(char*) "commandline", (char*) "sensor_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
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
    
    //Carmen Stuffs
    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);
	
    carmen_param_check_version(argv[0]);
    read_parameters(argc, argv);

    const std::string pcap_file = pcap_file2;
    const std::string json_file = json_file2;

    auto handle = sensor_utils::replay_initialize(pcap_file);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto info = sensor::metadata_from_json(json_file);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;
 
    carmen_velodyne_variable_scan_message message;
//TODO Checar se eh o de 64 raios para publicar o modo de 4 mensagens
    setup_message(message, w, h);
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
              << column_window.second << "]" 
              << "\n  Publising Lidar Message:   " << ouster_sensor_id << std::endl;

    // A LidarScan holds lidar data for an entire rotation of the device
    auto scan = LidarScan(w, h, info.format.udp_profile_lidar);

    std::cerr << "Capturing points... ";

//    while (true)
    {
        for (size_t i = 0; i < N_SCANS; i++)
        {
        	sleep(5);
            get_complete_scan(handle, scan, info);

        // /*
        //  * The example code includes functions for efficiently and accurately
        //  * computing point clouds from range measurements. LidarScan data can also
        //  * be accessed directly using the Eigen[0] linear algebra library.
        //  *
        //  * [0] http://eigen.tuxfamily.org
        //  */
            int number_of_shots = 0;
            // auto n_returns = (scan.field(sensor::RANGE) != 0).count();
            auto range = scan.field(sensor::RANGE);

            auto intensity = scan.field(sensor::INTENSITY);

            // if (ouster_intensity_type == 2)
            // intensity = scan.field(sensor::REFLECTIVITY);
            //     else if (ouster_intensity_type == 3)
            //         intensity = scan.field(sensor::NEAR_IR);

            auto measurement_id = scan.measurement_id();
            message.host = carmen_get_host();
            message.timestamp = carmen_get_time();

            for (int m_id = column_window.first; m_id <= column_window.second; m_id++)
            {
                double shot_angle = ((2 * M_PI * measurement_id(m_id)) / w);
                // TODO!!!!!!!!! Investigar o motivo dessa defasagem de 180 graus na linha abaixo
                double shot_angle_correction = carmen_normalize_angle_degree(carmen_radians_to_degrees(shot_angle) + (info.beam_azimuth_angles.at(0)) + 180);
                // std::cerr << "\n angle " << shot_angle << " shot_angle_correction " << shot_angle_correction << std::endl;
                message.partial_scan[m_id].angle = shot_angle_correction;
                message.partial_scan[m_id].shot_size = h;
                // std::cerr << "angle_correction " << angle_correction << std::endl;

                for (size_t ipx = 0; ipx < h; ipx++)
                {
                    message.partial_scan[m_id].distance[ipx] = (unsigned short)range(ipx, m_id);
                    message.partial_scan[m_id].intensity[ipx] = (unsigned char)intensity(ipx, m_id);
                }
                number_of_shots++;
            }

            message.number_of_shots = number_of_shots;
            carmen_velodyne_publish_variable_scan_message(&message, ouster_sensor_id);
            ouster::sensor_utils::replay_uninitialize(*handle);
            std::cerr << "Publiquei   " << std::endl;
        }

    }

    return 0;
}
