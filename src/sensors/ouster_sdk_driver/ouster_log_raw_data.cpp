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
#include <ouster/os_pcap.h>
#include <ouster/types.h>
#include <json/json.h>


using namespace ouster;

const size_t N_SCANS = 20;
const size_t UDP_BUF_SIZE = 65536;

char *ouster_ip = NULL;
char *host_ip = NULL;
int ouster_sensor_id = 0;
int ouster_publish_imu = 0;
int ouster_intensity_type = 1;
int ouster_port = 0;
int ouster_imu_port = 0;

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
save_single_line_in_file(std::string file_name, std::string line_to_save)
{
	std::ofstream file;
	file.open(file_name.c_str(), std::ofstream::app);
	file << line_to_save;
	file.close();
	std::cout << "New line Saved!" << std::endl;
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

    carmen_param_t param_list[] =
    {
			{lidar_string, (char *) "port", CARMEN_PARAM_INT, &ouster_port, 0, NULL},
			{lidar_string, (char *) "imu_port", CARMEN_PARAM_INT, &ouster_imu_port, 0, NULL}
    };

    int num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
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
   
    std::string pcap_fname = "laser-data.pcap";
    std::string info_json_fname = "laser_metadata_info.json";

    //Carmen Stuffs
    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);
	
    carmen_param_check_version(argv[0]);
    //TODO pegar o ip etc daqui, rodar o modulo soh com o numero da mensagem / -lidar_id 
    read_parameters(argc, argv);

    const std::string sensor_hostname = ouster_ip;
    const std::string data_destination = host_ip;
    std::cerr << "Connecting to \"" << sensor_hostname << "\"... " << std::endl;
    
//TODO checar se vai precisa de porta com varios sensores conectados
    auto handle = sensor::init_client(sensor_hostname, data_destination);
    std::cout << "Open a pcap recording file: " << pcap_fname << std::endl;
    std::cout << " " << std::endl;
    auto record_handle = sensor_utils::record_initialize(pcap_fname,sensor_hostname, data_destination, 1024, false);
    
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
    
    save_single_line_in_file(info_json_fname, sensor::to_string(info));

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

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
    std::cerr << "Capturing points... " << std::endl;

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

//    while (true)
    {
        for (size_t i = 0; i < N_SCANS; i++)
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
               
                // for (int icol = 0; icol < pf.columns_per_packet; icol++) {
                const uint8_t *col_buf = pf.nth_col(0, packet_buf.get());
                // std::cout << col_buf[0] << "\t";
                const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
                
                sensor_utils::record_packet(*record_handle, ouster_port, ouster_port, packet_buf.get(), pf.lidar_packet_size, ((uint64_t) (ts.count() / 1000)));
                std::cerr << "recording data points in pcap file... " << std::endl;
            }

            // check if IMU data is available (but don't do anything with it)
            if (st & sensor::IMU_DATA)
            {
                sensor::read_imu_packet(*handle, packet_buf.get(), pf);
            }
        }
    }
    std::cerr << "closing... " << std::endl;
    sensor_utils::record_uninitialize(*record_handle);


    return 0;
}
