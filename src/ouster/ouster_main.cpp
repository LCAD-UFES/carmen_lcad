
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "libouster_conn/os1.h"
#include "libouster_conn/os1_packet.h"

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include "ouster_config.h"


namespace OS1 = ouster::OS1;

static char *ouster_hostname = NULL;
static char *ouster_destination_ip = NULL;
static int ouster_sensor_id = 0;
static int ouster_publish_imu = 0;


/*********************************************************
		   --- Handlers ---
**********************************************************/


void
reallocate_message_if_necessary(carmen_velodyne_variable_scan_message **message_ptr, int n_horizontal_readings)
{
    carmen_velodyne_variable_scan_message *message = *message_ptr;

    // free message if the number of horizontal readings changed.
    if (message != NULL) 
    {
        if (message->number_of_shots != n_horizontal_readings)
        {
            for (int i = 0; i < message->number_of_shots; i++)
            {
                free(message->partial_scan[i].distance);
                free(message->partial_scan[i].intensity);
            }

            free(message->partial_scan);
            free(message);
            message = NULL;
        }
    }

    // allocate message if necessary
    if (message == NULL) 
    {
        message = (carmen_velodyne_variable_scan_message*) calloc (1, sizeof(carmen_velodyne_variable_scan_message));
        message->number_of_shots = n_horizontal_readings;
        message->partial_scan = (carmen_velodyne_shot*) calloc(n_horizontal_readings, sizeof(carmen_velodyne_shot));

        for (int i = 0; i < n_horizontal_readings; i++)
        {
            message->partial_scan[i].distance = (unsigned short*) calloc (H, sizeof(unsigned short));
            message->partial_scan[i].intensity = (unsigned char*) calloc (H, sizeof(unsigned char));
        }
    }

    (*message_ptr) = message;
}


void 
build_and_publish_variable_velodyne_message(uint8_t* buf) 
{
    static carmen_velodyne_variable_scan_message *message = NULL;

    reallocate_message_if_necessary(&message, W);
    
    static int next_m_id = W;
    static int64_t scan_ts = -1L;

    int index = 0;

    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) 
    {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (OS1::col_valid(col_buf) != 0xffffffff || m_id >= W) 
            continue; // @filipe: ??

        const uint64_t ts = OS1::col_timestamp(col_buf);
        float h_angle_0 = OS1::col_h_angle(col_buf);    

        if (m_id < next_m_id) 
        {
            // if not initializing with first packet
            if (scan_ts != -1)
            {
                // zero out remaining missing columns
                for (int i = index; i < W - next_m_id; i++, index++)
                {
                    message->partial_scan[i].angle = 0;
                    message->partial_scan[i].shot_size = 0;

                    for (int j = 0; j < H; j++)
                    {
                        message->partial_scan[i].distance[j] = (unsigned short) 0;
                        message->partial_scan[i].intensity[j] = (unsigned short) 0;
                    }
                }

                // publish
                message->host = carmen_get_host();
                message->timestamp = carmen_get_time(); // @filipe: use sensor timestamp
                carmen_velodyne_publish_variable_scan_message(message, ouster_sensor_id);
                index = 0;
            }

            scan_ts = ts;
            next_m_id = 0;
        }

        // fill zero out missing columns
        for (int i = index; i < m_id - next_m_id; i++, index++)
        {
            message->partial_scan[i].angle = 0;
            message->partial_scan[i].shot_size = 0;

            for (int j = 0; j < H; j++)
            {
                message->partial_scan[i].distance[j] = (unsigned short) 0;
                message->partial_scan[i].intensity[j] = (unsigned short) 0;
            }
        }

        next_m_id = m_id + 1;

        message->partial_scan[m_id].angle = h_angle_0;
        message->partial_scan[m_id].shot_size = H;

        for (uint8_t ipx = 0; ipx < H; ipx++) 
        {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t range = OS1::px_range(px_buf); // in mm
            //int ind = 3 * (m_id * H + ipx);

            //double h_angle = std::sin(beam_azimuth_angles.at(ipx) * 2 * M_PI / 360.0) + h_angle_0; // @filipe: ??
            //double v_angle = carmen_degrees_to_radians(beam_altitude_angles[ipx]);

            // @filipe: the precision of the range and intensity returned by the sensor are higher 
            // than what it is possible to store in the message.
            message->partial_scan[m_id].distance[ipx] = (unsigned short) range;
            message->partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_signal_photons(px_buf);
        }
    }
}


void 
handle_imu(uint8_t* buf __attribute__((unused))) 
{
    // TODO.
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("ouster sensor: disconnected.\n");
		exit(0);
	}
}


int 
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
			{(char*) "commandline", (char*) "sensor_hostname", CARMEN_PARAM_STRING, &ouster_hostname, 0, NULL},
			{(char*) "commandline", (char*) "ip_destination_computer", CARMEN_PARAM_STRING, &ouster_destination_ip, 0, NULL},
            {(char*) "commandline", (char*) "sensor_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
			{(char*) "commandline", (char*) "publish_imu", CARMEN_PARAM_ONOFF, &ouster_publish_imu, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


/*********************************************************
		   --- Main ---
**********************************************************/

int 
main(int argc, char** argv) 
{
    carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	carmen_velodyne_define_messages();

    std::shared_ptr<OS1::client> cli = OS1::init_client(ouster_hostname, ouster_destination_ip);
    
    if (!cli) 
    {
        std::cerr << "Failed to connect to client at: " << ouster_hostname << std::endl;
        return 1;
    }
    else
    {
        std::cerr << "Successfully connected to sensor: " << ouster_hostname << std::endl;
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    while (true) 
    {
        OS1::client_state st = OS1::poll_client(*cli);
        
        if (st & OS1::ERROR) 
        {
            // @filipe: what to do when the connection is lost? check what velodyne's module does.
            return 1;
        }
        else if (st & OS1::LIDAR_DATA) 
        {
            if (OS1::read_lidar_packet(*cli, lidar_buf))
                build_and_publish_variable_velodyne_message(lidar_buf);
        }
        else if (st & OS1::IMU_DATA) 
        {
            if (OS1::read_imu_packet(*cli, imu_buf)) 
                handle_imu(imu_buf);
        }
    }

    return 0;
}
