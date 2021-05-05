#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "libouster_conn/os1.h"
#include "libouster_conn/os1_packet.h"
#include "ouster_config.h"

#include <opencv/cv.hpp>

using namespace cv;

namespace OS1 = ouster::OS1;

#define INITIAL_MAX_NUM_SHOT 2048 // TODO usar 2048

char *ouster_ip = NULL;
char *host_ip = NULL;
int ouster_sensor_id = 0;
int ouster_publish_imu = 0;
int ouster_intensity_type = 0;


void
setup_message(carmen_velodyne_variable_scan_message &msg, int number_of_shots, int shot_size)
{
	msg.partial_scan = (carmen_velodyne_shot *) malloc ((number_of_shots + 1) * sizeof(carmen_velodyne_shot));
	
    for (int i = 0 ; i <= number_of_shots; i++)
	{
		msg.partial_scan[i].shot_size = shot_size;
		msg.partial_scan[i].distance  = (unsigned short*) malloc (shot_size * sizeof(unsigned short));
		msg.partial_scan[i].intensity = (unsigned char*)  malloc (shot_size * sizeof(unsigned char));
	}
	msg.host = carmen_get_host();
}


void
reallocate_message_if_necessary_old(carmen_velodyne_variable_scan_message **message_ptr, int n_horizontal_readings)
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
update_color(Mat &img, int row, int col, unsigned char intensity, unsigned char refl, unsigned char noise)
{
	int p;
	
	p = row * img.cols + col;
	img.data[p] = intensity;
	p = (64 + row) * img.cols + col;
	img.data[p] = refl;	
	p = (128 + row) * img.cols + col;	
	img.data[p] = noise;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                               Publishers                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////


void
build_and_publish_variable_velodyne_message_old(uint8_t* buf) 
{
    static carmen_velodyne_variable_scan_message *message = NULL;
    static Mat viewer = Mat::zeros(3 * 64, W, CV_8UC1);
    static int cont = 0; // Essa funcao seta o number_of_shots da mensagem para 1024 hard codded isso faz com q as primeiras mensagens que tem menos shots causem problemas em outros modulos

    reallocate_message_if_necessary_old(&message, W);
    
    static int next_m_id = W;
    static int64_t scan_ts = -1L;

    int index = 0;
    
    // This is the solution used by ouster to store the packets until a complete
    // scan is complete. TODO: develop a a more elegant (and efficient, if possible) solution.
    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) 
    {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (OS1::col_valid(col_buf) != 0xffffffff || m_id >= W) 
            continue; // what to do in this case??

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
                        
                        // update 1
						// update_color(viewer, j, i, 0, 0, 0);
                    }
                }

                // publish
                message->host = carmen_get_host();
                message->timestamp = carmen_get_time(); // @filipe: use sensor timestamp
     
                if (cont < 5)
                {
                    cont++;
                }
                else
                {
                    printf("Publicou\n");
                    carmen_velodyne_publish_variable_scan_message(message, 4);
                }
                // IMAGE VIEWER
                // imshow("viewer", viewer);
                // waitKey(1);
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
                
                // update 2
                update_color(viewer, j, i, 0, 0, 0);
            }
        }

        next_m_id = m_id + 1;

        message->partial_scan[m_id].angle = h_angle_0;
        // message->partial_scan[m_id].angle = carmen_radians_to_degrees(h_angle_0);
        message->partial_scan[m_id].shot_size = H;

        for (uint8_t ipx = 0; ipx < H; ipx++) 
        {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t range = OS1::px_range(px_buf); // in mm
            uint16_t intensity = 0;
            
            if (ouster_intensity_type == INTENSITY) 
                intensity = OS1::px_signal_photons(px_buf);
            else if (ouster_intensity_type == REFLECTIVITY)
                intensity = OS1::px_reflectivity(px_buf);
            else if (ouster_intensity_type == NOISE)
                intensity = OS1::px_noise_photons(px_buf);
            else
                fprintf(stderr, "Warning: Invalid intensity type: %d. Filling intensity with zeros.\n", intensity);

            // the precision of the range and intensity returned by the sensor are higher 
            // than what it is possible to store in the message.
            message->partial_scan[m_id].distance[ipx] = (unsigned short) range;
            message->partial_scan[m_id].intensity[ipx] = (unsigned char) intensity;

	        // update 3
	        update_color(viewer, ipx, m_id, OS1::px_signal_photons(px_buf), OS1::px_reflectivity(px_buf), OS1::px_noise_photons(px_buf));
	        //update_color(viewer, ipx, m_id, 0, 0, 0);
        }
    }
}


void 
build_and_publish_imu_message(uint8_t* buf __attribute__((unused))) 
{
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    // It is possible to acquire the accelerometer timestamp using 'imu_accel_ts(buf)'
    // and the gyroscope timestamp using 'imu_gyro_ts(buf)'.
    // uint64_t imu_timestamp;
    // imu_timestamp = OS1::imu_sys_ts(buf);

    // imu linear acceleration
    acc_x = OS1::imu_la_x(buf);
    acc_y = OS1::imu_la_y(buf);
    acc_z = OS1::imu_la_z(buf);

    // imu angular velocity
    gyro_x = OS1::imu_av_x(buf);
    gyro_y = OS1::imu_av_y(buf);
    gyro_z = OS1::imu_av_z(buf);

    // publish imu message.
    carmen_velodyne_gps_message imu_message;
    memset(&imu_message, 0, sizeof(carmen_velodyne_gps_message));
    
    imu_message.gyro1 = gyro_x;
    imu_message.gyro2 = gyro_y;
    imu_message.gyro3 = gyro_z;
    imu_message.accel1_x = acc_x;
    imu_message.accel2_x = acc_y;
    imu_message.accel3_x = acc_z;
    imu_message.timestamp = carmen_get_time();
    imu_message.host = carmen_get_host();

    carmen_velodyne_publish_gps_message(&imu_message);
}


void
build_and_publish_variable_scan_message_new(uint8_t* buf, carmen_velodyne_variable_scan_message &message1, carmen_velodyne_variable_scan_message &message2,
    carmen_velodyne_variable_scan_message &message3, carmen_velodyne_variable_scan_message &message4)
{
    static int next_m_id = W, number_of_shots = 0;
    static int64_t scan_ts = -1L;

    // This is the solution used by ouster to store the packets until a scan is complete. TODO: develop a a more elegant (and efficient, if possible) solution.
    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) 
    {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);

        if (OS1::col_valid(col_buf) != 0xffffffff || m_id >= W) // Drop invalid, out-of-bounds, data in case of misconfiguration
            continue;

        const uint64_t ts = OS1::col_timestamp(col_buf);
        float h_angle_0 = OS1::col_h_angle(col_buf);    

        if (m_id < next_m_id) 
        {
            if (scan_ts != -1)
            {
                message1.timestamp = carmen_get_time(); // TODO use sensor timestamp
                message2.timestamp = carmen_get_time();
                message3.timestamp = carmen_get_time();
                message4.timestamp = carmen_get_time();

                message1.number_of_shots = number_of_shots;
                message2.number_of_shots = number_of_shots;
                message3.number_of_shots = number_of_shots;
                message4.number_of_shots = number_of_shots;
                printf("Enviou\n");
                carmen_velodyne_publish_variable_scan_message(&message1, 0);
                carmen_velodyne_publish_variable_scan_message(&message2, 1);
                carmen_velodyne_publish_variable_scan_message(&message3, 2);
                carmen_velodyne_publish_variable_scan_message(&message4, 3);

                number_of_shots = 0;
            }

            scan_ts = ts;
            next_m_id = 0;
        }
        next_m_id = m_id + 1;

        number_of_shots++;

        message1.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) + 3.164);
        message2.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) + 1.055);
        message3.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) - 1.055);
        message4.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) - 3.164);

        for (uint8_t ipx = 0, r_index = 0; ipx < 16; ipx++, r_index += 4)
        {
            printf("%d %d \n", ipx, r_index);
            // WARNING the precision of the range and intensity returned by the sensor are higher than what is possible to store in the message.  Range [20 bits] - Range in millimeters, discretized to the nearest 12 millimeters.
            message1.partial_scan[m_id].distance[ipx] = (unsigned short) OS1::px_range(OS1::nth_px(r_index,     col_buf));
            message2.partial_scan[m_id].distance[ipx] = (unsigned short) OS1::px_range(OS1::nth_px(r_index + 1, col_buf));
            message3.partial_scan[m_id].distance[ipx] = (unsigned short) OS1::px_range(OS1::nth_px(r_index + 2, col_buf));
            message4.partial_scan[m_id].distance[ipx] = (unsigned short) OS1::px_range(OS1::nth_px(r_index + 3, col_buf));
            
            if (ouster_intensity_type == INTENSITY)
            {
                message1.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_signal_photons(OS1::nth_px(ipx,     col_buf));
                message2.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_signal_photons(OS1::nth_px(ipx + 1, col_buf));
                message3.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_signal_photons(OS1::nth_px(ipx + 2, col_buf));
                message4.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_signal_photons(OS1::nth_px(ipx + 3, col_buf));
            }
            else if (ouster_intensity_type == REFLECTIVITY)
            {
                message1.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_reflectivity(OS1::nth_px(ipx,     col_buf));
                message2.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_reflectivity(OS1::nth_px(ipx + 1, col_buf));
                message3.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_reflectivity(OS1::nth_px(ipx + 2, col_buf));
                message4.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_reflectivity(OS1::nth_px(ipx + 3, col_buf));
            }
            else if (ouster_intensity_type == NOISE)
            {
                message1.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_noise_photons(OS1::nth_px(ipx,     col_buf));
                message2.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_noise_photons(OS1::nth_px(ipx + 1, col_buf));
                message3.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_noise_photons(OS1::nth_px(ipx + 2, col_buf));
                message4.partial_scan[m_id].intensity[ipx] = (unsigned char) OS1::px_noise_photons(OS1::nth_px(ipx + 3, col_buf));
            }
        }
    }
}


void
build_and_publish_variable_scan_message(uint8_t* buf) 
{
    static carmen_velodyne_variable_scan_message *message = NULL;
    static int next_m_id = W;
    static int64_t scan_ts = -1L;
    int index = 0;

    reallocate_message_if_necessary_old(&message, W);
    
    // This is the solution used by ouster to store the packets until a complete scan is complete. TODO: develop a a more elegant (and efficient, if possible) solution.
    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) 
    {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (OS1::col_valid(col_buf) != 0xffffffff || m_id >= W) 
            continue; // what to do in this case??

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

                message->host = carmen_get_host();
                message->timestamp = carmen_get_time(); // @filipe: use sensor timestamp
     
                printf("Publicou\n");
                carmen_velodyne_publish_variable_scan_message(message, 4);
                index = 0;
            }

            scan_ts = ts;
            next_m_id = 0;
        }

        next_m_id = m_id + 1;

        // message->partial_scan[m_id].angle = h_angle_0;
        message->partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0));

        // printf("%lf\n", message->partial_scan[m_id].angle);

        // message->partial_scan[m_id].angle = carmen_radians_to_degrees(h_angle_0);
        message->partial_scan[m_id].shot_size = H;

        for (uint8_t ipx = 0; ipx < H; ipx++) 
        {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t range = OS1::px_range(px_buf); // in mm
            uint16_t intensity = 0;
            
            if (ouster_intensity_type == INTENSITY) 
                intensity = OS1::px_signal_photons(px_buf);
            else if (ouster_intensity_type == REFLECTIVITY)
                intensity = OS1::px_reflectivity(px_buf);
            else if (ouster_intensity_type == NOISE)
                intensity = OS1::px_noise_photons(px_buf);
            else
                fprintf(stderr, "Warning: Invalid intensity type: %d. Filling intensity with zeros.\n", intensity);

            // the precision of the range and intensity returned by the sensor are higher 
            // than what it is possible to store in the message.
            // if (ipx < 16)
            message->partial_scan[m_id].distance[ipx] = (unsigned short) range;
            // else
            // message->partial_scan[m_id].distance[ipx] = 0;
            message->partial_scan[m_id].intensity[ipx] = (unsigned char) intensity;
        }
    }
}


// typedef struct
// {
// 	int shot_size;
// 	unsigned short  *distance;
// 	unsigned char *intensity;
// 	double angle;              // In degrees
// }carmen_velodyne_shot;

// typedef struct
// {
// 	int number_of_shots;
// 	carmen_velodyne_shot *partial_scan;
// 	double timestamp;
// 	char *host;


void
build_and_publish_variable_scan_message_4_lidars(uint8_t* buf, carmen_velodyne_variable_scan_message &message1, carmen_velodyne_variable_scan_message &message2,
    carmen_velodyne_variable_scan_message &message3, carmen_velodyne_variable_scan_message &message4)
{
    static int next_m_id = W;
    static int64_t scan_ts = -1L;

    // This is the solution used by ouster to store the packets until a complete scan is complete. TODO: develop a a more elegant (and efficient, if possible) solution.
    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) 
    {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (OS1::col_valid(col_buf) != 0xffffffff || m_id >= W) 
            continue; // what to do in this case??

        const uint64_t ts = OS1::col_timestamp(col_buf);
        float h_angle_0 = OS1::col_h_angle(col_buf);    

        if (m_id < next_m_id) 
        {
            // if not initializing with first packet
            if (scan_ts != -1)
            {
                message1.timestamp = message2.timestamp = message3.timestamp = message4.timestamp = carmen_get_time(); // TODO use sensor timestamp

                message1.number_of_shots = next_m_id;

                printf("Publicou %d\n", message1.number_of_shots);
                carmen_velodyne_publish_variable_scan_message(&message1, 0);
                carmen_velodyne_publish_variable_scan_message(&message2, 1);
                carmen_velodyne_publish_variable_scan_message(&message3, 2);
                carmen_velodyne_publish_variable_scan_message(&message4, 3);
            }

            scan_ts = ts;
            next_m_id = 0;
        }

        next_m_id = m_id + 1;

        message1.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) + 3.164);
        message2.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) + 1.055);
        message3.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) - 1.055);
        message4.partial_scan[m_id].angle = carmen_normalize_angle_degree(carmen_radians_to_degrees(h_angle_0) - 3.164);

        for (uint8_t ipx = 0, j0 = 0, j1 = 0, j2 = 0, j3 = 0; ipx < H; ipx++) 
        {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t range = OS1::px_range(px_buf); // in mm
            uint16_t intensity = 0;
            
            if (ouster_intensity_type == INTENSITY) 
                intensity = OS1::px_signal_photons(px_buf);
            else if (ouster_intensity_type == REFLECTIVITY)
                intensity = OS1::px_reflectivity(px_buf);
            else if (ouster_intensity_type == NOISE)
                intensity = OS1::px_noise_photons(px_buf);
            else
                fprintf(stderr, "Warning: Invalid intensity type: %d. Filling intensity with zeros.\n", intensity);

            switch (ipx % 4)
            {
            case 0:
                // printf("0\n");
                message1.partial_scan[m_id].distance[j0] = (unsigned short)range;
                message1.partial_scan[m_id].intensity[j0] = (unsigned char)intensity;
                j0++;
                break;

            case 1:
                message2.partial_scan[m_id].distance[j1] = (unsigned short)range;
                message2.partial_scan[m_id].intensity[j1] = (unsigned char)intensity;
                j1++;
                break;

            case 2:
                message3.partial_scan[m_id].distance[j2] = (unsigned short)range;
                message3.partial_scan[m_id].intensity[j2] = (unsigned char)intensity;
                j2++;
                break;

            case 3:
                message4.partial_scan[m_id].distance[j3] = (unsigned short)range;
                message4.partial_scan[m_id].intensity[j3] = (unsigned char)intensity;
                j3++;
                break;
            }
        }
    }
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


///////////////////////////////////////////////////////////////////////////////////////////////
//                               Initializations                                             //
///////////////////////////////////////////////////////////////////////////////////////////////


void 
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char*) "commandline", (char*) "sensor_ip", CARMEN_PARAM_STRING, &ouster_ip, 0, NULL},
		{(char*) "commandline", (char*) "host_ip", CARMEN_PARAM_STRING, &host_ip, 0, NULL},
        {(char*) "commandline", (char*) "sensor_id", CARMEN_PARAM_INT, &ouster_sensor_id, 0, NULL},
        {(char*) "commandline", (char*) "intensity_type", CARMEN_PARAM_INT, &ouster_intensity_type, 0, NULL}, 
		{(char*) "commandline", (char*) "publish_imu", CARMEN_PARAM_ONOFF, &ouster_publish_imu, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

    if (ouster_intensity_type < 1 || ouster_intensity_type > 3)
    {
        fprintf(stderr, "Invalid intensity type: %d. Filling intensity with zeros.\n", ouster_intensity_type);
        exit(0);
    }
}


int 
main(int argc, char** argv) 
{
    carmen_velodyne_variable_scan_message message1;  // 4 messages are published because of misalignment of the 64 beans in a shot
    carmen_velodyne_variable_scan_message message2;  // Beans are alignment in groups of 16 beans
    carmen_velodyne_variable_scan_message message3;
    carmen_velodyne_variable_scan_message message4;

    setup_message(message1, INITIAL_MAX_NUM_SHOT, 16);
    setup_message(message2, INITIAL_MAX_NUM_SHOT, 16);
    setup_message(message3, INITIAL_MAX_NUM_SHOT, 16);
    setup_message(message4, INITIAL_MAX_NUM_SHOT, 16);

    carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);
	
    carmen_param_check_version(argv[0]);
	
    read_parameters(argc, argv);
	
    carmen_velodyne_define_messages();

    fprintf(stderr, "Intensity type: '%s'\n", intensity_type_to_string(ouster_intensity_type));

    std::shared_ptr<OS1::client> cli = OS1::init_client(ouster_ip, host_ip);

    if (!cli) 
    {
        std::cerr << "Failed to connect to client at: " << ouster_ip << std::endl;
        return 1;
    }
    else
    {
        std::cerr << "Successfully connected to sensor: " << ouster_ip << std::endl;
        std::cerr << "Wait 15-20 seconds to start receiving point clouds." << std::endl;
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    while (true) 
    {
        OS1::client_state st = OS1::poll_client(*cli);

        // printf("0\n");
        
        if (st & OS1::ERROR) 
        {
            return 1;
        }
        else if (st & OS1::LIDAR_DATA) 
        {
            // printf("1\n");
            if (OS1::read_lidar_packet(*cli, lidar_buf))
            {
                // printf("2\n");
                // build_and_publish_variable_velodyne_message_old(lidar_buf);
                // build_and_publish_variable_scan_message_new(lidar_buf, message1, message2, message3, message4);
                // build_and_publish_variable_scan_message(lidar_buf);
                build_and_publish_variable_scan_message_4_lidars(lidar_buf, message1, message2, message3, message4);
            }
        }
        else if (st & OS1::IMU_DATA) 
        {
            // The IMU is running in a smaller frequency than it should be. TODO: fix it!
            if (OS1::read_imu_packet(*cli, imu_buf)) 
                build_and_publish_imu_message(imu_buf);
        }
    }

    return 0;
}