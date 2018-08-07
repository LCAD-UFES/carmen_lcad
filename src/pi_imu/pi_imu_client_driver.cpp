#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <sys/socket.h>
#include <netdb.h>
#include <math.h>


char* tcp_ip_address;

#define PORT "3457"
#define SOCKET_DATA_PACKET_SIZE	2048
#define G 9.80665
#define ACCELEROMETER_CONSTANT (4.0 / 32768.0/*2^15*/)
#define MAGNETOMETER_CONSTANT (8.0 / 32768.0/*2^15*/)
#define GYROSCOPE_CONSTANT (500.0 / 32768.0/*2^15*/)

void
carmen_xsens_define_messages()
{
    IPC_RETURN_TYPE err;

    /* register xsens's global message */
    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

int
stablished_connection_with_server()
{
	struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
	int pi_socket = 0, status;

	if ((pi_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("--- Socket creation error! ---\n");
		return -1;
	}
	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

	status = getaddrinfo("10.42.0.28", PORT, &host_info, &host_info_list);

	if (status != 0)
	{

		printf("--- Get_Addrinfo ERROR! ---\n");
		return (-1);
	}

	status = connect(pi_socket, host_info_list->ai_addr, host_info_list->ai_addrlen);

	if(status < 0)
	{
		printf("--- Connection Failed! ---\n");
		return (-1);
	}

	printf("--- Connection established successfully! ---\n");
	return (pi_socket);
}


int
trying_to_reconnect()
{
	int pi_socket = stablished_connection_with_server();

	while (pi_socket == -1)
	{
		sleep((unsigned int) 5);
		pi_socket = stablished_connection_with_server();
	}
	return (pi_socket);
}
///////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Handlers																					 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////


void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////




int
main(int argc, char **argv)
{
	carmen_xsens_global_quat_message xsens_quat_message;


	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

//	int camera_number = read_parameters(argc, argv, &msg, cam_config);
//	initialize_message(&msg);

//	carmen_bumblebee_basic_define_messages(camera_number);

	int pi_socket = stablished_connection_with_server();

	double AccY = 0.0;
	double AccX = 0.0;
	double AccZ = 0.0;

	double GyrX = 0.0;
	double GyrY = 0.0;
	double GyrZ = 0.0;

	double MagX = 0.0;
	double MagY = 0.0;
	double MagZ = 0.0;

	/*double pressure;
	double temperature;*/

	int valread;

	IPC_RETURN_TYPE err;

	carmen_xsens_define_messages();

	while (1)
	{
		unsigned char rpi_imu_data[SOCKET_DATA_PACKET_SIZE];
		// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error
		valread = recv(pi_socket, rpi_imu_data, SOCKET_DATA_PACKET_SIZE, MSG_WAITALL);

		if (valread == 0) // Connection lost due to server shutdown.
		{
			close(pi_socket);
			pi_socket = trying_to_reconnect();
			continue;
		}
		else if ((valread == -1) || (valread != SOCKET_DATA_PACKET_SIZE))
			continue;

		int magRaw[3];
		int accRaw[3];
		int gyrRaw[3];

		sscanf((char *) rpi_imu_data, "%d %d %d %d %d %d %d %d %d *\n", &(accRaw[0]), &(accRaw[1]), &(accRaw[2]), &(gyrRaw[0]), &(gyrRaw[1]),  &(gyrRaw[2]),
				&(magRaw[0]), &(magRaw[1]), &(magRaw[2]));

		//printf("%d %d %d %d %d %d %d %d %d **\n", accRaw[0], accRaw[1], accRaw[2], gyrRaw[0], gyrRaw[1], gyrRaw[2],
				//magRaw[0], magRaw[1], magRaw[2]);

		AccX = accRaw[0] * ACCELEROMETER_CONSTANT * G;
		AccY = accRaw[1] * ACCELEROMETER_CONSTANT * G;
		AccZ = accRaw[2] * ACCELEROMETER_CONSTANT * G;

		GyrX = (gyrRaw[0] * GYROSCOPE_CONSTANT * M_PI) / 180.0;
		GyrY = (gyrRaw[1] * GYROSCOPE_CONSTANT * M_PI) / 180.0;
		GyrZ = (gyrRaw[2] * GYROSCOPE_CONSTANT * M_PI) / 180.0;

		MagX = magRaw[0] * MAGNETOMETER_CONSTANT * 1000;
		MagY = magRaw[1] * MAGNETOMETER_CONSTANT * 1000;
		MagZ = magRaw[2] * MAGNETOMETER_CONSTANT * 1000;

		printf("ACELEROMETRO = X:%f m/s^2 Y:%f m/s^2 Z:%f m/s^2\n", AccX, AccY, AccZ);
		printf("GIROSCÓPIO = X:%f radps Y:%f radps Z:%f radps\n", GyrX, GyrY, GyrZ);
		printf("MAGNETOMETRO = X:%f mgauss Y:%f mgauss Z:%f mgauss\n", MagX, MagY, MagZ);

		// publishing  carmen_xsens_global_quat_message

		 //Acceleration
		xsens_quat_message.m_acc.x = AccX;
		xsens_quat_message.m_acc.y = AccY;
		xsens_quat_message.m_acc.z = AccZ;

		//Gyro
		xsens_quat_message.m_gyr.x = GyrX;
		xsens_quat_message.m_gyr.y = GyrY;
		xsens_quat_message.m_gyr.z = GyrZ;

		//Magnetism
		xsens_quat_message.m_mag.x = MagX;
		xsens_quat_message.m_mag.y = MagY;
		xsens_quat_message.m_mag.z = MagZ;

		xsens_quat_message.quat_data.m_data[0] = 0.0;
		xsens_quat_message.quat_data.m_data[1] = 0.0;
		xsens_quat_message.quat_data.m_data[2] = 0.0;
		xsens_quat_message.quat_data.m_data[3] = 0.0;


		xsens_quat_message.m_temp = 0.0;
		xsens_quat_message.m_count = 0;

		//Timestamp
		xsens_quat_message.timestamp = carmen_get_time();

		//Host
		xsens_quat_message.host = carmen_get_host();

		err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &xsens_quat_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_NAME);

		/*printf("TEMPERATURA =  %f C\n", temperature);
		printf("PRESSÃO = %f mb\n", pressure);*/
		//		publish_image_message(camera_number, &msg);

		//imshow("Pi Camera Driver", cv_image);  waitKey(1);
	}
	return 0;
}
