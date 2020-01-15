#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <sys/socket.h>
#include <netdb.h>
#include <math.h>
#include <carmen/pi_imu_interface.h>

#define PORT "3458"
#define SOCKET_DATA_PACKET_SIZE	2048
#define G 9.80665
#define ACCELEROMETER_CONSTANT (4.0 / 32768.0/*2^15*/)
#define MAGNETOMETER_CONSTANT (8.0 / 32768.0/*2^15*/)
#define GYROSCOPE_CONSTANT (500.0 / 32768.0/*2^15*/)

char *tcp_ip_address;

#define do_logger false

void
carmen_xsens_define_messages()
{
    IPC_RETURN_TYPE err;

    /* register xsens's global message */
    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

void
carmen_pi_imu_define_messages()
{
    IPC_RETURN_TYPE err;

    /* register xsens's global message */
    err = IPC_defineMsg(CARMEN_PI_IMU_NAME, IPC_VARIABLE_LENGTH, CARMEN_PI_IMU_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_PI_IMU_NAME);
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

	status = getaddrinfo("192.168.1.15", PORT, &host_info, &host_info_list);

	if (status != 0)
	{
		printf("--- Get_Addrinfo ERROR! ---\n");
		return (-1);
	}
	status = connect(pi_socket, host_info_list->ai_addr, host_info_list->ai_addrlen);

	if (status < 0)
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
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_publish_xsens_quat_message(carmen_xsens_global_quat_message xsens_quat_message)
{
	xsens_quat_message.timestamp = carmen_get_time();
	xsens_quat_message.host = carmen_get_host();

	// Acceleration
	xsens_quat_message.m_acc.x = xsens_quat_message.m_acc.x * G;
	xsens_quat_message.m_acc.y = xsens_quat_message.m_acc.y * G;
	xsens_quat_message.m_acc.z = xsens_quat_message.m_acc.z * G;
	xsens_quat_message.m_temp = 0.0;
	xsens_quat_message.m_count = 0;
	//printf("ei\n");
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &xsens_quat_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

void
publish_pi_imu_message(carmen_pi_imu_message_t message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_PI_IMU_NAME, &message);
	//printf("oi\n");
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_PI_IMU_NAME);
}
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
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	int pi_socket = stablished_connection_with_server();


	int valread;

	

	carmen_pi_imu_define_messages();
	carmen_xsens_define_messages();

	carmen_xsens_global_quat_message xsens_quat_message;
	carmen_pi_imu_message_t pi_imu_message;
	//pi_imu_message.imu_vector = (carmen_imu_t*) calloc (1, sizeof (carmen_imu_t));

	while (1)
	{

		unsigned char rpi_imu_data[SOCKET_DATA_PACKET_SIZE];
		// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error
		valread = recv(pi_socket, rpi_imu_data, SOCKET_DATA_PACKET_SIZE, MSG_WAITALL);
		if (valread == 0 || valread == -1) // 0 Connection lost due to server shutdown -1 Could not connect
		{
			close(pi_socket);
			pi_socket = trying_to_reconnect();
			continue;
		}
		else if ((valread == -1) || (valread != SOCKET_DATA_PACKET_SIZE))
			continue;

		sscanf((char *) rpi_imu_data, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf*\n",
				&(xsens_quat_message.m_acc.x), &(xsens_quat_message.m_acc.y), &(xsens_quat_message.m_acc.z),
				&(xsens_quat_message.m_gyr.x), &(xsens_quat_message.m_gyr.y), &(xsens_quat_message.m_gyr.z),
				&(xsens_quat_message.quat_data.m_data[0]), &(xsens_quat_message.quat_data.m_data[1]), 
				&(xsens_quat_message.quat_data.m_data[2]), &(xsens_quat_message.quat_data.m_data[3]),
				&(xsens_quat_message.m_mag.x), &(xsens_quat_message.m_mag.y), 
				&(xsens_quat_message.m_mag.z));
		//printf("%lf\n", xsens_quat_message.m_acc.x);
		pi_imu_message.imu_vector.accel.x = xsens_quat_message.m_acc.x;
		pi_imu_message.imu_vector.accel.y = xsens_quat_message.m_acc.y;
		pi_imu_message.imu_vector.accel.z = xsens_quat_message.m_acc.z;
		pi_imu_message.imu_vector.gyro.x =  xsens_quat_message.m_gyr.x;
		pi_imu_message.imu_vector.gyro.y = xsens_quat_message.m_gyr.y;
		pi_imu_message.imu_vector.gyro.z = xsens_quat_message.m_gyr.z;
		pi_imu_message.imu_vector.magnetometer.x = xsens_quat_message.m_mag.x;
		pi_imu_message.imu_vector.magnetometer.y = xsens_quat_message.m_mag.y;
		pi_imu_message.imu_vector.magnetometer.z = xsens_quat_message.m_mag.z;
		pi_imu_message.imu_vector.quat_data.m_data[0] = xsens_quat_message.quat_data.m_data[0];
		pi_imu_message.imu_vector.quat_data.m_data[1] = xsens_quat_message.quat_data.m_data[1];
		pi_imu_message.imu_vector.quat_data.m_data[2] = xsens_quat_message.quat_data.m_data[2];
		pi_imu_message.imu_vector.quat_data.m_data[3] = xsens_quat_message.quat_data.m_data[3];
		pi_imu_message.timestamp = carmen_get_time();
		pi_imu_message.host = carmen_get_host();

		publish_pi_imu_message(pi_imu_message);
		carmen_publish_xsens_quat_message(xsens_quat_message);
	}

	return (0);
}
