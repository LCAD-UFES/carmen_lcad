/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.


 I compiled this program sucessfully on Ubuntu 10.04 with the following command

 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c

 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif

#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/xsens_mtig_interface.h>
/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "c_library_v2/ardupilotmega/mavlink.h"
#include "c_library_v2/common/mavlink_msg_heartbeat.h"
#include "c_library_v2/ardupilotmega/mavlink_msg_ahrs2.h"
#include "c_library_v2/common/mavlink_msg_attitude.h"
#include "c_library_v2/common/mavlink_msg_global_position_int.h"
#include "c_library_v2/common/mavlink_msg_raw_imu.h"


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define PORT 14550;

uint64_t microsSinceEpoch();
mavlink_ahrs2_t ahrs2;
mavlink_attitude_t attitude_msg;
mavlink_heartbeat_t heart_msg;
mavlink_global_position_int_t global_pos_msg;
//mavlink_attitude_quaternion_t attitude_quaternion_msg;

mavlink_raw_imu_t raw_imu;


mavlink_message_t msg;
mavlink_message_t msg_send;
mavlink_status_t status;

int raw_msg_received = 0;
int attitude_msg_received = 0;

static int first = 1;


socklen_t mavlink_socket;
uint8_t buf[BUFFER_LENGTH];
struct sockaddr_in from;
socklen_t fromlen = sizeof(from);
ssize_t recsize;



void
shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("xsens_mtig: disconnected.\n");
     exit(0);
  }
}


//static int
//read_parameters(int argc, char **argv)
//{
//	return 0;
//}
//

static void
define_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

	err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////

void
build_and_publish_xsens_mti_quat_message()
{
	carmen_xsens_global_quat_message carmen_xsens_message;
	raw_msg_received = 0;
	attitude_msg_received = 0;
	float attitude_quaternion[4];

	mavlink_euler_to_quaternion(attitude_msg.roll, attitude_msg.pitch,attitude_msg.yaw, attitude_quaternion);

	carmen_xsens_message.host = carmen_get_host();
	carmen_xsens_message.quat_data.m_data[0] = attitude_quaternion[0];
	carmen_xsens_message.quat_data.m_data[1] = attitude_quaternion[1];
	carmen_xsens_message.quat_data.m_data[2] = attitude_quaternion[2];
	carmen_xsens_message.quat_data.m_data[3] = attitude_quaternion[3];
	carmen_xsens_message.m_acc.x = raw_imu.xacc;
	carmen_xsens_message.m_acc.y = raw_imu.yacc;
	carmen_xsens_message.m_acc.z = raw_imu.zacc;

	carmen_xsens_message.m_gyr.x = raw_imu.xgyro;
	carmen_xsens_message.m_gyr.y = raw_imu.ygyro;
	carmen_xsens_message.m_gyr.z = raw_imu.zgyro;

	carmen_xsens_message.m_mag.x = raw_imu.xmag;
	carmen_xsens_message.m_mag.y = raw_imu.ymag;
	carmen_xsens_message.m_mag.z = raw_imu.zmag;

	carmen_xsens_message.m_temp = raw_imu.temperature/100;
	carmen_xsens_message.m_count = 0.0;


	publish_mti_quat_message(carmen_xsens_message);

}
///////////////////////////////////////////////////////////////////////////////////////////////



socklen_t
connect_to_navio2_ardupilot()
{
	socklen_t mavlink_socket, length;
	struct sockaddr_in server;

	mavlink_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (mavlink_socket == 0)
		printf("Error: socket can't be open");

	length = sizeof(server);
	bzero(&server, length);
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port= PORT;
	if (bind(mavlink_socket, (struct sockaddr *)&server, length) < 0)
		printf("Error binding");

	return mavlink_socket;
}


int
trying_to_reconnect()
{
	int nav_socket = connect_to_navio2_ardupilot();

	while (nav_socket == -1)
	{
		sleep(5);
		nav_socket = connect_to_navio2_ardupilot();
	}
	return (nav_socket);
}


void
request_stream_packages(socklen_t mavlink_socket, struct sockaddr_in from, mavlink_message_t msg, int requested_message_id, int stream_rate)
{
	uint8_t buf_send[BUFFER_LENGTH];
	memset(buf_send, 0, BUFFER_LENGTH);

	mavlink_msg_request_data_stream_pack(255, 0, &msg_send, msg.sysid, msg.compid, requested_message_id, stream_rate, 1);
	int len = mavlink_msg_to_send_buffer(buf_send, &msg_send);
	sendto(mavlink_socket, buf_send, len, 0, (struct sockaddr*)&from, sizeof(struct sockaddr_in));
	first = 0;

}


void
process_messages(ssize_t recsize, socklen_t mavlink_socket, struct sockaddr_in from, uint8_t *buf)
{
	int i;
	for (i = 0; i < recsize; ++i)
	{
		//				printf("%02x ", (unsigned char)temp);
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
		{
			switch(msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT: // ID for GLOBAL_POSITION_INT
			{
				// Get all fields in payload (into global_position)
				mavlink_msg_heartbeat_decode(&msg, &heart_msg);
				printf("heartbeat %d, %d\n", heart_msg.autopilot, heart_msg.system_status);
				if (first)
				{
					printf("Requesting STREAM_EXTRA (AHRS, ATTITUDE MESAGES\n");
					request_stream_packages(mavlink_socket, from, msg, MAV_DATA_STREAM_ALL, 10);
					request_stream_packages(mavlink_socket, from, msg, MAV_DATA_STREAM_RAW_SENSORS, 10);
					first = 0;
				}
			}
			break;

			case MAVLINK_MSG_ID_ATTITUDE:
			{
				mavlink_msg_attitude_decode(&msg, &attitude_msg);
//				printf("attitude_quaternion_msg %f, %f, %f, %f\n", attitude_quaternion_msg.q1, attitude_quaternion_msg.q2,
//						attitude_quaternion_msg.q3, attitude_quaternion_msg.q4);
				attitude_msg_received = 1;
			}
			break;

			case MAVLINK_MSG_ID_RAW_IMU:
			{
				mavlink_msg_raw_imu_decode(&msg, &raw_imu);

				printf("Raw_IMU ID:%d,  %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
						raw_imu.id,
						raw_imu.xacc,
						raw_imu.yacc,
						raw_imu.zacc,
						raw_imu.xgyro,
						raw_imu.ygyro,
						raw_imu.zgyro,
						raw_imu.xmag,
						raw_imu.ymag,
						raw_imu.zmag);

				raw_msg_received = 1;
			}
			break;
			default:
				break;
			}
		}
	}
}

int main(int argc, char** argv)
{

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
//	read_parameters(argc, argv);
//	initialize_xsens_message(&xsens_msg);
	define_ipc_messages();
	signal(SIGINT, shutdown_module);

	mavlink_socket = connect_to_navio2_ardupilot();

	while (1)
	{
		memset(buf, 0, BUFFER_LENGTH);

		recsize = recvfrom(mavlink_socket, buf, BUFFER_LENGTH, 0,(struct sockaddr *)&from, sizeof(struct sockaddr_in));

		if (recsize == 0 || recsize == -1) // 0 Connection lost due to server shutdown -1 Could not connect
		{
			close(mavlink_socket);
			mavlink_socket = trying_to_reconnect();
			continue;
		}
		//			printf("Recebendo mensagem\n");
		process_messages(recsize, mavlink_socket, from, buf);

		if(raw_msg_received && attitude_msg_received)
			build_and_publish_xsens_mti_quat_message();
	}
}
