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

#include "../global/carmen.h"
/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "c_library_v2/ardupilotmega/mavlink.h"
#include "c_library_v2/ardupilotmega/mavlink_msg_ahrs2.h"
#include "c_library_v2/common/mavlink_msg_statustext.h"
#include "c_library_v2/common/mavlink_msg_attitude.h"
#include "c_library_v2/common/mavlink_msg_request_data_stream.h"
#include "c_library_v2/common/mavlink_msg_message_interval.h"


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();
mavlink_ahrs3_t ahrs2;
mavlink_attitude_t attitude_msg;
mavlink_statustext_t status_text;


int main(int argc, char* argv[])
{

	char help[] = "--help";
	printf("\n");
	printf("\tUse --help to usage:\n\n");


	char target_ip[100];
	char port_udp[100];

	float position[6] = {};
	struct sockaddr_in gcAddr;
//	struct sockaddr_in locAddr;
	struct sockaddr_in client;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcAddr);
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	//int success = 0;
	unsigned int temp = 0;

	// Check if --help flag was used
	if ((argc == 2) && (strcmp(argv[1], help) == 0))
    {
		printf("\n");
		printf("\tUsage:\n\n");
		printf("\t");
		printf("%s", argv[0]);
		printf("<port> \n");
		printf("\tThe computer running this program is the server, Ardupilot has to send udp packages to this computer\n\n");
		printf("\tDefault port 14550\n\n");
		exit(EXIT_FAILURE);
    }


	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// Change the target ip if parameter was given
	strcpy(port_udp, "14550");
	if (argc == 2)
    {
		strcpy(port_udp, argv[1]);
    }

	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = INADDR_ANY;
	gcAddr.sin_port = htons(14550);

//	memset(&fromAddr, 0, sizeof(fromAddr));
//	fromAddr.sin_family = AF_INET;
//	fromAddr.sin_addr.s_addr = htonl("10.42.0.21");
//	fromAddr.sin_port = htons(14550);

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
	if (-1 == bind(sock,(struct sockaddr *)&gcAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    }
	else
		printf("\n Listening UDP port: %s \n", port_udp);

	/* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
#else
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
#endif

    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }

	int cont1  = 0;
	int cont2  = 0;
	double time1 = 0.0;
	static double time2;

//	//REQUEST DATA STREAM
//	mavlink_msg_request_data_stream_pack(255, 0, &msg,1, 0, MAV_DATA_STREAM_POSITION, 10, 1);
//	len = mavlink_msg_to_send_buffer(buf, &msg);
//	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

//

	while(1)
    {
//		if(time1 == 0.0)
//			time1 = carmen_get_time();
//		if (carmen_get_time() - time1 >= 1)
//		{
//			/*Send Heartbeat */
//			mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
//			len = mavlink_msg_to_send_buffer(buf, &msg);
//			bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&fromAddr, sizeof(struct sockaddr_in));
//			printf("cont1 %d \n", cont1);
//			time1 = 0.0;

			////
			//
			////
			////
			////
			//		/* Send Status */
//			mavlink_msg_sys_status_pack(255, 10, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
//			len = mavlink_msg_to_send_buffer(buf, &msg);
//			bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&fromAddr, sizeof (struct sockaddr_in));

//			/* Send Local Position */
//			mavlink_msg_local_position_ned_pack(255, 10, &msg, microsSinceEpoch(),
//					position[0], position[1], position[2],
//					position[3], position[4], position[5]);
//			len = mavlink_msg_to_send_buffer(buf, &msg);
//			bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&fromAddr, sizeof(struct sockaddr_in));
			//
			/* Send attitude */
//			mavlink_msg_attitude_pack(10, 10, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
//			len = mavlink_msg_to_send_buffer(buf, &msg);
//			bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		}



		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;

//			printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];
//				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

					switch(msg.msgid) {
					case MAVLINK_MSG_ID_AHRS3: // ID for GLOBAL_POSITION_INT
					{
						// Get all fields in payload (into global_position)
						mavlink_msg_ahrs3_decode(&msg, &ahrs2);
						printf("AHR3 %f, %f, %f\n", ahrs2.roll, ahrs2.pitch, ahrs2.yaw);

					}
					break;
					case MAVLINK_MSG_ID_ATTITUDE:
					{
						if(cont2 == 0)
							time2 = carmen_get_time();
						cont2++;
						// Get just one field from payload
						printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
						mavlink_msg_attitude_decode(&msg, &attitude_msg);
						printf("Attitude %f, %f, %f\n", attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw);
					}
					break;
					case 0:
					{
						// Get just one field from payload
						mavlink_msg_command_long_pack(255, 1, &msg, 1, 0, 512, 0, 33, 100000,0,0,0,0,0);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

					}
					break;
					case MAVLINK_MSG_ID_STATUSTEXT:
					{
						// Get just one field from payload
//						printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
						mavlink_msg_statustext_decode(&msg, &status_text);
						printf("Status %d, %s\n", status_text.severity, status_text.text);
					}
					break;
					default:
						break;
					}
				}
			}

		}
		memset(buf, 0, BUFFER_LENGTH);
//		sleep(1); // Sleep one second
    }
}


/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{

	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_REALTIME, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;

	return micros;
}
#else
uint64_t microsSinceEpoch()
{

	struct timeval tv;

	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	return micros;
}
#endif
