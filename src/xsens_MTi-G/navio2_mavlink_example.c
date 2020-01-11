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

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "c_library_v2/ardupilotmega/mavlink.h"
#include "c_library_v2/common/mavlink_msg_heartbeat.h"
#include "c_library_v2/ardupilotmega/mavlink_msg_ahrs2.h"
#include "c_library_v2/common/mavlink_msg_statustext.h"
#include "c_library_v2/common/mavlink_msg_attitude.h"
#include "c_library_v2/common/mavlink_msg_request_data_stream.h"
#include "c_library_v2/common/mavlink_msg_message_interval.h"


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define PORT 14550

uint64_t microsSinceEpoch();
mavlink_heartbeat_t heart_msg;
mavlink_ahrs3_t ahrs2;
mavlink_attitude_t attitude_msg;
mavlink_statustext_t status_text;

static int first = 1;

int main(int argc, char* argv[])
{

	socklen_t tam_cliente;
	socklen_t meuSocket;
	// MTU padrão pela IETF

	struct sockaddr_in servidor;
	struct sockaddr_in client;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	//int success = 0;
	unsigned int temp = 0;

	meuSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&servidor, 0, sizeof(servidor));
	servidor.sin_family = AF_INET;
	servidor.sin_addr.s_addr = INADDR_ANY;
	servidor.sin_port = htons(PORT);

	if (-1 == bind(meuSocket,(struct sockaddr *)&servidor, sizeof(servidor)))
	{
		perror("error bind failed");
		close(meuSocket);
		exit(EXIT_FAILURE);
	}
	else
		printf("\n Listening UDP port: %d \n", PORT);

	//	memset(&fromAddr, 0, sizeof(fromAddr));
	//	fromAddr.sin_family = AF_INET;
	//	fromAddr.sin_addr.s_addr = htonl("10.42.0.21");
	//	fromAddr.sin_port = htons(14550);

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */


	//	//REQUEST DATA STREAM
	//	mavlink_msg_request_data_stream_pack(255, 0, &msg,1, 0, MAV_DATA_STREAM_POSITION, 10, 1);
	//	len = mavlink_msg_to_send_buffer(buf, &msg);
	//	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	//

	while(1)
	{
		tam_cliente=sizeof(struct sockaddr_in);

		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(meuSocket, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&client, &tam_cliente);
		if (recsize > 0)
		{
			printf("Recebi:%s de <endereço:%s> <porta:%d>\n",buf,inet_ntoa(client.sin_addr),ntohs(client.sin_port));
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_message_t msg_send;
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

					switch(msg.msgid)
					{
					case MAVLINK_MSG_ID_HEARTBEAT: // ID for GLOBAL_POSITION_INT
					{
						// Get all fields in payload (into global_position)
						mavlink_msg_heartbeat_decode(&msg, &heart_msg);
						printf("heartbeat %d, %d\n", heart_msg.autopilot, heart_msg.system_status);
						if (first)
						{
							uint8_t buf_send[BUFFER_LENGTH];
							memset(buf_send, 0, BUFFER_LENGTH);

							mavlink_msg_request_data_stream_pack(255, 0, &msg_send, msg.sysid, msg.compid, MAV_DATA_STREAM_ALL, 10, 1);
							int len = mavlink_msg_to_send_buffer(buf_send, &msg_send);
							sendto(meuSocket, buf_send, len, 0, (struct sockaddr*)&client, sizeof(struct sockaddr_in));
							first = 0;
						}
					}
					break;

					case MAVLINK_MSG_ID_AHRS3: // ID for GLOBAL_POSITION_INT
					{
						// Get all fields in payload (into global_position)
						mavlink_msg_ahrs3_decode(&msg, &ahrs2);
						printf("AHR3 %f, %f, %f\n", ahrs2.roll, ahrs2.pitch, ahrs2.yaw);

					}
					break;
					case MAVLINK_MSG_ID_ATTITUDE:
					{
						//						if(cont2 == 0)
						//							time2 = carmen_get_time();
						//						cont2++;
						// Get just one field from payload
						printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
						mavlink_msg_attitude_decode(&msg, &attitude_msg);
						printf("Attitude %f, %f, %f\n", attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw);
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
