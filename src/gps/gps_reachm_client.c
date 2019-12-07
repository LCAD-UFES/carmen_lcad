 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>

#include "gps.h"
#include "gps-ipc.h"
#include "gps-io.h"


char tcp_ip_address[MAX_NAME_LENGTH];
char port[MAX_NAME_LENGTH];


void
read_parameters(SerialDevice *dev, int argc, char **argv)
{
	char *device;

	carmen_param_t gps_dev[] = {
		{"gps", "nmea_dev",  CARMEN_PARAM_STRING, &device, 0, NULL},
		{"gps", "nmea_baud", CARMEN_PARAM_INT,    &(dev->baud),    0, NULL}};
  
	carmen_param_install_params(argc, argv, gps_dev, 
			      sizeof(gps_dev) / sizeof(gps_dev[0]));
  
	strncpy(dev->ttyport, device, MAX_NAME_LENGTH);

	free(device);

}


int
stablished_connection_with_server()
{
	struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the linked list of host_info's.
	int pi_socket = 0, status;

	if ((pi_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("--- Socket creation error! ---\n");
		return -1;
	}
	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

	status = getaddrinfo(tcp_ip_address, port, &host_info, &host_info_list);

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
		sleep(5);
		pi_socket = stablished_connection_with_server();
	}
	return (pi_socket);
}


/**************************************************************************
 * MAIN-LOOP
 **************************************************************************/

int
main(int argc, char *argv[])
{
	int gps_nr = 0;
	carmen_gps_gpgga_message gpgga;
	carmen_gps_gprmc_message gprmc;

	if (argc != 3)
	{
		fprintf( stderr, "Syntax: gps_reachm_client server_tcp_ip_address server_port\n");
		exit(1);
	}

	carmen_erase_structure(&gpgga, sizeof(carmen_gps_gpgga_message) );
	carmen_erase_structure(&gprmc, sizeof(carmen_gps_gprmc_message) );
  
	gpgga.host = carmen_get_host();
	gprmc.host = carmen_get_host();

	carmen_ipc_initialize( argc, argv );
	ipc_initialize_messages();
 
	strncpy(tcp_ip_address, argv[1], MAX_NAME_LENGTH);
	strncpy(port, argv[2], MAX_NAME_LENGTH);

	carmen_extern_gpgga_ptr = &gpgga;
	carmen_extern_gpgga_ptr->nr = gps_nr;
	carmen_extern_gprmc_ptr = &gprmc;
	carmen_extern_gprmc_ptr->nr = gps_nr;

	fprintf( stderr, "INFO: ************************\n" );
	fprintf( stderr, "INFO: ********* GPS   ********\n" );
	fprintf( stderr, "INFO: ************************\n" );

	int pi_socket = stablished_connection_with_server();

	int valread;
	while (TRUE)
	{
		// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error
		valread = recv(pi_socket, &gpgga, sizeof(carmen_gps_gpgga_message), MSG_WAITALL);

		if (valread == 0 || valread == -1) // 0 Connection lost due to server shutdown -1 Could not connect
		{
			close(pi_socket);
			pi_socket = trying_to_reconnect();
			continue;
		}
		else if ((valread == -1) || (valread != sizeof(carmen_gps_gpgga_message)))
			continue;

		ipc_publish_position();
	}

	return (0);
}


