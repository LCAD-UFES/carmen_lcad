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
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <iostream>
#include <cstring>      // Needed for memset
#include <sys/socket.h> // Needed for the socket functions
#include <netdb.h>      // Needed for the socket functions

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>

#include "gps.h"
#include "gps-ipc.h"
#include "gps-io.h"

#define TCP_IP_ADDRESS			"192.168.0.103"
#define TCP_IP_PORT				"5017"
//http://bd982.com/ -> site de teste do gps Trimble bd982
//#define TCP_IP_ADDRESS			"155.63.160.20"
//#define TCP_IP_PORT				"5018"
#define INCOME_DATA_BUFFER_SIZE	1000

int socketfd; // The socket descriptor


void
print_usage( void )
{
	fprintf( stderr, "Syntax: gps_nmea_trimble\n");
}


int
get_data_from_socket(char *incomming_data_buffer, int socketfd)
{
    ssize_t bytes_received;

    bytes_received = recv(socketfd, incomming_data_buffer, INCOME_DATA_BUFFER_SIZE - 1, 0);
    if (bytes_received == 0)
        std::cout << "host shut down?" << std::endl;
    if (bytes_received == -1)
        std::cout << "socket recv() error!" << std::endl;

    incomming_data_buffer[bytes_received] = '\0';

    return(bytes_received);
}


char *
parse_gga_info(char *incomming_data_buffer)
{
	// Find the end of the first NMEA string
	char *first_nmea_string_end = strchr(incomming_data_buffer, '*');
	if (first_nmea_string_end == NULL)
		return (NULL);
	first_nmea_string_end[0] = '\0';
	if (carmen_gps_parse_data(incomming_data_buffer, strlen(incomming_data_buffer)))
		return (first_nmea_string_end);
	else
		return (NULL);
}


int
parse_hdt_info(char *first_nmea_string_end)
{
	if (first_nmea_string_end == NULL)
		return (0);

	// Find the end of the first NMEA string
	char *second_nmea_string_begining = strchr(first_nmea_string_end, '$');
	if (second_nmea_string_begining == NULL)
		return (0);
	char *second_nmea_string_end = strchr(second_nmea_string_begining, '*');
	if (first_nmea_string_end == NULL)
		return (0);
	second_nmea_string_end[0] = '\0';

	return (carmen_gps_parse_data(second_nmea_string_begining, strlen(second_nmea_string_begining)));
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
publish_carmen_gps_gphdt_message()
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}
}


int
publish_carmen_gps_gpgga_message()
{
	static double previous_utc_gpgga = -1.0;
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gpgga_ptr != NULL)
	{
		if (carmen_extern_gpgga_ptr->utc == previous_utc_gpgga)
			return (0);
		previous_utc_gpgga = carmen_extern_gpgga_ptr->utc;

		err = IPC_publishData (CARMEN_GPS_GPGGA_MESSAGE_NAME,
					   carmen_extern_gpgga_ptr );
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPGGA_MESSAGE_NAME);
		// fprintf( stderr, "(gga)" );
		// printf("gga lt:% .20lf lg:% .20lf\t", carmen_extern_gpgga_ptr->latitude, carmen_extern_gpgga_ptr->longitude);

		return (1);
	}
	else
		return (0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int a __attribute__ ((unused)))
{
	close(socketfd);

	printf("Disconnected from IPC.\n");

	exit(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


int
init_tcp_ip_socket()
{
    int status;
    struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
    struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.

    memset(&host_info, 0, sizeof host_info);

    host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

    status = getaddrinfo(TCP_IP_ADDRESS, TCP_IP_PORT, &host_info, &host_info_list);
    if (status != 0)
    {
        std::cout << "getaddrinfo error" << gai_strerror(status) << std::endl;
        return (-1);
    }

    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    if (socketfd == -1)
    {
        std::cout << "socket error" << std::endl;
        return (-1);
    }

    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)
    {
        std::cout << "connect error" << std::endl;
        return (-1);
    }
    freeaddrinfo(host_info_list);

    return (socketfd);
}


void
ipc_initialize_messages( void )
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPHDT_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPHDT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPHDT_MESSAGE_NAME);

	int gps_nr = 1; // This says it is a Trimble GPS
	static carmen_gps_gpgga_message gpgga;
	static carmen_gps_gphdt_message gphdt;

	carmen_erase_structure(&gpgga, sizeof(carmen_gps_gpgga_message));
	carmen_erase_structure(&gphdt, sizeof(carmen_gps_gphdt_message));
  
	gpgga.host = carmen_get_host();
	gphdt.host = carmen_get_host();

	carmen_extern_gpgga_ptr = &gpgga;
	carmen_extern_gpgga_ptr->nr = gps_nr;
	carmen_extern_gphdt_ptr = &gphdt;
	carmen_extern_gphdt_ptr->nr = gps_nr;
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	ipc_initialize_messages();
 
	fprintf( stderr, "INFO: ************************\n" );
	fprintf( stderr, "INFO: ********* GPS   ********\n" );
	fprintf( stderr, "INFO: ************************\n" );

	fprintf( stderr, "INFO: open socket to tcp/ip ip and port: %s:%s\n", TCP_IP_ADDRESS, TCP_IP_PORT);
	if ((socketfd = init_tcp_ip_socket()) < 0)
	{
		fprintf(stderr, "ERROR: can't open tcp ip socket !!!\n\n");
		exit(1);
	}
	else
	{
		fprintf(stderr, "INFO: done\n");
	}

	while (TRUE)
	{
		char incomming_data_buffer[INCOME_DATA_BUFFER_SIZE];
		char *first_nmea_string_end;

		get_data_from_socket(incomming_data_buffer, socketfd);

		carmen_extern_gpgga_ptr->timestamp = carmen_get_time();
		carmen_extern_gphdt_ptr->timestamp = carmen_extern_gpgga_ptr->timestamp;

		if ((first_nmea_string_end = parse_gga_info(incomming_data_buffer)) != NULL)
			publish_carmen_gps_gpgga_message();

		if (first_nmea_string_end)
		{
			// strcpy(first_nmea_string_end, "$GPHDT,333.694,T*3D");
			if (parse_hdt_info(first_nmea_string_end))
				publish_carmen_gps_gphdt_message();
		}
	}

	return (0);
}
