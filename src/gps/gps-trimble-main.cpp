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
//#define TCP_IP_ADDRESS			"172.20.10.4"
//#define TCP_IP_PORT				"5017"

//http://bd982.com/ -> site de teste do gps Trimble bd982
//#define TCP_IP_ADDRESS			"155.63.160.20"
//#define TCP_IP_PORT				"5018"
#define INCOME_DATA_BUFFER_SIZE	1000

#define GPS_REACH1 2
#define GPS_REACH2 3


int socketfd;


void
print_usage()
{
	fprintf(stderr,
			"\nUsage:\n"
			" ./gps_nmea_trimble <empty> -> This will try and connect with the Trimble GPS (default)\n"
			" ./gps_nmea_trimble <gps number> <port number> <ip address 2>  <ip address 1> ...  <ip address n>\n"
			"For GPS Trimble -> ./gps_nmea_trimble 1 5018 192.168.0.103\n"
			"For GPS Reach 1 -> ./gps_nmea_trimble 2 5019 172.20.10.2 (when using IPhone)\n"
			"For GPS Reach 2 -> ./gps_nmea_trimble 3 5020 172.20.10.4 (when using IPhone)\n\n");
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
		return (first_nmea_string_end + 1);
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


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
publish_carmen_gps_gphdt_message(int gps_number)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		carmen_extern_gphdt_ptr->nr = gps_number;
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}
}


int
publish_carmen_gps_gpgga_message(int gps_number)
{
//	static double previous_utc_gpgga = -1.0;
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gpgga_ptr != NULL)
	{
//		if (carmen_extern_gpgga_ptr->utc == previous_utc_gpgga)
//			return (0);

		//presention_mode force GPS quality to the viewer3D shows velodyne message in indoor
//		carmen_extern_gpgga_ptr->gps_quality = 4;

//		previous_utc_gpgga = carmen_extern_gpgga_ptr->utc;
		carmen_extern_gpgga_ptr->nr = gps_number;

		err = IPC_publishData (CARMEN_GPS_GPGGA_MESSAGE_NAME, carmen_extern_gpgga_ptr);
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
init_tcp_ip_socket(char *address, char *port)
{
    int status;
    struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
    struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.

    memset(&host_info, 0, sizeof host_info);

    host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

    status = getaddrinfo(address, port, &host_info, &host_info_list);
    if (status != 0)
    {
        std::cout << "getaddrinfo error" << gai_strerror(status) << std::endl;
        return (-1);
    }

    int socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
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

	int gps_number = 1; // This says it is a Trimble GPS
	static carmen_gps_gpgga_message gpgga;
	static carmen_gps_gphdt_message gphdt;
	static carmen_gps_gprmc_message gprmc;

	carmen_erase_structure(&gpgga, sizeof(carmen_gps_gpgga_message));
	carmen_erase_structure(&gphdt, sizeof(carmen_gps_gphdt_message));
	carmen_erase_structure(&gprmc, sizeof(carmen_gps_gprmc_message));
  
	gpgga.host = carmen_get_host();
	gphdt.host = carmen_get_host();
	gprmc.host = carmen_get_host();

	carmen_extern_gpgga_ptr = &gpgga;
	carmen_extern_gpgga_ptr->nr = gps_number;
	carmen_extern_gphdt_ptr = &gphdt;
	carmen_extern_gphdt_ptr->nr = gps_number;
	carmen_extern_gprmc_ptr = &gprmc;
	carmen_extern_gprmc_ptr->nr = gps_number;
}


int
get_socketfd(char *address, char *port)
{
	int socketfd;

	fprintf(stderr, "Trying to open socket to tcp/ip ip and port: %s:%s\n", address, port);
	if ((socketfd = init_tcp_ip_socket(address, port)) < 0)
		fprintf(stderr, "can't open tcp/ip socket on ip and port: %s:%s\n", address, port);
	else
		fprintf(stderr, "socket on %s:%s established!\n", address, port);

	return (socketfd);
}


int
try_to_establish_socket_with_gps(int argc, char **argv)
{
	char *address = (char *) (TCP_IP_ADDRESS);
	char *port = (char *) (TCP_IP_PORT);
	int gps_number = 1;

	if (argc == 1)
	{
		// Default to Trimble, gps_number 1
		socketfd = get_socketfd(address, port);
		if (socketfd == -1)
			exit(1);
	}
	else if (argc > 3)
	{
		gps_number = atoi(argv[1]);
		if ((gps_number != GPS_REACH1) && (gps_number != GPS_REACH2))
		{
			printf("Wrong gps_number parameter. See usage above.\n");
			exit(1);
		}

		bool found = false;
		for (int i = 3; i < argc; i++)
		{
			socketfd = get_socketfd(argv[i], argv[2]);
			if (socketfd != -1)
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			printf("Could not establish socket...\n");
			exit(1);
		}
	}
	else
	{
		printf("Wrong number of parameters. See usage above.\n");
		exit(1);
	}

	return (gps_number);
}

//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	ipc_initialize_messages();

	fprintf(stderr, "INFO: ************************\n");
	fprintf(stderr, "INFO: ********* GPS   ********\n");
	fprintf(stderr, "INFO: ************************\n");

	print_usage();

	int gps_number = try_to_establish_socket_with_gps(argc, argv);

	while (TRUE)
	{
		char incomming_data_buffer[INCOME_DATA_BUFFER_SIZE];
		char *first_nmea_string_end;

		get_data_from_socket(incomming_data_buffer, socketfd);

//		printf("%s\n", incomming_data_buffer);
		carmen_extern_gpgga_ptr->timestamp = carmen_get_time();
		carmen_extern_gphdt_ptr->timestamp = carmen_extern_gpgga_ptr->timestamp;

		if ((first_nmea_string_end = parse_gga_info(incomming_data_buffer)) != NULL)
			publish_carmen_gps_gpgga_message(gps_number);

		if (first_nmea_string_end)
		{
			// strcpy(first_nmea_string_end, "$GPHDT,333.694,T*3D");
			if (parse_hdt_info(first_nmea_string_end))
			{
				if (gps_number == 1) // Only the Trimble GPS knows a suitable orientation at the moment in this point.
					publish_carmen_gps_gphdt_message(gps_number);
			}
		}
	}

	return (0);
}
