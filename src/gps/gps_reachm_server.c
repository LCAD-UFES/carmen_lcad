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


int server_fd;


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
connect_with_client(int port)
{
    int new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("--- Socket Failed ---\n");
        return (-1);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("--- Setsockopt Failed ---\n");
        return (-1);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Forcefully attaching socket to the port defined
    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        perror("--- Bind Failed ---\n");
        return (-1);
    }

    if (listen(server_fd, 3) < 0)
     {
         perror("-- Listen Failed ---\n");
         return (-1);
     }

    printf("--- Waiting for connection! --\n server_fd = %d \n", server_fd);
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("--- Accept Failed ---\n");
        return (-1);
    }
    printf("--- Connection established successfully! ---\n");

    return (new_socket);
}


/**************************************************************************
 * MAIN-LOOP
 **************************************************************************/


int
main(int argc, char *argv[])
{
	int gps_nr = 0;
	SerialDevice dev;
	carmen_gps_gpgga_message gpgga;
	carmen_gps_gprmc_message gprmc;

	if (argc != 5)
	{
		fprintf( stderr, "Syntax: gps_reachm /dev/ttyXXX baud_rate gps_nr socket_port\n");
		exit(1);
	}

	carmen_erase_structure(&gpgga, sizeof(carmen_gps_gpgga_message) );
	carmen_erase_structure(&gprmc, sizeof(carmen_gps_gprmc_message) );
  
	gpgga.host = carmen_get_host();
	gprmc.host = carmen_get_host();

	DEVICE_init_params( &dev );

	strncpy(dev.ttyport, argv[1], MAX_NAME_LENGTH);
	dev.baud = atoi(argv[2]);
	gps_nr = atoi(argv[3]);
	int port = atoi(argv[4]);

	printf("dev.ttyport = %s dev.baud = %d gps_nr = %d port = %d\n", dev.ttyport,dev.baud, gps_nr, port);

	carmen_extern_gpgga_ptr = &gpgga;
	carmen_extern_gpgga_ptr->nr = gps_nr;
	carmen_extern_gprmc_ptr = &gprmc;
	carmen_extern_gprmc_ptr->nr = gps_nr;

	fprintf( stderr, "INFO: ************************\n" );
	fprintf( stderr, "INFO: ********* GPS   ********\n" );
	fprintf( stderr, "INFO: ************************\n" );

	fprintf( stderr, "INFO: open device: %s\n", dev.ttyport);
	if (DEVICE_connect_port(&dev) < 0)
	{
		fprintf(stderr, "ERROR: can't open device !!!\n\n");
		exit(1);
	}
	else
	{
		fprintf(stderr, "INFO: done\n");
	}

	int pi_socket = connect_with_client(port);

	while (TRUE)
	{
		if (DEVICE_read_data(dev))
		{
			gpgga.timestamp = carmen_get_time();
			gprmc.timestamp = gpgga.timestamp;

			int result = send(pi_socket, &gpgga, sizeof(carmen_gps_gpgga_message), MSG_NOSIGNAL);  // Returns number of bytes read, 0 in case of connection lost, -1 in case of error

			if (result == -1)
			{
				printf("--- Disconnected ---\n");
	            close(server_fd);
	            sleep(3);

	            pi_socket = connect_with_client(port);
	        }
		}
	}

	return (0);
}


