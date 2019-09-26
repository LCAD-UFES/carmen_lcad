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
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
//#include <wait.h>
#include <ctype.h>

#include "orc.h"

typedef struct 
{
	char *device;
} rawprovider_t;

//cfmakeraw is not POSIX, so we provide our own.
static void my_cfmakeraw(struct termios *p)
{
	p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
			|INLCR|IGNCR|ICRNL|IXON);
	p->c_oflag &= ~OPOST;
	p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	p->c_cflag &= ~(CSIZE|PARENB);
	p->c_cflag |= CS8;	
}

static int raw_connect(orc_comms_impl_t *impl)
{
	rawprovider_t *t = (rawprovider_t*) impl->_p;

	printf("LIBORC: Opening port (%s)", t->device);
	int fd = open(t->device, O_RDWR | O_NOCTTY, 0);
	if (fd == -1)	
		return -1;

	struct termios opts;

	if (tcgetattr(fd, &opts)) {
		printf("LIBORC: Couldn't read serial device attributes (%s): %s", t->device, strerror(errno));
		return 0;
	}

	my_cfmakeraw(&opts);
	
	tcflush(fd, TCIFLUSH);

	if (tcsetattr(fd, TCSANOW, &opts)) {
		printf("LIBORC: Couldn't set serial device attributes (%s): %s", t->device, strerror(errno));
		return 0;
	}

	return fd;
}

static void raw_disconnect(orc_comms_impl_t *impl __attribute__ ((unused)), int fd)
{
	close(fd);
	return;
}

orc_comms_impl_t *orc_rawprovider_create(char *device)
{
	orc_comms_impl_t *impl = (orc_comms_impl_t*) calloc(sizeof(orc_comms_impl_t), 1);
	rawprovider_t *t = (rawprovider_t*) calloc(sizeof(rawprovider_t), 1);
	
	t->device = strdup(device);

	impl->connect = raw_connect;
	impl->disconnect = raw_disconnect;
	impl->_p = t;

	return impl;
}


