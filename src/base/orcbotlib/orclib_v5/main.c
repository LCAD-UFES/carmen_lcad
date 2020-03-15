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
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>

#include "orc.h"

void *runthread(void *arg);

pthread_mutex_t mutex;

volatile int cnt = 0; // used to allocate thread #s
volatile long tcnt = 0; // used to count transactions

int main() //int argc, char *argv[])
{
	orc_comms_impl_t *impl = orc_rawprovider_create("/dev/ttyUSB0");
  //	orc_comms_impl_t *impl = orc_tcpprovider_create("localhost", 7000);

	orc_t *orc = orc_create(impl);

	pthread_attr_t threadAttr;
	pthread_attr_init(&threadAttr);
	pthread_attr_setstacksize(&threadAttr, 32768);
	pthread_t newthread;

	pthread_mutex_init(&mutex, NULL);

	orc_lcd_clear(orc);
	orc_lcd_console_home(orc);
	orc_lcd_console_write(orc, "Hello, world.");

	orc_lcd_draw_string(orc, 5,16,'B',"Ahoy %i %f",2,3.14);


	for (int i =0; i<32;i++)
	{
		if (pthread_create(&newthread, &threadAttr, runthread, orc)) {
			printf("Couldn't create orc reader thread: %s", strerror(errno));
			exit(EXIT_FAILURE);
		}
	}	


	while (1)
	{
		sleep(10);
		printf(" TOT TPS: %5.3f\n", tcnt/10.0);
		tcnt = 0;
	}
}

void *runthread(void *arg)
{
	orc_t *orc = (orc_t*) arg;

	struct timeval starttime, endtime;

	int iter = 1000;

	int myid;

	pthread_mutex_lock(&mutex);
	myid = cnt++;
	pthread_mutex_unlock(&mutex);

	while (1)
	{
		gettimeofday(&starttime, NULL);

		for (int i = 0; i < iter; i++)
		{
			int v = orc_quadphase_read(orc, 0);
			v = v; // silence compiler warning
			pthread_mutex_lock(&mutex);
			tcnt++;
			pthread_mutex_unlock(&mutex);
		}

		gettimeofday(&endtime, NULL);

		double elapsedTime = (endtime.tv_sec-starttime.tv_sec) +
			(endtime.tv_usec - starttime.tv_usec)/1000000.0;

		printf("%4i TPS: %f\n", myid, iter/elapsedTime);
	}

	return NULL;
}
