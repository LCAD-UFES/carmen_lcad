#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>

#include <jaus.h>
#include <openJaus.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>

#include <pthread.h>

#include "can_utils.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

static int mainRunning = FALSE;

int out_can_sockfd;


static void signal_handler(int signo)
{
	close(out_can_sockfd);

	exit(0);
}

void send_can_velocity_stimulus(int can_sockfd, double v)
{
#define METERS_BY_ODOMETRY_COUNT	0.0255
#define HISTORY_SIZE	30

	static double t = 0.0;
	static int current_odometry = 0;
	static double meters_advanced = 0.0;
	static double odometry_v[HISTORY_SIZE];
	static int odometry_index = 0;

	if (t == 0.0)
	{
		t = ojGetTimeSec();
		return;
	}

	double delta_t = ojGetTimeSec() - t;
	meters_advanced += v * delta_t;

	int new_odometry = meters_advanced / METERS_BY_ODOMETRY_COUNT;
	int delta_odometry = new_odometry - current_odometry;
	current_odometry = new_odometry;

	odometry_v[odometry_index] = ((double) delta_odometry * METERS_BY_ODOMETRY_COUNT) / delta_t;
	odometry_index++;
	if (odometry_index >= HISTORY_SIZE)
		odometry_index = 0;

	double avg_odometry_v = 0.0;
	int i;
	for (i = 0; i < HISTORY_SIZE; i++)
		avg_odometry_v += odometry_v[i];
	avg_odometry_v /= (double) HISTORY_SIZE;

	printf("%lf %lf\n", v, ojGetTimeSec());

	struct can_frame frame;
	frame.can_id = 0x216;
	frame.can_dlc = 8;

	frame.data[0] = current_odometry & 0xFF;
	frame.data[1] = current_odometry & 0xFF;
	frame.data[2] = current_odometry & 0xFF;
	frame.data[3] = current_odometry & 0xFF;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	if (can_sockfd != -1)
		send_frame(can_sockfd, &frame);

	t = ojGetTimeSec();
}

int main(int argCount, char **argString)
{
	signal(SIGTERM, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);

	if (argCount != 2)
	{
		printf("Error. Wrong number of parameter.\n Usage %s <can interface>\n", argString[0]);
		exit(1);
	}
	out_can_sockfd = init_can(argString[1]);
	if (out_can_sockfd == -1)
	{
		printf("Error opening out_can_sockfd (can device = %s)\n", argString[1]);
		exit(1);
	}

	mainRunning = TRUE;
	double a = 0.6;
	double v = 0.0;
	double t = ojGetTimeSec();
	while (mainRunning)
	{
		double delta_t = ojGetTimeSec() - t;
		if (v > 15.0)
		{
			a = -0.6;
			v = 15.0;
		}
		else if (v < 0.0)
		{
			a = 0.6;
			v = 0.0;
		}

		v = v + a * delta_t;
		send_can_velocity_stimulus(out_can_sockfd, v);

		t = ojGetTimeSec();

		ojSleepMsec(16.6667);
	}

	close(out_can_sockfd);

	return (0);
}
