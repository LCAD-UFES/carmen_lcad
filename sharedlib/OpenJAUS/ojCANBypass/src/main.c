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

int in_can_sockfd;
int out_can_sockfd;

char *can_in;
char *can_out;


static void signal_handler(int signo)
{
	close(in_can_sockfd);
	close(out_can_sockfd);

	exit(0);
}

void forward_frame_but_filter_speed(int can_sockfd, struct can_frame frame)
{
//	if (frame.can_id == 0x425) // Velocidade
//	{
//		frame.data[0] = 0x00;
//		frame.data[1] = 0x00;
//		frame.data[2] = 0x23;
//		frame.data[3] = 0x28;
//		frame.data[4] = 0x02;
//		frame.data[5] = 0x58;
//		frame.data[6] = 0x28;
//		frame.data[7] = 0xa8;
//	}
	send_frame(can_sockfd, &frame);
}

static void print_frame_in_out(struct can_frame *frame, char *can_a, char *can_b)
{
	int i;

	printf("%s -> %s  %04x   ", can_a, can_b, frame->can_id);
	if (frame->can_id & CAN_RTR_FLAG)
		printf("remote request");
	else
	{
		printf("[%d] ", frame->can_dlc);
		for (i = 0; i < frame->can_dlc; i++)
			printf(" %02X", frame->data[i]);
	}

	printf("\n");
	fflush(stdout);
}

static void ojSleepMsec_(int msec)
{
	usleep(msec * 1000);
}


void *can_in_read_thread_func(void *unused)
{
	struct can_frame frame;

	while (1)
	{
		recv_frame(in_can_sockfd, &frame);
		print_frame_in_out(&frame, can_in, can_out);
		ojSleepMsec_(10);
		forward_frame_but_filter_speed(out_can_sockfd, frame);
	}

	return (NULL);
}

void *can_out_read_thread_func(void *unused)
{
	struct can_frame frame;

	while (1)
	{
		recv_frame(out_can_sockfd, &frame);
		print_frame_in_out(&frame, can_out, can_in);
//		ojSleepMsec_(40);
//		send_frame(in_can_sockfd, &frame);
	}

	return (NULL);
}

int main(int argCount, char **argString)
{
	pthread_t can_in_read_thread, can_out_read_thread;

	signal(SIGTERM, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);

	can_in = argString[1];
	in_can_sockfd = init_can(argString[1]);
	if (in_can_sockfd == -1)
	{
		printf("Error opening in_can_sockfd (can device = %s)\n", argString[1]);
		exit(1);
	}

	can_out = argString[2];
	out_can_sockfd = init_can(argString[2]);
	if (out_can_sockfd == -1)
	{
		printf("Error opening out_can_sockfd (can device = %s)\n", argString[2]);
		exit(1);
	}

	mainRunning = TRUE;
	while (mainRunning)
	{
		static int first_time = true;
		if (first_time)
		{
			pthread_create(&can_in_read_thread, NULL, can_in_read_thread_func, NULL);
			pthread_create(&can_out_read_thread, NULL, can_out_read_thread_func, NULL);
			first_time = false;
		}
		ojSleepMsec(100);
	}

	close(in_can_sockfd);
	close(out_can_sockfd);

	return (0);
}
