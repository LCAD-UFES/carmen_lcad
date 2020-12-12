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

static pthread_mutex_t print_frame_lock = PTHREAD_MUTEX_INITIALIZER;


static void print_frame_in_out(struct can_frame *frame, char *can_a, char *can_b)
{
	int i;

	pthread_mutex_lock(&print_frame_lock);

	printf("%s -> %s  %04X   ", can_a, can_b, frame->can_id & (~CAN_RTR_FLAG));
	printf("[%d] ", frame->can_dlc);

	if (frame->can_id & CAN_RTR_FLAG)
		printf("remote request");
	else
	{
		for (i = 0; i < frame->can_dlc; i++)
			printf(" %02X", frame->data[i]);
	}

	printf("\n");
	fflush(stdout);

	pthread_mutex_unlock(&print_frame_lock);
}

//static void ojSleepMsec_(int msec)
//{
//	usleep(msec * 1000);
//}


void *can_in_read_thread_func(void *unused)
{
	struct can_frame frame;

	while (1)
	{
		recv_frame(in_can_sockfd, &frame);
		print_frame_in_out(&frame, can_in, can_out);
//		ojSleepMsec_(10);
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
		send_frame(in_can_sockfd, &frame);
	}

	return (NULL);
}

void move_up_down_left_right(int move_up_down, int move_left_right, int out_can_sockfd)
{
	struct can_frame frame;
	frame.can_dlc = 8;
	for (int i = 0; i < frame.can_dlc; i++)
		frame.data[i] = 0;
	frame.data[0] = move_up_down;
	frame.data[1] = move_left_right;
	if (out_can_sockfd != -1)
	{
		frame.can_id = 0x281;
		send_frame(out_can_sockfd, &frame);
		frame.can_id = 0x2B1;
		send_frame(out_can_sockfd, &frame);
	}
}

void send_can_package(int out_can_sockfd, int can_id,
		int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7)
{
	struct can_frame frame;
	frame.can_dlc = 8;
	frame.data[0] = data0;
	frame.data[1] = data1;
	frame.data[2] = data2;
	frame.data[3] = data3;
	frame.data[4] = data4;
	frame.data[5] = data5;
	frame.data[6] = data6;
	frame.data[7] = data7;

	if (out_can_sockfd != -1)
	{
		frame.can_id = can_id;
		send_frame(out_can_sockfd, &frame);
	}
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

//	int move_up_down = 0;
//	int move_left_right = 0;
//	int horn = 2;

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

//		int ch = getchar();
//		printf("Peguei o char %c\n", ch);
//		switch (ch)
//		{
//		case 'i':
//			move_up_down += 30;
//			if (move_up_down > 255)
//				move_up_down = 255;
//			move_up_down_left_right(move_up_down, move_left_right, out_can_sockfd);
//			break;
//		case 'm':
//			move_up_down -= 30;
//			if (move_up_down < 0)
//				move_up_down = 0;
//			move_up_down_left_right(move_up_down, move_left_right, out_can_sockfd);
//			break;
//
//		case 'j':
//			move_left_right += 30;
//			if (move_left_right > 255)
//				move_left_right = 255;
//			move_up_down_left_right(move_up_down, move_left_right, out_can_sockfd);
//			break;
//		case 'k':
//			move_left_right -= 30;
//			if (move_left_right < 0)
//				move_left_right = 0;
//			move_up_down_left_right(move_up_down, move_left_right, out_can_sockfd);
//			break;
//
//		case ' ':
//			move_up_down = 0;
//			move_left_right = 0;
//			move_up_down_left_right(move_up_down, move_left_right, out_can_sockfd);
//			break;
//
//		case 'b':
//			horn = (horn == 3)? 2: 3;
//			send_can_package(out_can_sockfd, 0x1B1, 0x00, 0x00, 0x00, 0x00, horn, 0x00, 0x00, 0x00);
//			ojSleepMsec(50);
//			send_can_package(out_can_sockfd, 0x508, 0x00, 0x23, 0x00, 0x24, 0x00, 0x1A, 0x00, 0x1A);
//			ojSleepMsec(50);
//			send_can_package(out_can_sockfd, 0x508, 0x00, 0x24, 0x00, 0x24, 0x00, 0x1A, 0x00, 0x1A);
//			ojSleepMsec(50);
//			send_can_package(out_can_sockfd, 0x191, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
//			ojSleepMsec(50);
//			send_can_package(out_can_sockfd, 0x191, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
//			break;
//
//		case 'a': // para frente
//			send_can_package(out_can_sockfd, 0x181, 0xE8, 0x03, 0x00, 0x40, 0x03, 0x00, 0x00, 0x00);
//			break;
//
//		case 's': // neutro
//			send_can_package(out_can_sockfd, 0x181, 0xE8, 0x03, 0x00, 0x44, 0x03, 0x00, 0x00, 0x00);
//			break;
//
//		}
//		printf("move_up_down %d, move_left_right %d\n", move_up_down, move_left_right);
	}

	close(in_can_sockfd);
	close(out_can_sockfd);

	return (0);
}
