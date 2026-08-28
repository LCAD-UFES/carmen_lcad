/*
 * canfdtest.c - Full-duplex test program (DUT and host part)
 *
 * (C) 2009 by Vladislav Gribov, IXXAT Automation GmbH, <gribov@ixxat.de>
 * (C) 2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include <getopt.h>
#include <time.h>
#include <sched.h>
#include <limits.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_MSG_ID	0x77
#define CAN_MSG_LEN	8
#define CAN_MSG_COUNT	50
#define CAN_MSG_WAIT	27

void print_frame(struct can_frame *frame)
{
	int i;

	printf("%04x: ", frame->can_id);
	if (frame->can_id & CAN_RTR_FLAG) {
		printf("remote request");
	} else {
		printf("[%d]", frame->can_dlc);
		for (i = 0; i < frame->can_dlc; i++)
			printf(" %02x", frame->data[i]);
	}
	printf("\n");
}

int recv_frame(int sockfd, struct can_frame *frame)
{
	int ret;

	ret = recv(sockfd, frame, sizeof(*frame), 0);
	if (ret != sizeof(*frame)) {
		if (ret < 0)
			perror("recv failed");
		else
			fprintf(stderr, "recv returned %d", ret);
		return 0;
	}
	return 1;
}

int send_frame(int sockfd, struct can_frame *frame)
{
	int ret;

	while ((ret = send(sockfd, frame, sizeof(*frame), 0))
	       != sizeof(*frame)) {
		if (ret < 0) {
			if (errno != ENOBUFS) {
				perror("send failed");
				return 0;
			}
		} else {
			fprintf(stderr, "send returned %d", ret);
			return 0;
		}
	}
	return 1;
}

int init_can(char *intf_name)
{
	struct ifreq ifr;
	struct sockaddr_can addr;
	int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;

//	printf("interface = %s, family = %d, type = %d, proto = %d\n",
//	       intf_name, family, type, proto);

	int sockfd;
	if ((sockfd = socket(family, type, proto)) < 0) {
		perror(intf_name);
		return -1;
	}

	addr.can_family = family;
	strcpy(ifr.ifr_name, intf_name);
	ioctl(sockfd, SIOCGIFINDEX, &ifr);
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror(intf_name);
		close(sockfd);
		return -1;
	}

	return (sockfd);
}
