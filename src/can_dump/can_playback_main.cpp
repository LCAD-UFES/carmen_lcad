#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/can_dump_interface.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <can_utils.h>

int desired_can;
int out_can_sockfd = -1;


void print_test_frame(struct can_frame *frame)
{
	int i;

	printf("              ");
	printf("%04X   ", frame->can_id & (~CAN_RTR_FLAG));
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
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
can_dump_publish_can_message(carmen_can_dump_can_line_message *message)
{
	struct can_frame frame;

	int can_in, can_out;
	char buffer[1024];
	int can_dlc;

	buffer[0] = '\0';
	sscanf(message->can_line, "can%d -> vcan%d  %X   [%d] %[^\n]",
			&can_in, &can_out, &(frame.can_id), &can_dlc, buffer);
	frame.can_dlc = can_dlc;

	if ((desired_can != 2) && (can_in != desired_can))
		return;

	if (strncmp(buffer, "remote", 6) == 0)
	{
		frame.can_id |= CAN_RTR_FLAG;
	}
	else
	{
		for (int i = 0; i < frame.can_dlc; i++)
		{
			int can_byte;
			sscanf(buffer + i * 3, "%02X", &can_byte);
			frame.data[i] = can_byte;
		}
	}
//	printf("%s\n", message->can_line);
//	print_test_frame(&frame);
	send_frame(out_can_sockfd, &frame);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
can_dump_message_handler(carmen_can_dump_can_line_message *message)
{
	can_dump_publish_can_message(message);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("can_view: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_can_dump_define_can_line_message();

	if (argc < 3)
	{
		printf("Error\n Usage: can_playback <input can interface> <output can interface>\n");
		exit(1);
	}

	desired_can = atoi(argv[1]);

	out_can_sockfd = init_can(argv[2]);
	if (out_can_sockfd == -1)
	{
		printf("Error: Could not open can%s\n", argv[2]);
		exit(1);
	}

	carmen_can_dump_subscribe_can_line_message(NULL, (carmen_handler_t) can_dump_message_handler, CARMEN_SUBSCRIBE_ALL);
	signal(SIGINT, shutdown_module);

	carmen_ipc_dispatch();

	return (0);
}
