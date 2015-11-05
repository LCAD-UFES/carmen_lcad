/*
 * data_collector_main.cpp
 *
 *  Created on: 03/07/2013
 *      Author: romulo
 */

#include <carmen/carmen.h>

static bool ready_to_write = false;
static char log_folder_path[256] = "";
static int file_id = 0;

FILE *localize_file = NULL, *odometry_file = NULL, *command_file = NULL;

void get_file_name(const char *file_path, const char *prefix, int id, char *full_file_path)
{
	sprintf(full_file_path, "%s/%s_log%d.data", file_path, prefix, id);
}


void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (!ready_to_write)
		return;

	fprintf(localize_file, "%f %f %f %f %f %f\n",
			msg->globalpos.x, msg->globalpos.y, carmen_radians_to_degrees(msg->globalpos.theta),
			msg->v, msg->phi, msg->timestamp);
}

void
odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	if (!ready_to_write)
		return;

	fprintf(odometry_file, "%f %f %f\n",
			msg->v, msg->phi, msg->timestamp);
}

void
command_handler(carmen_base_ackerman_motion_command_message *msg)
{
	if (!ready_to_write || msg->num_motion_commands <= 0)
		return;

	fprintf(command_file, "%f %f %f\n",
			msg->motion_command->v, msg->motion_command->phi, msg->timestamp);
}


void
go_handler()
{
	char aux[1000];

	if (ready_to_write)
		return;

	get_file_name(log_folder_path, "localize", file_id, aux);
	localize_file = fopen(aux, "w");

	get_file_name(log_folder_path, "odometry", file_id, aux);
	odometry_file = fopen(aux, "w");

	get_file_name(log_folder_path, "command", file_id, aux);
	command_file = fopen(aux, "w");

	printf("Reading log %d\n", file_id);

	file_id++;

	ready_to_write = true;
}

void
stop_handler()
{

	if (localize_file)
		fclose(localize_file);
	localize_file = NULL;

	if (odometry_file)
		fclose(odometry_file);
	odometry_file = NULL;

	if (command_file)
		fclose(command_file);
	command_file = NULL;

	ready_to_write = false;

	printf("Log Stopped\nWaiting go command..\n");
}

static void
signal_handler(int sig)
{
	stop_handler();
	printf("Signal %d received, exiting program ...\n", sig);
	exit(1);
}

static void
register_handlers()
{
	signal(SIGINT, signal_handler);

	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL,
			(carmen_handler_t) localize_globalpos_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(
			NULL,
			(carmen_handler_t) odometry_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t)go_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t)stop_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME,
			(char *)CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT,
			NULL, sizeof(carmen_base_ackerman_motion_command_message),
			(carmen_handler_t)command_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if (argc <= 1)
	{
		fprintf(stderr, "[log_folder_path]\n");
		return 0;
	}

	strcpy(log_folder_path, argv[1]);


	register_handlers();

	printf("Waiting go command..\n");

	carmen_ipc_dispatch();
}



