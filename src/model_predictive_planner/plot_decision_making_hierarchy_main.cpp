#include <carmen/carmen.h>

#include "util.h"
#include "publisher_util.h"


// Global variables
carmen_robot_ackerman_motion_command_message *robot_motion_command_message;
carmen_base_ackerman_motion_command_message *base_motion_command_message;
rrt_path_message *mpp_message;


void
plot_phi_in_trajectories()
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:3]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-0.55:0.55]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 't'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'phi'\n");
//		fprintf(gnuplot_pipeMP, "set tics out\n");
		fprintf(gnuplot_pipeMP, "set size ratio -1\n");
	}

	FILE *mpp_file = fopen("mpp.txt", "w");
	FILE *base_motion_file = fopen("base_motion.txt", "w");
	FILE *robot_motion_file = fopen("robot_motion.txt", "w");

	double t;
	t = 0.0;
	for (int i = 0; (i < base_motion_command_message->num_motion_commands) && (t < 3.0); i++)
	{
		fprintf(base_motion_file, "%lf %lf\n", base_motion_command_message->motion_command[i].phi, t);
		t += base_motion_command_message->motion_command[i].time;
		fprintf(base_motion_file, "%lf %lf\n", base_motion_command_message->motion_command[i].phi, t);
	}

	t = 0.0;
	for (int i = 0; (i < robot_motion_command_message->num_motion_commands) && (t < 3.0); i++)
	{
		fprintf(robot_motion_file, "%lf %lf\n", robot_motion_command_message->motion_command[i].phi, t);
		t += base_motion_command_message->motion_command[i].time;
		fprintf(robot_motion_file, "%lf %lf\n", robot_motion_command_message->motion_command[i].phi, t);
	}

	t = 0.0;
	for (int i = 0; (i < mpp_message->size) && (t < 3.0); i++)
	{
		fprintf(mpp_file, "%lf %lf\n", mpp_message->path[i].p1.phi, t);
		t += mpp_message->path[i].time;
		fprintf(mpp_file, "%lf %lf\n", mpp_message->path[i].p1.phi, t);
	}

	fclose(mpp_file);
	fclose(base_motion_file);
	fclose(robot_motion_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./mpp.txt' using 2:1 w l title 'mpp' lt rgb 'blue',"
			"'./base_motion.txt' using 2:1 w l title 'base' lt rgb 'green',"
			"'./robot_motion.txt' using 2:1 with linespoints title 'robot' lt rgb 'red'\n");

	fflush(gnuplot_pipeMP);
}


void
plot_v_in_trajectories()
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:3]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-10:10]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 't'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'v'\n");
//		fprintf(gnuplot_pipeMP, "set tics out\n");
//		fprintf(gnuplot_pipeMP, "set size ratio -1\n");
	}

	FILE *mpp_file = fopen("mpp_v.txt", "w");
	FILE *base_motion_file = fopen("base_motion_v.txt", "w");
	FILE *robot_motion_file = fopen("robot_motion_v.txt", "w");

	double t;
	t = 0.0;
	for (int i = 0; (i < base_motion_command_message->num_motion_commands) && (t < 3.0); i++)
	{
		fprintf(base_motion_file, "%lf %lf\n", base_motion_command_message->motion_command[i].v, t);
		t += base_motion_command_message->motion_command[i].time;
		fprintf(base_motion_file, "%lf %lf\n", base_motion_command_message->motion_command[i].v, t);
	}

	t = 0.0;
	for (int i = 0; (i < robot_motion_command_message->num_motion_commands) && (t < 3.0); i++)
	{
		fprintf(robot_motion_file, "%lf %lf\n", robot_motion_command_message->motion_command[i].v, t);
		t += base_motion_command_message->motion_command[i].time;
		fprintf(robot_motion_file, "%lf %lf\n", robot_motion_command_message->motion_command[i].v, t);
	}

	t = 0.0;
	for (int i = 0; (i < mpp_message->size) && (t < 3.0); i++)
	{
		fprintf(mpp_file, "%lf %lf\n", mpp_message->path[i].p1.v, t);
		t += mpp_message->path[i].time;
		fprintf(mpp_file, "%lf %lf\n", mpp_message->path[i].p1.v, t);
	}

	fclose(mpp_file);
	fclose(base_motion_file);
	fclose(robot_motion_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./mpp_v.txt' using 2:1 w l title 'mpp' lt rgb 'blue',"
			"'./base_motion_v.txt' using 2:1 w l title 'base' lt rgb 'green',"
			"'./robot_motion_v.txt' using 2:1 with linespoints title 'robot' lt rgb 'red'\n");

	fflush(gnuplot_pipeMP);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


// Do MPP
void
rrt_path_message_handler(rrt_path_message *msg)
{
	mpp_message = msg;
}


// Do path_follower
void
robot_ackerman_motion_command_message_handler(carmen_robot_ackerman_motion_command_message *motion_command_message)
{
	robot_motion_command_message = motion_command_message;
}


// Do obstacle_avoider
static void
base_ackerman_motion_command_message_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	base_motion_command_message = motion_command_message;

	plot_phi_in_trajectories();
	plot_v_in_trajectories();
}


static void 
shutdown_module(int sig)
{
	static int done = 0;

	printf("Signal %d received, exiting program ...\n", sig);

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("base_ackerman disconnected from IPC.\n");
		done = 1;
	}

	exit(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
subscribe_to_relevant_messages()
{
	carmen_subscribe_message((char *) RRT_PATH_NAME, (char *) RRT_PATH_FMT, NULL, sizeof(rrt_path_message),
			(carmen_handler_t) rrt_path_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_robot_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) robot_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int 
main(int argc, char** argv)
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	subscribe_to_relevant_messages();

	carmen_ipc_dispatch();

	return 0;
}
