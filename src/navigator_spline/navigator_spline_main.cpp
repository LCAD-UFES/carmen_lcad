
#include <carmen/carmen.h>
#include <string.h>
#include <carmen/rddf_interface.h>
#include <carmen/tracker_interface.h>
#include "navigator_spline_interface.h"

#include <vector>
#include "spline.h"

carmen_point_t globalpos = {0, 0, 0};
std::vector<carmen_localize_ackerman_globalpos_message>  global_localize_ackerman_globalpos_message;
#define MAX_POSITIONS 20
#define EXTRA_POSITIONS 20
static carmen_robot_and_trailer_traj_point_t poses[MAX_POSITIONS + EXTRA_POSITIONS + 100];


static void tracker_position_handler(carmen_tracker_position_message *message);
void publish_spline_path_message(carmen_robot_and_trailer_traj_point_t *poses, int number_of_poses);
static void publish_spline_goal_message(carmen_robot_and_trailer_traj_point_t poses);
//void publish_spline_goal_list_message(carmen_robot_and_trailer_traj_point_t *poses, int number_of_poses);


void
carmen_navigator_spline_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_NAVIGATOR_SPLINE_PATH_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_SPLINE_PATH_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_SPLINE_PATH_NAME);
}


/*********************************************************
		   --- Publishers ---
 **********************************************************/
static void
periodic_publish_spline_path_message()
{
	printf("periodic_publish_spline_path_message\n");

//	publish_spline_path_message(poses, poses_size);
	publish_spline_goal_message(poses[20]);
}


void
publish_spline_path_message(carmen_robot_and_trailer_traj_point_t *poses, int number_of_poses)
{
	IPC_RETURN_TYPE err;
	carmen_navigator_spline_path_message spline_path_message;

	spline_path_message.goal_list = poses;
	spline_path_message.size = number_of_poses;
	spline_path_message.host = carmen_get_host();
	spline_path_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_NAVIGATOR_SPLINE_PATH_NAME, &spline_path_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_SPLINE_PATH_NAME);
}


void publish_spline_goal_message(carmen_robot_and_trailer_traj_point_t poses)
{
	IPC_RETURN_TYPE err;
	carmen_navigator_ackerman_set_goal_message goal_msg;
	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME);
		initialized = 1;
	}

	goal_msg.x = poses.x;
	goal_msg.y = poses.y;
	goal_msg.theta = globalpos.theta;
	goal_msg.host = carmen_get_host();
	goal_msg.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME, &goal_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME);
}


//void publish_spline_goal_list_message(carmen_robot_and_trailer_traj_point_t *poses, int number_of_poses)
//{
//	IPC_RETURN_TYPE err;
//	carmen_behavior_selector_goal_list_message goal_list_msg;
//	poses->theta = globalpos.theta;
//
//	goal_list_msg.goal_list = poses;
//	goal_list_msg.size = number_of_poses;
//	goal_list_msg.host = carmen_get_host();
//	goal_list_msg.timestamp = carmen_get_time();
//	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &goal_list_msg);
//	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
//}


/*********************************************************
		   --- Handlers ---
 **********************************************************/

////static void
////rddf_message_handler(carmen_rddf_road_profile_message *message)
//{

//	std::vector<double> X;
//	std::vector<double> Y;
//	std::vector<double> I;
//
//	globalpos = global_localize_ackerman_globalpos_message[global_localize_ackerman_globalpos_message.size() - 1].globalpos;
//	if (globalpos.x == 0 && globalpos.y == 0 && globalpos.theta == 0)
//		return;
//
//	X.push_back(globalpos.x);
//	Y.push_back(globalpos.y);
//	I.push_back(0);
//
//	X.push_back(message->poses[5].x);
//	Y.push_back(message->poses[5].y);
//	I.push_back(5);
//
//	X.push_back(message->poses[20].x);
//	Y.push_back(message->poses[20].y);
//	I.push_back(20);
//
//	tk::spline x;
//	x.set_points(I,X);
//
//	tk::spline y;
//	y.set_points(I,Y);
//
//	message->number_of_poses = 20;
//
//	printf("poses %d\n", message->number_of_poses);
//	for(int i = 0; i < message->number_of_poses; i++)
//	{
//		poses[i].x = x(I[0] + i);
//		poses[i].y = y(I[0] + i);
//		poses[i].v = 2;
//		poses[i].theta = globalpos.theta;
//	}
//
//	publish_spline_path_message(poses, message->number_of_poses);
//
//	carmen_point_t goal;
//	goal.x = poses[8].x;
//	goal.y = poses[8].y;
//	goal.theta = globalpos.theta;
//	carmen_behavior_selector_add_goal(goal);
//	publish_spline_goal_list_message(poses + 3, 1);
//}



//static void
//rddf_message_handler(carmen_rddf_road_profile_message *message)
//{
//	carmen_tracker_position_message tracker_position_message;
//	tracker_position_message.object_position.x = message->poses[10].x - globalpos.x;
//	tracker_position_message.object_position.y = message->poses[10].y - globalpos.y;
//
//	tracker_position_message.timestamp = message->timestamp;
//	tracker_position_message.host = message->host;
//	tracker_position_handler(&tracker_position_message);
//}



static void
tracker_position_handler(carmen_tracker_position_message *message)
{
	static std::vector<double> X;
	static std::vector<double> Y;
	static std::vector<double> I;
	static unsigned int index = 0;

	tk::spline x, y;
	double distancia;
	double angulo;


	if (global_localize_ackerman_globalpos_message.size() == 0)
	{
		printf("globalpos nao inicializado!\n");
		return;
	}
	else
	{
		for (unsigned int i = 0; i < global_localize_ackerman_globalpos_message.size(); i++)
		{
			if (fabs(message->timestamp - global_localize_ackerman_globalpos_message[i].timestamp) < 0.01)
			{
				globalpos = global_localize_ackerman_globalpos_message[i].globalpos;
			}
		}
	}

	if (globalpos.x == 0 && globalpos.y == 0 && globalpos.theta == 0)
	{
		printf("globalpos nao inicializado!\n");
		return;
	}

	if (message->object_position.x == 0.0)
	{
		printf("pose zerada: message->object_position.x %.2f message->object_position.y %.2f\n",message->object_position.x,message->object_position.y);
		return;
	}

	distancia = sqrt(pow(message->object_position.x, 2) + pow(message->object_position.y, 2));
	angulo = atan2(message->object_position.y, message->object_position.x);
	distancia -= 4;

	X.push_back(globalpos.x + distancia * cos(carmen_normalize_theta(globalpos.theta + carmen_degrees_to_radians(angulo))) - 0);
	Y.push_back(globalpos.y + distancia * sin(carmen_normalize_theta(globalpos.theta + carmen_degrees_to_radians(angulo))));
	I.push_back(index++);

	if (I.size() > MAX_POSITIONS)
	{
		X.erase(X.begin());
		Y.erase(Y.begin());
		I.erase(I.begin());
	}

	if (I.size() > 1)
	{
		x.set_points(I,X);
		y.set_points(I,Y);

		for (unsigned int i = 0; i < I.size() + (unsigned int)EXTRA_POSITIONS; i++)
		{
			poses[i].x = x(I[0] + i);
			poses[i].y = y(I[0] + i);
		}
	}
	carmen_point_t goal;
	goal.x = poses[I.size() - 1].x;
	goal.y = poses[I.size() - 1].y;
	goal.theta = globalpos.theta;
	carmen_behavior_selector_clear_goal_list();
	carmen_behavior_selector_add_goal(goal);
	//publish_spline_goal_list_message(poses + I.size() - 1, 1); //todo colocar o menor possivel, pode ajudar.
//	publish_spline_path_message(poses, I.size() + EXTRA_POSITIONS);

	printf("pose X %f Y %f I %d theta %f\t", message->object_position.x, message->object_position.y, (int)I[I.size() - 1], globalpos.theta);
	printf("I.size(): %d\n",(int)I.size());
}

static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *message)
{

	global_localize_ackerman_globalpos_message.push_back(*message);
	static int primeiro = 0;

	if(primeiro == 1)
		return;
	primeiro = 1;


//	carmen_tracker_position_message message_test;
//	message_test.host = message->host;
//	message_test.timestamp = message->timestamp;
//	message_test.object_position.x = 10;
//	message_test.object_position.x = 0;
//
//	tracker_position_handler(&message_test);
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("navigator_spline: disconnected.\n");

		exit(0);
	}
}

int 
main(int argc, char **argv) 
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	carmen_navigator_spline_define_messages();
	carmen_tracker_define_message_position();

    //carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_tracker_subscribe_position_message(NULL,(carmen_handler_t) tracker_position_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ipc_addPeriodicTimer(1 / 2, (TIMER_HANDLER_TYPE) periodic_publish_spline_path_message, NULL);
	carmen_ipc_dispatch();

	return (0);
}
