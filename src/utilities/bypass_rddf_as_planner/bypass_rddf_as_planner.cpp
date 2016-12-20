
#include <carmen/carmen.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/rddf_interface.h>
#include "g2o/types/slam2d/se2.h"

using namespace g2o;

carmen_localize_ackerman_globalpos_message globalpos;
bool go_active = false;

double
dist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}



SE2
compute_goal_pose_in_car_reference(carmen_point_t globalpos, carmen_ackerman_traj_point_t goal)
{
	SE2 p(globalpos.x, globalpos.y, globalpos.theta);
	SE2 g(goal.x, goal.y, goal.theta);
	SE2 o = p.inverse() * g;

	return o;
}


SE2
compute_goal_pose_in_car_reference(carmen_ackerman_traj_point_t globalpos, carmen_ackerman_traj_point_t goal)
{
	SE2 p(globalpos.x, globalpos.y, globalpos.theta);
	SE2 g(goal.x, goal.y, goal.theta);
	SE2 o = p.inverse() * g;

	return o;
}


//Tree
//build_tree(carmen_ackerman_motion_command_t *motion_commands)
//{
//	Tree tree;
//	return tree;
//}
//
//
//void
//publish_plan_tree_message(Tree tree)
//{
//	static carmen_navigator_ackerman_plan_tree_message plan_tree_msg;
//	IPC_RETURN_TYPE err = IPC_OK;
//	static bool first_time = true;
//
//	if (first_time)
//	{
//		define_plan_tree_message();
//		plan_tree_msg.host = carmen_get_host();
//		first_time = false;
//	}
//
//	plan_tree_msg.timestamp = GlobalState::localizer_pose_timestamp;//carmen_get_time();
//	plan_tree_msg.num_edges = tree.num_edges;
//
//	plan_tree_msg.p1 = tree.p1;
//	plan_tree_msg.p2 = tree.p2;
//	plan_tree_msg.mask = tree.mask;
//
//	plan_tree_msg.num_path = tree.num_paths;
//	if (plan_tree_msg.num_path > 500)
//	{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
//		printf("Error: plan_tree_msg.num_path > 500 in Publisher_Util::publish_plan_tree_message()\n");
//		exit(1);
//	}
//	for (int i = 0; i < tree.num_paths; i++)
//	{
//		if (tree.paths_sizes[i] > 100)
//		{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
//			printf("Error: paths_sizes[%d] > 100 in Publisher_Util::publish_plan_tree_message()\n", i);
//			exit(1);
//		}
//		memcpy(plan_tree_msg.paths[i], tree.paths[i], sizeof(carmen_ackerman_traj_point_t) * tree.paths_sizes[i]);
//		plan_tree_msg.path_size[i] = tree.paths_sizes[i];
//	}
//
//	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);
//
//	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
//}


void
rddf_handler(carmen_rddf_road_profile_message *rddf)
{
	double d;
	double t;
	double p;

	int n = (rddf->number_of_poses > 20) ? (20) : (rddf->number_of_poses);
	int s = 5;

	if (rddf->number_of_poses <= s)
		return;

	carmen_ackerman_motion_command_t *motion_commands;
	motion_commands = new carmen_ackerman_motion_command_t[n - s];

	for (int i = s; i < n; i++)
	{
		double desired_v;

		if (globalpos.v < rddf->poses[i].v)
			desired_v = globalpos.v + 0.2;
		else
			desired_v = rddf->poses[i].v;

		if (desired_v <= 1.0)
			desired_v = 1.0;

		if (i < (n - 1))
		{
			d = dist(rddf->poses[i].x,rddf->poses[i].y, rddf->poses[i + 1].x, rddf->poses[i + 1].y);
			if (d <= 0.1) d = 0.1;
			t = d / desired_v;

			SE2 goal;
			double dth;

			//if (i <= s)
			{
				goal = compute_goal_pose_in_car_reference(globalpos.globalpos, rddf->poses[i]);
				dth = atan2(goal[1], goal[0]);
				//p = atan((dth * 2.625) / (desired_v * 1.0));
				p = atan((dth * 2.625) / (3.0 * 1.0));
			}
			//else
			//{
				//p = atan(((rddf->poses[i + 1].theta - rddf->poses[i].theta) * 2.625) / (t * desired_v));
			//}
		}
		else
		{
			t = 0.05;
			p = 0;
		}

		motion_commands[i - s].time = t;
		motion_commands[i - s].v = desired_v; //rddf->poses[i].v;
		motion_commands[i - s].phi = p;

		motion_commands[i - s].x = rddf->poses[i].x;
		motion_commands[i - s].y = rddf->poses[i].y;
		motion_commands[i - s].theta = rddf->poses[i].theta;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands, n - s, rddf->timestamp);
	delete(motion_commands);
}


carmen_behavior_selector_road_profile_message *lane;

void
lane_message_handler(carmen_behavior_selector_road_profile_message *message)
{
	lane = message;
}


static void
navigator_ackerman_go_message_handler()
{
	go_active = true;
}


static void
navigator_ackerman_stop_message_handler()
{
	go_active = false;
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);

	exit(1);
}

void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (!go_active || msg == NULL || lane == NULL || lane->number_of_poses < 10)
		return;

	int num_motion_commands=0.0, size = lane->number_of_poses - 1;
	double distance=0.0, previous_dist=0.0, desired_v=0.0;
	carmen_ackerman_motion_command_t *motion_commands;

	int i=0;

//	for (i=0.0; i < size; i++)
//	{
//		printf("%lf\n", lane->poses[i].x);
//	}

//	printf("%lf %lf %lf %lf\n", lane->poses[i].x, lane->poses[i].y, msg->globalpos.x, msg->globalpos.y);
//
//	for (i=0; i < size; i++)
//	{
//		distance = dist(msg->globalpos.x, msg->globalpos.y, lane->poses[i].x, lane->poses[i].y);
//
//		if (distance < previous_dist)
//		{
//			break;
//		}
//		else
//		{
//			previous_dist = distance;
//		}
//	}
//
//	printf("%d\n", i);

	i = 1;
	num_motion_commands = size - i;
	motion_commands = new carmen_ackerman_motion_command_t[num_motion_commands];
	for (int j=0.0; i < size; i++, j++)
	{
		if (msg->v < lane->poses->v)
			desired_v = msg->v + 0.2;
		else
			desired_v = lane->poses->v;

		distance = dist(lane->poses[i].x, lane->poses[i].y, lane->poses[i+1].x, lane->poses[i+1].y);

		motion_commands[j].time = distance / desired_v;
		motion_commands[j].v = desired_v;
		motion_commands[j].phi = lane->poses[i].phi;
		motion_commands[j].x = lane->poses[i].x;
		motion_commands[j].y = lane->poses[i].y;
		motion_commands[j].theta = lane->poses[i].theta;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands, num_motion_commands, msg->timestamp);
	delete(motion_commands);
}


void
register_handlers()
{
	signal(SIGINT, signal_handler);

	//carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);

	//carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

	//carmen_obstacle_distance_mapper_subscribe_message(NULL,	(carmen_handler_t) carmen_obstacle_distance_mapper_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message(
    		(char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
    		NULL, sizeof (carmen_behavior_selector_road_profile_message),
			(carmen_handler_t) lane_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
		(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
		(char *) CARMEN_DEFAULT_MESSAGE_FMT,
		NULL, sizeof(carmen_navigator_ackerman_go_message),
		(carmen_handler_t)navigator_ackerman_go_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
		(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
		(char *) CARMEN_DEFAULT_MESSAGE_FMT,
		NULL, sizeof(carmen_navigator_ackerman_stop_message),
		(carmen_handler_t)navigator_ackerman_stop_message_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	//carmen_param_check_version(argv[0]);

	define_messages();

	register_handlers();

	carmen_ipc_dispatch();

	return 0;
}
