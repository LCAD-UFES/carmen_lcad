#include <carmen/carmen.h>
#include "navigator_astar_messages.h"
#include "navigator_astar_interface.h"

#include <carmen/behavior_selector_messages.h>
#include <carmen/behavior_selector_interface.h>


carmen_point_t globalpos, goalpos;

void periodic_publish_globalpos()
{
	static carmen_localize_ackerman_globalpos_message message;
	message.globalpos = globalpos;
	message.timestamp = carmen_get_time();
	carmen_localize_ackerman_publish_globalpos_message(&message);
}

//void periodic_publish_goal()
//{
//	static carmen_behavior_selector_goal_list_message message;
//	static carmen_ackerman_traj_point_t goal;
//	goal.x = goalpos.x;
//	goal.y = goalpos.y;
//	goal.theta = goalpos.theta;
//	message.goal_list = &goal;
//	message.size = 1;
//
//	IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &message);
//}


int main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if (argc >= 6)
	{
		globalpos.x = atof(argv[1]);
		globalpos.y = atof(argv[2]);
		globalpos.theta = atof(argv[3]);

		goalpos.x = atof(argv[4]);
		goalpos.y = atof(argv[5]);
		goalpos.theta = atof(argv[6]);

		carmen_ipc_addPeriodicTimer(1, periodic_publish_globalpos, NULL);
//		carmen_ipc_addPeriodicTimer(1, periodic_publish_goal, NULL);
		carmen_ipc_dispatch();
	}
	printf("parametros insuficientes\n");
	return 0;

}
