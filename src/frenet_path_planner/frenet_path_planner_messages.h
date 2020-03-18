
#ifndef __CARMEN_FRENET_PATH_PLANNER_MESSAGES_H__
#define __CARMEN_FRENET_PATH_PLANNER_MESSAGES_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
    {
        int plan_size;	// it is equal to numbr_of_poses * frenet_path_planner_num_plans (frenet_path_planner_num_plans is a carmen.ini param)
        int number_of_poses;
        carmen_ackerman_traj_point_t *plan;
        int selected_plan;

        int number_of_poses_back;
        carmen_ackerman_traj_point_t *poses_back;

        int *annotations;
        int *annotations_codes;

        double timestamp;
        char *host;
    } carmen_frenet_path_planner_plan_message;

	#define CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_NAME "carmen_frenet_path_planner_plan_message"
	#define CARMEN_FRENET_PATH_PLANNER_PLAN_MESSAGE_FMT  "{int, int, <{double, double, double, double, double}:1>, int, int, <{double, double, double, double, double}:5>, <int:2>, <int:2>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
