
#ifndef __CARMEN_FRENET_PATH_PLANNER_MESSAGES_H__
#define __CARMEN_FRENET_PATH_PLANNER_MESSAGES_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
    {
        int set_of_paths_size;	// it is equal to numbr_of_poses * frenet_path_planner_num_paths (frenet_path_planner_num_paths is a carmen.ini parameter)
        int number_of_poses;
        carmen_ackerman_traj_point_t *set_of_paths;
        int selected_path;

        int number_of_poses_back;
        carmen_ackerman_traj_point_t *poses_back;

        carmen_ackerman_traj_point_t *rddf_poses_ahead;
        carmen_ackerman_traj_point_t *rddf_poses_back;

        int *annotations;
        int *annotations_codes;

        double timestamp;
        char *host;
    } carmen_frenet_path_planner_set_of_paths;

	#define CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME "carmen_frenet_path_planner_set_of_paths"
	#define CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_FMT  "{int, int, <{double, double, double, double, double}:1>, int, int, <{double, double, double, double, double}:5>, <{double, double, double, double, double}:2>, <{double, double, double, double, double}:5>, <int:2>, <int:2>, double, string}"


	typedef struct
    {
        int selected_path;

        double timestamp;
        char *host;
    } carmen_frenet_path_planner_selected_path;

	#define CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_NAME "carmen_frenet_path_planner_selected_path"
	#define CARMEN_FRENET_PATH_PLANNER_SELECTED_PATH_MESSAGE_FMT  "{int, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
