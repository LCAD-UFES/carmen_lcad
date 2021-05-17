
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
        carmen_robot_and_trailer_traj_point_t *set_of_paths;
        int selected_path;

        int number_of_poses_back;
        carmen_robot_and_trailer_traj_point_t *poses_back;

        carmen_robot_and_trailer_traj_point_t *rddf_poses_ahead;
        carmen_robot_and_trailer_traj_point_t *rddf_poses_back;

        int *annotations;
        int *annotations_codes;

    	int number_of_nearby_lanes;
    	int *nearby_lanes_indexes;	// O ponto em nearby_lanes onde comecca cada lane.
    	int *nearby_lanes_sizes;	// O tamanho de cada lane.
    	int *nearby_lanes_ids;		// Cada id eh um codigo que identifica uma lane unicamente.
    	int nearby_lanes_size;		// Igual ao numero de poses de todas as lanes somado.
    	carmen_robot_and_trailer_traj_point_t *nearby_lanes;	// Todas as lanes (number_of_nearby_lanes), uma apos a outra. A primeira lane eh sempre a rota e sempre deve ter id = 0, jah que ela eh uma composicao de lanes do grafo
    	int *traffic_restrictions; 	// Veja route_planner_messages.h.

    	double timestamp;
        char *host;
    } carmen_frenet_path_planner_set_of_paths;

	#define CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_NAME "carmen_frenet_path_planner_set_of_paths"
	#define CARMEN_FRENET_PATH_PLANNER_SET_OF_PATHS_MESSAGE_FMT  "{int, int, <{double, double, double, double, double, double}:1>, int, int, <{double, double, double, double, double, double}:5>, <{double, double, double, double, double, double}:2>, <{double, double, double, double, double, double}:5>, <int:2>, <int:2>, int, <int:11>, <int:11>, <int:11>, int, <{double, double, double, double, double, double}:15>, <int:15>, double, string}"


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
