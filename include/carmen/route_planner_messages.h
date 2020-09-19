/*
 * route_planner_messages.h
 *
 *  Created on: 14/04/2020
 *      Author: Alberto
 */

#ifndef ROUTE_PLANNER_MESSAGES_H_
#define ROUTE_PLANNER_MESSAGES_H_

#include <carmen/carmen.h>


#ifdef __cplusplus
extern "C" {
#endif


#define ROUTE_PLANNER_GET_LANE_WIDTH(traffic_restrictions) (((double) (traffic_restrictions & 0x3f)) * 0.1)
#define ROUTE_PLANNER_SET_LANE_WIDTH(traffic_restrictions, lane_width) ((traffic_restrictions & ~0x3f) | ((int) (lane_width * 10.0) & 0x3f))

typedef enum
{
	NO_REQUEST,
	PLAN_FROM_POSE_TO_LANE,
	PLAN_FROM_LANE_TO_FINAL_POSE,
	WITHIN_OFFROAD_PLAN
} offroad_planner_request_t;


typedef enum ROUTE_PLANNER_FEEDBACK
{
	ROUTE_PLANNED,
	ROUTE_PLANNER_IN_RDDF_MODE,
	COULD_NOT_COMPUTE_THE_ROUTE,
	PUBLISHING_ROUTE,
	IDLE
} carmen_route_planner_feedback_t;

#define print_route_planner_feedback(x) ( \
	(x == ROUTE_PLANNED)? "ROUTE_PLANNED": \
	(x == ROUTE_PLANNER_IN_RDDF_MODE)? "ROUTE_PLANNER_IN_RDDF_MODE": \
	(x == COULD_NOT_COMPUTE_THE_ROUTE)? "COULD_NOT_COMPUTE_THE_ROUTE": \
	(x == PUBLISHING_ROUTE)? "PUBLISHING_ROUTE": \
	(x == IDLE)? "IDLE": ""	)


typedef struct
{
	int number_of_poses;
	int number_of_poses_back;
	carmen_ackerman_traj_point_t *poses;
	carmen_ackerman_traj_point_t *poses_back;
	int *annotations;
	int *annotations_codes;

	int number_of_nearby_lanes;
	int *nearby_lanes_indexes;	// O ponto em nearby_lanes onde comecca cada lane.
	int *nearby_lanes_sizes;	// O tamanho de cada lane.
	int *nearby_lanes_ids;		// Cada id eh um codigo que identifica uma lane unicamente.
	int nearby_lanes_size;		// Igual ao numero de poses de todas as lanes somado.
	carmen_ackerman_traj_point_t *nearby_lanes;	// Todas as lanes (number_of_nearby_lanes), uma apos a outra. A primeira lane eh sempre a rota e sempre deve ter id = 0, jah que ela eh uma composicao de lanes do grafo
	int *traffic_restrictions; 	// LANE_WIDTH | LEFT_MARKING | RIGHT_MARKING | LEVEL | YIELD | BIFURCATION
								//   6 bits   | 3 bits enum  |  3 bits enum  | 2 bits| 1 bit |   1 bit

    //  Uma network com tres lanes com tamanhos 5, 3 e 6 poses teria:
    //  number_of_nearby_lanes = 3
    //	nearby_lanes_indexes -> 0, 5, 8
    //	nearby_lanes_sizes -> 5, 3, 6
    //	nearby_lanes_size = 5+3+6 = 14
    //	nearby_lanes (p_lane_pose) -> p_0_0, p_0_1, p_0_2, p_0_3, p_0_4, p_1_0, p_1_1, p_1_2, p_2_0, p_2_1, p_2_2, p_2_3, p_2_4, p_2_5

	int offroad_planner_request;
	carmen_route_planner_feedback_t route_planner_feedback;

    double timestamp;
    char *host;
} carmen_route_planner_road_network_message;

#define		CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME		"carmen_route_planner_road_network_message"
#define		CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_FMT		"{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, <int:1>, int, <int:7>, <int:7>, <int:7>, int, <{double, double, double, double, double}:11>, <int:11>, int, int, double, string}"


typedef struct
{
	char *destination;
	carmen_point_t destination_point;
	double timestamp;
	char *host;
} carmen_route_planner_destination_message;

#define		CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME		"carmen_route_planner_destination_message"
#define		CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_FMT		"{string, {double, double, double}, double, string}"


typedef struct
{
	char *pallet;
	carmen_point_t pallet_point;
	char *destination;
	carmen_point_t destination_point;
	double timestamp;
	char *host;
} carmen_route_planner_pallet_and_destination_message;

#define		CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME	"carmen_route_planner_pallet_and_destination_message"
#define		CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_FMT		"{string, {double, double, double}, string, {double, double, double}, double, string}"


typedef struct
{
	int route_id;
	int status;			/* enabled:1 , disabled:0 */
	double timestamp;
	char *host;
} carmen_route_planner_route_status_change_message;

#define		CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME		"carmen_route_planner_route_status_change"
#define		CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_FMT		"{int, int, double, string}"


typedef struct
{
	char *road_netword_id;
	double timestamp;
	char *host;
} carmen_route_planner_route_reload_message;

#define		CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME				"carmen_route_planner_route_reload"
#define		CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_FMT				"{string, double, string}"


#ifndef EDGE_TYPE_
#define EDGE_TYPE_
// These structs are defined in a C++ library (road_network_generator_utils.h) but this is a C library

typedef struct
{
	int u;
	int v;
	int u_ref;
	int v_ref;
	double cost;
} edge_t;

#define DISABLING_COST	1.0e100

typedef struct
{
	int id;
	carmen_position_t start;
	carmen_position_t end;
	edge_t edge;
	int status;		/* enabled:1 , disabled:0 */
} route_t;

#endif


typedef struct
{
	carmen_position_t center;
	double range;
	int number_of_routes;
	route_t *routes;
	double timestamp;
	char *host;
} carmen_route_planner_route_list_message;

#define		CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME		"carmen_route_planner_route_list_request"
#define		CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT			"{{double, double}, double, int, <{int, {double, double}, {double, double}, {int, int, int, int, double}, int}:3>, double, string}"

#define		CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME		"carmen_route_planner_route_list_response"
#define		CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_FMT		CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT


#ifdef __cplusplus
}
#endif

#endif /* ROUTE_PLANNER_MESSAGES_H_ */
