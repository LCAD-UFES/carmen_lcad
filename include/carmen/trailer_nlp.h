#ifndef SHAREDLIB_LIB_TRAILER_NLP_H_
#define SHAREDLIB_LIB_TRAILER_NLP_H_

#include <vector>
#include <carmen/carmen.h>

using namespace std;


typedef struct {
	double d;
	double M;
	double distance_between_axle_and_front;
	double distance_between_axle_and_back;
} carmen_trailer_nlp_config_semi_t;


typedef struct
{
	double amax;
	double vmax;
	double wmax;
	double phimax;
	double distance_between_front_car_and_front_wheels;
	double distance_between_front_and_rear_axles;
	double distance_between_rear_car_and_rear_wheels;
	double distance_between_rear_wheels_half;
	carmen_trailer_nlp_config_semi_t semi_trailers[MAX_NUM_TRAILERS];
} carmen_trailer_nlp_config_all_t;


typedef struct
{
	double x;
	double y;
	double theta[MAX_NUM_TRAILERS];
	double phi;
	double v;
	double a;
	double w;
} carmen_trailer_nlp_initial_state_t;


typedef struct
{
	double x;
	double y;
	double theta[MAX_NUM_TRAILERS];
	double v;
	double a;
	double w;
} carmen_trailer_nlp_final_state_t;


typedef struct
{
	carmen_point_t poses[80][MAX_NUM_TRAILERS];
} carmen_trailer_nlp_initial_guess_t;


vector<carmen_robot_and_trailers_traj_point_t>
trailer_nlp_analytical_expansion_casadi(const carmen_robot_and_trailers_traj_point_t &current_pose, const carmen_robot_and_trailers_traj_point_t &goal_pose,
		const carmen_robot_ackerman_config_t &robot_config, const carmen_semi_trailers_config_t &semi_trailer_config, int limited_number_of_trailers);


#endif /* SHAREDLIB_LIB_TRAILER_NLP_H_ */
