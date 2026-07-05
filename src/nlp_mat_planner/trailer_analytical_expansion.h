#ifndef TRAILER_ANALYTICAL_EXPANSION_H
#define TRAILER_ANALYTICAL_EXPANSION_H

#include <utility>
#include <vector>

#include <carmen/carmen.h>
#include "nlp_mat_planner.h"

/* The type of container used to hold the state vector */
typedef std::vector< double > solver_state_t;


typedef struct
{
	double a;
	double b;
	double c;
	double d;
	double e;
	double f;
	double g;
	double h;
} kugler_coef_t;


typedef struct
{
	double xRef;
	double dxRef;
	double d2xRef;
	double d3xRef;
	double d4xRef;
} kugler_xRef_t;


typedef struct
{
	double xRef;
	double dxref_dsigma;
	double d2xref_dsigma2;
	double d3xref_dsigma3;
	double d4xref_dsigma4;
	double yRef;
	double dyref_dsigma;
	double d2yref_dsigma2;
	double d3yref_dsigma3;
	double d4yref_dsigma4;
	double dyxx;
} kugler_Ref_t;


typedef struct
{
	double x;
	double y;
	double theta0;
	double theta1;
	double phi;
} kugler_state_t;

typedef struct
{
	kugler_coef_t coef;
	double d0;
	double d1;
	double T;
	double x0;	// x da Start pose do trailer
	double x1;	// x da Final pose do trailer
	double k0;
	double k1;
	double k2;
	double k3;
	int direction;
	double M;
} kuglerParameters_t;

typedef struct
{
	double Lf_h1;
	double Lf_h2;
	double Lf2_h1;
	double Lf2_h2;
	double Lf3_h1;
	double Lf3_h2;
	double Lf4_h1;
	double Lf4_h2;
	double L_g1_Lf3_h1 ;
	double L_g2_Lf3_h1;
	double L_g1_Lf3_h2 ;
	double L_g2_Lf3_h2;
} lie_derivative_t;



carmen_robot_and_trailers_traj_point_t
change_pose_to_relative_coordinates(carmen_robot_and_trailers_traj_point_t reference_pose, carmen_robot_and_trailers_traj_point_t pose);

void
get_start_and_final_states(kugler_state_t &start_x0, kugler_state_t &start_x1, kugler_state_t &final_x0, kugler_state_t &final_x1,
		int direction,
		carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates, double d1,
		carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates);

kugler_coef_t
KuglerPathPlanner(kugler_state_t state_x0, kugler_state_t state_x1, double d0, double d1, double M);

solver_state_t
get_ode_start_state(int direction, const kugler_state_t &start_x0, const kugler_state_t &final_x0, double xi_1, double xi_2, double xi_3);

void
change_path_to_absolute_coordinates(std::vector<carmen_robot_and_trailers_traj_point_t> &path, carmen_robot_and_trailers_traj_point_t localizer_pose);

kugler_Ref_t
KuglerCalcRefValues_ODE(double &eta, double &xRef_dot, double t, kuglerParameters_t Parameters);

std::pair<bool, std::vector<carmen_robot_and_trailers_traj_point_t>>
trailer_nlp_analytical_expansion(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
		double max_phi_multiplier, bool test_limits, bool make_table, goal_constraints_t goal_constraints, int semi_trailer_solver_type);

#endif
