/*
 * trajectory_lookup_table.h
 *
 *  Created on: Jun 10, 2015
 *      Author: alberto
 */

#ifndef TRAJECTORY_LOOKUP_TABLE_H_
#define TRAJECTORY_LOOKUP_TABLE_H_

#include <vector>
#include "carmen/rddf_interface.h"
#include "model/global_state.h"


#define N_DIST			21	//15 or 21	// Number of Distances traveled in polar coordinates
#define FIRST_DIST		2.3				// First Distance, or scale factor of its geometric progression (Wikipedia)
#define RATIO_DIST		1.18			// Ratio (Wikipedia) of the Distance geometric progression
#define ZERO_DIST_I		-1				// Index of zero Distance traveled

#define N_THETA			15				// Number of Angles in polar coordinates
#define COMMON_THETA	((8.0 * M_PI) / 180.0)	// Common difference in the arithmetic progression of Theta for each side
#define ZERO_THETA_I	7				// Index of zero Angle

#define N_D_YAW			15				// Number of Displacements in yaw
#define FIRST_D_YAW		((10.0 * M_PI) / 180.0)	// First Displacement in yaw, or scale factor of its geometric progression
#define RATIO_D_YAW		1.3				// Ratio (Wikipedia) of the Displacements in yaw geometric progression
#define ZERO_D_YAW_I	7				// Index of zero yaw displacement

#define N_I_PHI			15				// Number of Initial steering wheel angles (phi)
#define FIRST_I_PHI		((3.0 * M_PI) / 180.0)	// Scale factor of phi geometric progression
#define RATIO_I_PHI		1.394			// Ratio (Wikipedia) of phi geometric progression
#define ZERO_I_PHI_I	7				// Index of zero initial phi

#define N_K2			15				// Number of k2 wheel angles
#define FIRST_K2		((3.0 * M_PI) / 180.0)	// Scale factor of k2 geometric progression
#define RATIO_K2		1.394			// Ratio (Wikipedia) of k2 geometric progression
#define ZERO_K2_I		7				// Index of zero k2

#define N_K3			15				// Number of k3 wheel angles
#define FIRST_K3		((3.0 * M_PI) / 180.0)	// Scale factor of k3 geometric progression
#define RATIO_K3		1.394			// Ratio (Wikipedia) of k3 geometric progression
#define ZERO_K3_I		7				// Index of zero k3

#define N_I_V			9				// Number of Initial velocities
#define FIRST_I_V		1.3				// First Initial velocity, or scale factor of its geometric progression
#define RATIO_I_V		1.381				// Ratio (Wikipedia) of Initial velocity geometric progression
#define ZERO_I_V_I		0				// Index of zero Initial velocity

#define N_D_V			8				// Number of Velocities displacements
#define FIRST_D_V		1.6				// First Velocity displacement, or scale factor of its geometric progression
#define RATIO_D_V		1.3				// Ratio (Wikipedia) of Velocity displacement geometric progression
#define ZERO_D_V_I		4				// Index of zero Velocity displacement

#define NUM_VELOCITY_PROFILES	4

#define LATENCY_CICLE_TIME		0.01
#define MAX_PLANNING_TIME		1.0

//To error layout
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"


typedef enum
{
	CONSTANT_PROFILE = 0,
	LINEAR_PROFILE = 1,
	LINEAR_RAMP_PROFILE = 2,
	TRAPEZOIDAL_PROFILE = 3
} TrajectoryVelocityProfile;
#define PROFILE_TIME 			5.0


class TrajectoryLookupTable
{
public:

	struct TrajectoryControlParameters_old
	{
		bool valid;
		double tt;
		double k2;
		double k3;
		double k1;
		bool has_k1;
		double a;
		double vf;
		double sf;
		double s;
	};

	struct TrajectoryControlParameters
	{
		bool valid;
		double tt;
		double k2;
		double k3;
		double k1;
		bool has_k1;
		bool shift_knots;
		double a;
		double vf;
		double sf;
		double s;
	};

	struct TrajectoryDimensions
	{
		double dist;	// Distance traveled in polar coordinates
		double theta;	// Angle in polar coordinates
		double d_yaw;	// Displacement in yaw
		double phi_i;	// Initial steering wheel angle
		double v_i;		// Initial velocity

		struct TrajectoryControlParameters control_parameters;
	};

	//typedef struct _TrajectoryDimensions TrajectoryDimensions;


	struct TrajectoryDiscreteDimensions
	{
		unsigned int dist;		// Distance traveled in polar coordinates
		unsigned int theta;	// Angle in polar coordinates
		unsigned int d_yaw;	// Displacement in yaw
		unsigned int phi_i;	// Initial steering wheel angle
		unsigned int v_i;		// Initial velocity
	};

	//typedef struct _TrajectoryDiscreteDimensions TrajectoryDiscreteDimensions;

	struct Plan
	{
		vector<carmen_ackerman_path_point_t> path;
		double timestamp;
	};

	TrajectoryLookupTable(int update_lookup_table);

	bool load_trajectory_lookup_table_old();
	bool load_trajectory_lookup_table();
	void build_trajectory_lookup_table();
	void evaluate_trajectory_lookup_table();
	void update_lookup_table_entries();
};


typedef struct
{
	double phi;
	double timestamp;
} steering_delay_t;


void save_trajectory_lookup_table();
TrajectoryLookupTable::TrajectoryDimensions convert_to_trajectory_dimensions(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd,
		TrajectoryLookupTable::TrajectoryControlParameters tcp);
TrajectoryLookupTable::TrajectoryDiscreteDimensions get_discrete_dimensions(TrajectoryLookupTable::TrajectoryDimensions td);
bool has_valid_discretization(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd);
TrajectoryLookupTable::TrajectoryControlParameters search_lookup_table(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd);

vector<carmen_ackerman_path_point_t> simulate_car_from_parameters(TrajectoryLookupTable::TrajectoryDimensions &td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp, double v0, double i_phi,
		bool display_phi_profile, double delta_t = 0.15);

bool path_has_loop(double dist, double sf);
void move_path_to_current_robot_pose(vector<carmen_ackerman_path_point_t> &path, Pose *localizer_pose);
TrajectoryLookupTable::TrajectoryControlParameters get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> detailed_lane,
		bool use_lane, bool has_previous_good_tcp);

float get_d_yaw_by_index(int index);
float get_theta_by_index(int index);
float get_distance_by_index(int index);


#endif /* TRAJECTORY_LOOKUP_TABLE_H_ */
