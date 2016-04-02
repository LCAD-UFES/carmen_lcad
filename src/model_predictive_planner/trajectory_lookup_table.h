/*
 * trajectory_lookup_table.h
 *
 *  Created on: Jun 10, 2015
 *      Author: alberto
 */

#ifndef TRAJECTORY_LOOKUP_TABLE_H_
#define TRAJECTORY_LOOKUP_TABLE_H_

#include "carmen/rddf_interface.h"

#define N_DIST			15				// Number of Distances traveled in polar coordinates
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

#define N_K1			15				// Number of k1 wheel angles
#define FIRST_K1		((3.0 * M_PI) / 180.0)	// Scale factor of k1 geometric progression
#define RATIO_K1		1.394			// Ratio (Wikipedia) of k1 geometric progression
#define ZERO_K1_I		7				// Index of zero k1

#define N_K2			15				// Number of k2 wheel angles
#define FIRST_K2		((3.0 * M_PI) / 180.0)	// Scale factor of k2 geometric progression
#define RATIO_K2		1.394			// Ratio (Wikipedia) of k2 geometric progression
#define ZERO_K2_I		7				// Index of zero k2

#define N_I_V			8				// Number of Initial velocities
#define FIRST_I_V		1.3				// First Initial velocity, or scale factor of its geometric progression
#define RATIO_I_V		1.381				// Ratio (Wikipedia) of Initial velocity geometric progression
#define ZERO_I_V_I		0				// Index of zero Initial velocity

#define N_D_V			8				// Number of Velocities displacements
#define FIRST_D_V		1.6				// First Velocity displacement, or scale factor of its geometric progression
#define RATIO_D_V		1.3				// Ratio (Wikipedia) of Velocity displacement geometric progression
#define ZERO_D_V_I		4				// Index of zero Velocity displacement

#define NUM_VELOCITY_PROFILES	4
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

	struct TrajectoryControlParameters
	{
		bool valid;
		TrajectoryVelocityProfile velocity_profile;
		double v0;
		double vt;
		double vf;
		double a0;
		double af;
		double t0;
		double tt;
		double tf;
		double k1;
		double k2;
		double sf;
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

	static Robot_State predict_next_pose(Robot_State &robot_state, const Command &requested_command,
			double full_time_interval, double *distance_traveled, double delta_t);
	bool load_trajectory_lookup_table();
	void build_trajectory_lookup_table();
	void evaluate_trajectory_lookup_table();
	static vector<vector<carmen_ackerman_path_point_t>> compute_path_to_goal(Pose *localize_pose, Pose *goal_pose,
			Command last_odometry, double max_v, carmen_rddf_road_profile_message *goal_list_message);
	void update_lookup_table_entries();

};

void save_trajectory_lookup_table();

#endif /* TRAJECTORY_LOOKUP_TABLE_H_ */
