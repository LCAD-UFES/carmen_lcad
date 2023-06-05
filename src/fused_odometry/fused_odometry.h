#ifndef FUSED_ODOMETRY_H
#define FUSED_ODOMETRY_H


#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
	carmen_pose_3D_t 	pose;
	double			xsens_yaw_bias;
	carmen_vector_3D_t 	velocity;	// relative to the car and not to the global frame of reference. X points to the front of the car, Y to the side and Z up. Because this is relative to the car, Y and Z should be zero unless the car is slipping or flying.
	carmen_orientation_3D_t ang_velocity;	// rate at which orientation is changing
	double 			phi;		// This is the angle the wheels are turned.
	int num_trailers;
	double trailer_theta[MAX_NUM_TRAILERS];
	double 			timestamp;
} carmen_fused_odometry_state_vector;

typedef struct
{
	carmen_fused_odometry_state_vector 	state;
	double 					weight;
	int 					weight_type;
} carmen_fused_odometry_particle;

typedef struct
{
	double v;
	double phi;
	double z;
	double v_z;
	double v_pitch;

	double pitch;
	double roll;
	
	int gps_available;
} carmen_fused_odometry_control;

typedef struct
{
	double axis_distance;

	double velocity_noise_velocity;
	double velocity_noise_phi;
	double phi_noise_phi;
	double phi_noise_velocity;
	double pitch_v_noise_pitch_v;
	double pitch_v_noise_velocity;

	double minimum_speed_for_correction;
	double maximum_phi;

	double xsens_gps_x_std_error;
	double xsens_gps_y_std_error;
	double xsens_gps_z_std_error;

	double xsens_roll_std_error;
	double xsens_pitch_std_error;
	double xsens_yaw_std_error;
	
	double xsens_yaw_bias_noise;
	double xsens_maximum_yaw_bias;
} carmen_fused_odometry_parameters;


void init_particles(carmen_fused_odometry_state_vector initial_state, carmen_fused_odometry_parameters *fused_odometry_parameters);

void prediction(double timestamp, carmen_fused_odometry_parameters *fused_odometry_parameters);
void correction(double (*weight_func)(carmen_fused_odometry_state_vector, void *, carmen_fused_odometry_parameters *), void *sensor_vector, carmen_fused_odometry_parameters *fused_odometry_parameters);
void publish_fused_odometry(void);
void set_all_particles_to_initial_pose(double *initial_pose);

void set_fused_odometry_control_vector(carmen_fused_odometry_control c);
carmen_fused_odometry_control *get_fused_odometry_control_vector();
carmen_fused_odometry_message *get_fused_odometry();

void init_particles(carmen_fused_odometry_state_vector initial_state, carmen_fused_odometry_parameters *fused_odometry_parameters);
void carmen_fused_odometry_initialize(carmen_fused_odometry_parameters *fused_odometry_parameters);

void  localize_ackerman_initialize_from_xsens(carmen_fused_odometry_state_vector initial_state);
void  globalpos_ackerman_initialize_from_xsens(carmen_fused_odometry_state_vector initial_state, double timestamp);

#ifdef __cplusplus
}
#endif

#endif
