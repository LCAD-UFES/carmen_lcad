#ifndef XSENS_XYZ_HANDLER_H
#define XSENS_XYZ_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#define XSENS_SENSOR_VECTOR_SIZE (10 * (100 / 20) * 2)

typedef struct 
{
	carmen_pose_3D_t xsens_pose;
	carmen_orientation_3D_t orientation;
	carmen_orientation_3D_t ang_velocity;

	carmen_pose_3D_t gps_pose_in_the_car;

	carmen_pose_3D_t sensor_board_pose;
	
	double last_xsens_message_timestamp;
	
	int initial_state_initialized;
	
	int extra_gps;
	carmen_gps_xyz_message gps_xyz;
	carmen_gps_gphdt_message gps_hdt;

	int gps_performance_changed;
	double gps_performance_degradation;

	int gps_orientation_performance_changed;
	double gps_orientation_performance_degradation;
} xsens_xyz_handler;


typedef struct
{
	carmen_vector_3D_t acceleration;
	carmen_orientation_3D_t orientation;
	carmen_orientation_3D_t ang_velocity;
	carmen_vector_3D_t position;

	carmen_orientation_3D_t orientation_std_error;
	carmen_vector_3D_t position_std_error;

	double timestamp;
} sensor_vector_xsens_xyz;

xsens_xyz_handler *create_xsens_xyz_handler(int argc, char **argv, carmen_fused_odometry_parameters *fused_odometry_parameters);
void reset_xsens_xyz_handler(xsens_xyz_handler *xsens_handler);
void destroy_xsens_xyz_handler(xsens_xyz_handler *xsens_handler);
int is_global_pos_initialized();
void add_gps_samples_into_particle_pool(carmen_gps_xyz_message *gps_xyz);
carmen_point_t get_std_error(xsens_xyz_handler *xsens_handler, carmen_fused_odometry_parameters *fused_odometry_parameters);


#ifdef __cplusplus
}
#endif

#endif
