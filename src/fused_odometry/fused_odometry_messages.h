
#ifndef CARMEN_FUSED_ODOMETRY_MESSAGES_H
#define CARMEN_FUSED_ODOMETRY_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 
{

	carmen_pose_3D_t 	pose;
	double				xsens_yaw_bias;
	carmen_vector_3D_t 	velocity;
	carmen_orientation_3D_t angular_velocity;
	double 				phi;
	double				beta;
	
	carmen_vector_3D_t	gps_position_at_turn_on;

	double 				timestamp;
	char 				*host;
} carmen_fused_odometry_message;
  
#define      CARMEN_FUSED_ODOMETRY_NAME			"carmen_fused_odometry_message"
#define      CARMEN_FUSED_ODOMETRY_FMT			"{{{double,double,double},{double,double,double}},double,{double,double,double},{double,double,double},double,double,{double,double,double},double,string}"

typedef struct 
{

	carmen_pose_3D_t 	pose;
	double				xsens_yaw_bias;
	carmen_vector_3D_t 	velocity;
	carmen_orientation_3D_t angular_velocity;
	double 				phi;
	double				beta;
	
	carmen_vector_3D_t	gps_position_at_turn_on;

	int 				num_particles;
	carmen_vector_3D_t 	*particle_pos;

	double 				*weights;
	int 				weight_type;	// 0-gps 1-imu

	double 				timestamp;
	char 				*host;
} carmen_fused_odometry_particle_message;
  
#define      CARMEN_FUSED_ODOMETRY_PARTICLE_NAME	"carmen_fused_odometry_particle_message"
#define      CARMEN_FUSED_ODOMETRY_PARTICLE_FMT		"{{{double,double,double},{double,double,double}},double,{double,double,double},{double,double,double},double,double,{double,double,double},int,<{double,double,double}:7>,<{double}:7>,int,double,string}"


#ifdef __cplusplus
}
#endif

#endif
// @}
