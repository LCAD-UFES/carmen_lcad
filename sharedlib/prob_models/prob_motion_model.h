#ifndef CARMEN_PROB_MOTION_MODEL_H
#define CARMEN_PROB_MOTION_MODEL_H

#include <carmen/global.h>

#ifdef __cplusplus
extern "C" 
{
#endif

#define INITIAL_ODOMETRY {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
#define INITIAL_VELOCITY {0.0, 0.0, 0.0, 0.0}

typedef enum _MotionModelTypes
{
	OdometryMotionModel = 1,
	VelocityMotionModel = 2,
	AckermanMotionModel = 3
} MotionModelTypes;

typedef struct _OdometryMotionModelParams
{
	double alpha1, alpha4; //Sigma Error (angular motion)
	double alpha2, alpha3; //Sigma Error (linear motion)
} OdometryMotionModelParams;

typedef struct 
{
	carmen_point_t initial;
	carmen_point_t final;
} OdometryMotionCommand;

typedef struct 
{
	double alpha1, alpha3; //Sigma Error related to v (linear velocity)
	double alpha2, alpha4; //Sigma Error related to phi (steer angle)
	double alpha5, alpha6; //Sigma Error related to theta (head angle)
} VelocityMotionModelParams;

typedef struct 
{
	double L;//wheel base length
	double alpha1, alpha3; //Sigma Error related to v (linear velocity)
	double alpha2, alpha4; //Sigma Error related to phi (steer angle)
	double global_distance_threshold; // Threshold of particles dispersion to change to non-ackerman motion for helping localization
} AckermanMotionModelParams;

typedef struct 
{
	double v;
	double w;
	double delta_t;
	double last_timestamp;
} VelocityMotionCommand;

typedef struct 
{
	double v;
	double phi;
	double delta_t;
	double last_timestamp;
} AckermanMotionCommand;

static const double SMALL_TRANS = 0.001;
static const double SMALL_ROT = 0.001;

void init_odometry_motion_model(OdometryMotionModelParams params);
void init_velocity_motion_model(VelocityMotionModelParams params);
void init_ackerman_motion_model(AckermanMotionModelParams params);

carmen_point_t sample_motion_model_odometry(const OdometryMotionCommand *ut, carmen_point_t xt_1);
carmen_point_t sample_motion_model_velocity(const VelocityMotionCommand *ut, carmen_point_t xt_1);
carmen_point_t sample_motion_model_ackerman(const AckermanMotionCommand *ut, carmen_point_t xt_1, int converged);

int update_ackerman_motion_command(AckermanMotionCommand *ut, const double v, const double phi, const double timestamp);
int update_velocity_motion_command(VelocityMotionCommand *ut, const double v, const double w, const double timestamp);
void update_odometry_motion_command(OdometryMotionCommand *ut, carmen_point_t odometry);

#ifdef __cplusplus
}
#endif

#endif
