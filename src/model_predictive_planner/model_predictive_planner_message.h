#ifndef MODEL_PREDICTIVE_PLANNER_MESSAGE_H
#define MODEL_PREDICTIVE_PLANNER_MESSAGE_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_robot_and_trailers_traj_point_t *plan;
	int	   plan_length;
	double timestamp;
	char  *host;
} carmen_model_predictive_planner_motion_plan_message;

#define CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME "carmen_model_predictive_planner_motion_plan"
#define CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT "{<{double, double, double, int, [double:5], double, double}:2>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif
