#ifndef GLOBAL_PLANNING_H
#define GLOBAL_PLANNING_H
#include <carmen/global.h>
#include <carmen/carmen_gps_wrapper.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	NOT_IN_FRONT = 1,
	IS_IN_FRONT = 2,
	JUST_STOPPED_BEING_IN_FRONT = 3,
} annotation_state_transition;


int carmen_global_planning_pose_passed_through_annotation(int annotation_type, carmen_point_t pose, carmen_rddf_annotation_message *annotation_message);


#ifdef __cplusplus
}
#endif
#endif