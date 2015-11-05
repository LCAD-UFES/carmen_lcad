#ifndef CARMEN_PROB_TRANSFORMS_H
#define CARMEN_PROB_TRANSFORMS_H

#include "prob_measurement_model.h"

#ifdef __cplusplus
extern "C" 
{
#endif

void tf_transform_robot_pose_to_laser_pose_initialize(const BeanRangeFinderMeasurementModelParams *laser_params);

void tf_transform_robot_pose_to_laser_pose_update(carmen_point_t *robot_pose_with_laser_offset, const carmen_point_t *robot_pose_without_laser_offset);

void transform_robot_pose_to_laser_pose(carmen_point_t *robot_pose_with_laser_offset, const carmen_point_t *robot_pose_without_laser_offset);

double *convert_zt(double *zt_buffer, const float *zt, const int num_readings, double max_range);

#ifdef __cplusplus
}
#endif

#endif
