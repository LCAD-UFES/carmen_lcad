
#ifndef __VELODYNE_TRANSFORMS_H__
#define __VELODYNE_TRANSFORMS_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif

carmen_pose_3D_t
get_velodyne_pose_in_relation_to_car(int argc, char** argv);

void
get_world_pose_with_velodyne_offset_initialize(int argc, char **argv);

carmen_pose_3D_t
get_world_pose_with_velodyne_offset(carmen_pose_3D_t world_pose);

#ifdef __cplusplus
}
#endif

#endif
// @}
