#ifndef TF_HELPER_H
#define TF_HELPER_H

#ifdef __cplusplus
extern "C"
{
#endif

carmen_pose_3D_t
get_velodyne_pose_in_relation_to_car_helper(int argc, char** argv);

void
get_world_pose_with_velodyne_offset_initialize_helper(int argc, char **argv);

carmen_pose_3D_t
get_world_pose_with_velodyne_offset_helper(carmen_pose_3D_t world_pose);

#ifdef __cplusplus
}
#endif

#endif

