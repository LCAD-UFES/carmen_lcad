
#include <carmen/velodyne_transforms.h>
#include <carmen/rotation_geometry.h>
#include "tf_helper.h"

carmen_pose_3D_t
get_velodyne_pose_in_relation_to_car(int argc, char** argv)
{		
	return (get_velodyne_pose_in_relation_to_car_helper(argc, argv));
}


void
get_world_pose_with_velodyne_offset_initialize(int argc, char **argv)
{
	get_world_pose_with_velodyne_offset_initialize_helper(argc, argv);
}


carmen_pose_3D_t
get_world_pose_with_velodyne_offset(carmen_pose_3D_t world_pose)
{
	return (get_world_pose_with_velodyne_offset_helper(world_pose));
}
