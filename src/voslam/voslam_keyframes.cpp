#include "voslam_keyframes.h"

VoslamKeyframes::VoslamKeyframes(float distance, float angle)
{
	this->distance_offset = distance;
	this->angle_offset = carmen_degrees_to_radians(angle);
}

VoslamKeyframes::~VoslamKeyframes()
{
	// TODO Auto-generated destructor stub
}

carmen_voslam_pointcloud_t*
VoslamKeyframes::getSourceKeyframe()
{
	return &(this->list[this->list.size() - 2]);
}

carmen_voslam_pointcloud_t*
VoslamKeyframes::getTargetKeyframe()
{
	return &(this->list[this->list.size() - 1]);
}

carmen_voslam_pointcloud_t*
VoslamKeyframes::getKeyframe(int i)
{
	return &(this->list[i]);
}

void
VoslamKeyframes::addKeyframe(carmen_voslam_pointcloud_t* keyframe, stereo_util stereo_reprojection_params_g)
{
	carmen_voslam_pointcloud_t local_keyframe;

	local_keyframe.pose = keyframe->pose;

	local_keyframe.pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
	local_keyframe.pointcloud->height = stereo_reprojection_params_g.height;
	local_keyframe.pointcloud->width = stereo_reprojection_params_g.width;
	local_keyframe.pointcloud_size = keyframe->pointcloud_size;
	local_keyframe.pointcloud->points.resize(local_keyframe.pointcloud->width * local_keyframe.pointcloud->height);

	*local_keyframe.pointcloud = *keyframe->pointcloud;


	local_keyframe.image = (unsigned char*) calloc (3 *  stereo_reprojection_params_g.height * stereo_reprojection_params_g.width,  sizeof(unsigned char));
	carmen_test_alloc(local_keyframe.image);

	memcpy(local_keyframe.image, keyframe->image, 3 *  stereo_reprojection_params_g.height * stereo_reprojection_params_g.width * sizeof(unsigned char));

#ifndef NO_CUDA
	local_keyframe.depth = (unsigned short*) calloc ( stereo_reprojection_params_g.height * stereo_reprojection_params_g.width,  sizeof(unsigned short));
	carmen_test_alloc(local_keyframe.depth);

	memcpy(local_keyframe.depth, keyframe->depth,  stereo_reprojection_params_g.height * stereo_reprojection_params_g.width * sizeof(unsigned short));
#endif

	local_keyframe.timestamp = keyframe->timestamp;

	this->list.push_back(local_keyframe);
}

int
VoslamKeyframes::isKeyframe(carmen_voslam_pose_t pose)
{
	carmen_voslam_pose_t last_keyframe_pose;
	last_keyframe_pose = this->list[this->list.size() - 1].pose;

	if(this->framePositionDiference(pose.position, last_keyframe_pose.position) > this->distance_offset)
	{
		last_keyframe_pose.position[0] = pose.position[0];
		last_keyframe_pose.position[1] = pose.position[1];
		last_keyframe_pose.position[2] = pose.position[2];
		last_keyframe_pose.orientation[0] = pose.orientation[0];
		last_keyframe_pose.orientation[1] = pose.orientation[1];
		last_keyframe_pose.orientation[2] = pose.orientation[2];

		return 1;
	}

	if(this->frameOrientationDiference(pose.orientation, last_keyframe_pose.orientation, 0) > this->angle_offset ||
	   this->frameOrientationDiference(pose.orientation, last_keyframe_pose.orientation, 1) > this->angle_offset ||
	   this->frameOrientationDiference(pose.orientation, last_keyframe_pose.orientation, 2) > this->angle_offset)
	{
		last_keyframe_pose.position[0] = pose.position[0];
		last_keyframe_pose.position[1] = pose.position[1];
		last_keyframe_pose.position[2] = pose.position[2];
		last_keyframe_pose.orientation[0] = pose.orientation[0];
		last_keyframe_pose.orientation[1] = pose.orientation[1];
		last_keyframe_pose.orientation[2] = pose.orientation[2];

		return 1;
	}

	return 0;
}

double
VoslamKeyframes::framePositionDiference(tf::Vector3 frame_position, tf::Vector3 last_frame_position)
{
	return sqrt((frame_position[0] - last_frame_position[0])*(frame_position[0] - last_frame_position[0]) +
				(frame_position[1] - last_frame_position[1])*(frame_position[1] - last_frame_position[1]) +
				(frame_position[2] - last_frame_position[2])*(frame_position[2] - last_frame_position[2]));
}

double
VoslamKeyframes::frameOrientationDiference(tf::Vector3 frame_orientation, tf::Vector3 last_frame_orientation, int ref)
{
	double difference = 0.0;

	switch(ref)
	{
		//pitch diff
		case 0:
			difference = fabs(frame_orientation[0]-last_frame_orientation[0]);
			break;

		//roll diff
		case 1:
			difference = fabs(frame_orientation[1]-last_frame_orientation[1]);
			break;

		//yaw diff
		case 2:
			difference = fabs(frame_orientation[2]-last_frame_orientation[2]);
			break;
		default:
			break;
	}

	return difference;
}
