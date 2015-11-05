#ifndef VOSLAM_KEYFRAMES_H_
#define VOSLAM_KEYFRAMES_H_

#include <vector>
#include <tf.h>
#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include "voslam_util.h"

class VoslamKeyframes {

private:
	float distance_offset;
	float angle_offset;
	std::vector<carmen_voslam_pointcloud_t> list;

	double framePositionDiference(tf::Vector3 frame_position, tf::Vector3 last_frame_position);
	double frameOrientationDiference(tf::Vector3 frame_orientation, tf::Vector3 last_frame_orientation, int ref);

public:
	std::vector<carmen_voslam_pointcloud_t>* getKeyframesList() { return &list; }
	int getKeyframesListSize() { return list.size(); }
	carmen_voslam_pointcloud_t* getSourceKeyframe();
	carmen_voslam_pointcloud_t* getTargetKeyframe();
	carmen_voslam_pointcloud_t* getKeyframe(int i);
	void addKeyframe(carmen_voslam_pointcloud_t* keyframe, stereo_util stereo_reprojection_params_g);
	int isKeyframe(carmen_voslam_pose_t pose);

	VoslamKeyframes(float distance, float angle);

	virtual ~VoslamKeyframes();

};

#endif /* VOSLAM_KEYFRAMES_H_ */
