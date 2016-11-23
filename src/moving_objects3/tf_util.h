/*
 * tf_util.h
 *
 *  Created on: Nov 27, 2012
 *      Author: fmutz
 */

#ifndef __TF_UTIL_H__
#define __TF_UTIL_H__

#include <tf.h>
#include <carmen/carmen.h>

class TfFrameTransformationWrapper
{
	tf::Transform origin_to_destination_transform;

	public:

		TfFrameTransformationWrapper();
		TfFrameTransformationWrapper(carmen_pose_3D_t origin_frame, carmen_pose_3D_t destination_frame);
		~TfFrameTransformationWrapper();

		carmen_pose_3D_t transform(carmen_pose_3D_t point);
		tf::Transform get_transform() const;
};


tf::Vector3 carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector);
tf::Quaternion carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation);
carmen_vector_3D_t tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector);
carmen_orientation_3D_t get_carmen_orientation_from_tf_transform(tf::Transform transform);
carmen_pose_3D_t tf_transform_to_carmen_pose_3D(tf::Transform transform);
tf::Transform carmen_pose_3D_to_tf_transform(carmen_pose_3D_t carmen_pose_3D);
carmen_pose_3D_t transform_point_to_new_pose_using_tf(carmen_pose_3D_t, carmen_pose_3D_t, carmen_pose_3D_t);

#endif /* __TF_UTIL_H__ */
