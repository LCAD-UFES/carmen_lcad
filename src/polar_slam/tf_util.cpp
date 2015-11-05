/*
 * tf_util.c
 *
 *  Created on: Nov 27, 2012
 *      Author: fmutz
 */

#include "tf_util.h"
#include <tf.h>
#include <carmen/carmen.h>

tf::Vector3
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return tf_vector;
}


tf::Quaternion
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


carmen_vector_3D_t
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();

	return carmen_vector;
}


carmen_orientation_3D_t
get_carmen_orientation_from_tf_transform(tf::Transform transform)
{
	carmen_orientation_3D_t orientation;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(orientation.yaw, orientation.pitch, orientation.roll);

	return orientation;
}


carmen_pose_3D_t
tf_transform_to_carmen_pose_3D(tf::Transform transform)
{
	carmen_pose_3D_t pose;

	pose.position = tf_vector3_to_carmen_vector3(transform.getOrigin());
	pose.orientation = get_carmen_orientation_from_tf_transform(transform);

	return pose;
}


tf::Transform
carmen_pose_3D_to_tf_transform(carmen_pose_3D_t carmen_pose_3D)
{
	tf::Transform transformation;

	transformation.setOrigin(carmen_vector3_to_tf_vector3(carmen_pose_3D.position));
	transformation.setRotation(carmen_rotation_to_tf_quaternion(carmen_pose_3D.orientation));

	return transformation;
}


TfFrameTransformationWrapper::TfFrameTransformationWrapper()
{
	origin_to_destination_transform = tf::Transform::getIdentity();
}


TfFrameTransformationWrapper::TfFrameTransformationWrapper(carmen_pose_3D_t origin_frame, carmen_pose_3D_t destination_frame)
{
	tf::Transformer transformer(false);
	tf::Transform world_to_origin;
	tf::Transform world_to_destination;

	tf::Time::init();

	world_to_origin.setOrigin(tf::Vector3(
		origin_frame.position.x,
		origin_frame.position.y,
		0.0 // origin_frame.position.z
	));

	world_to_origin.setRotation(tf::Quaternion(
		origin_frame.orientation.yaw,
		0.0, // origin_frame.orientation.pitch,
		0.0  // origin_frame.orientation.roll
	));

	tf::StampedTransform world_to_origin_transform(world_to_origin, tf::Time(0), "/world", "/origin");
	transformer.setTransform(world_to_origin_transform, "world_to_origin");

	world_to_destination.setOrigin(tf::Vector3(
		destination_frame.position.x,
		destination_frame.position.y,
		0.0 // destination_frame.position.z
	));

	world_to_destination.setRotation(tf::Quaternion(
		destination_frame.orientation.yaw,
		0.0, // destination_frame.orientation.pitch,
		0.0  // destination_frame.orientation.roll
	));

	tf::StampedTransform world_to_destination_transform(world_to_destination, tf::Time(0), "/world", "/destination");
	transformer.setTransform(world_to_destination_transform, "world_to_destination_transform");

	 // transformer.lookupTransform("/destination", "/origin", tf::Time(0), origin_to_destination_transform);
	origin_to_destination_transform = world_to_origin_transform.inverse() * world_to_destination;
}


TfFrameTransformationWrapper::~TfFrameTransformationWrapper()
{
}


carmen_pose_3D_t
TfFrameTransformationWrapper::transform(carmen_pose_3D_t point)
{
	tf::Transform point_tf = carmen_pose_3D_to_tf_transform(point);
	tf::Transform destination_point_tf = origin_to_destination_transform * point_tf;
	carmen_pose_3D_t destination_point = tf_transform_to_carmen_pose_3D(destination_point_tf);
	return destination_point;
}


tf::Transform
TfFrameTransformationWrapper::get_transform() const
{
	return origin_to_destination_transform;
}
