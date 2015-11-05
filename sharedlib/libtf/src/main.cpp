#include <carmen/carmen.h>

#include "tf.h"
#include <stdio.h>

int main()
{
	// See: http://www.ros.org/wiki/tf

	tf::Pose carmen_pose;
	tf::Pose camera_pose;
	tf::Pose vodom_pose;

	tf::StampedTransform vodom_to_carmen_transform;

	tf::Time::init();
	tf::Transformer transformer(false);

	vodom_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	vodom_pose.setRotation(tf::Quaternion(- M_PI / 2.0, 0.0, - M_PI / 2.0));

	camera_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	camera_pose.setRotation(tf::Quaternion(0.0, carmen_degrees_to_radians(5.0), 0.0));

	carmen_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	carmen_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::StampedTransform camera_to_world_transform(camera_pose, tf::Time(0), "/world", "/camera");
	transformer.setTransform(camera_to_world_transform, "camera_to_world_transform");

	tf::StampedTransform vodom_to_camera_transform(vodom_pose, tf::Time(0), "/camera", "/vodom");
	transformer.setTransform(vodom_to_camera_transform, "vodom_to_camera_transform");

	tf::StampedTransform carmen_to_world_transform(carmen_pose, tf::Time(0), "/world", "/carmen");
	transformer.setTransform(carmen_to_world_transform, "carmen_world_transform");

	// ----------------------------- //

	printf("Initial Poses:\n");

	transformer.lookupTransform("/carmen", "/vodom", tf::Time(0), vodom_to_carmen_transform);
	printf("Vodom pose with respect to Carmen  : x: % 6.2f, y: % 6.2f, z: % 6.2f\n", vodom_to_carmen_transform.getOrigin().x(), vodom_to_carmen_transform.getOrigin().y(), vodom_to_carmen_transform.getOrigin().z());

	printf("\n");

	for (double z = 0.0; z < 10.0; z += 1.0)
	{
		printf("Poses at x = %lf\n", z);

		vodom_pose.setOrigin(tf::Vector3(0.0, 0.0, z));
		vodom_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

		transformer.lookupTransform("/carmen", "/vodom", tf::Time(0), vodom_to_carmen_transform);
		carmen_pose = vodom_to_carmen_transform * vodom_pose;

		transformer.lookupTransform("/camera", "/vodom", tf::Time(0), vodom_to_camera_transform);
		camera_pose = vodom_to_camera_transform * vodom_pose;

		double yaw, pitch, roll;

		tf::Matrix3x3(vodom_pose.getRotation()).getRPY(roll, pitch, yaw);
		printf("Vodom pose with respect to World  x: % 6.2f, y: % 6.2f, z: % 6.2f, yaw: % 6.2f, pitch: % 6.2f, roll: %6.2f\n", vodom_pose.getOrigin().x(), vodom_pose.getOrigin().y(), vodom_pose.getOrigin().z(), carmen_radians_to_degrees(yaw), carmen_radians_to_degrees(pitch), carmen_radians_to_degrees(roll));

		tf::Matrix3x3(camera_pose.getRotation()).getRPY(roll, pitch, yaw);
		printf("Vodom pose with respect to Camera x: % 6.2f, y: % 6.2f, z: % 6.2f, yaw: % 6.2f, pitch: % 6.2f, roll: %6.2f\n", camera_pose.getOrigin().x(), camera_pose.getOrigin().y(), camera_pose.getOrigin().z(), carmen_radians_to_degrees(yaw), carmen_radians_to_degrees(pitch), carmen_radians_to_degrees(roll));

		tf::Matrix3x3(carmen_pose.getRotation()).getRPY(roll, pitch, yaw);
		printf("Vodom pose with respect to Carmen x: % 6.2f, y: % 6.2f, z: % 6.2f, yaw: % 6.2f, pitch: % 6.2f, roll: %6.2f\n", carmen_pose.getOrigin().x(), carmen_pose.getOrigin().y(), carmen_pose.getOrigin().z(), carmen_radians_to_degrees(yaw), carmen_radians_to_degrees(pitch), carmen_radians_to_degrees(roll));
	}

	return 0;
}
