#include <carmen/carmen.h>
#include "prob_transforms.h"
#include <tf.h>

tf::Transformer transformer(false);


void
tf_transform_robot_pose_to_laser_pose_initialize(const BeanRangeFinderMeasurementModelParams *laser_params)
{
	tf::Transform world_to_robot_pose;
	tf::Transform laser_to_robot_pose;

	//TODO definir altura do robô em relação ao mundo
	world_to_robot_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	world_to_robot_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::StampedTransform world_to_robot_transform(world_to_robot_pose, tf::Time(0), "/world", "/robot");
	transformer.setTransform(world_to_robot_transform, "world_to_robot_transform");

	//TODO definir altura do laser em relação ao robô
	laser_to_robot_pose.setOrigin( tf::Vector3(laser_params->front_offset, laser_params->side_offset, 0.0));
	laser_to_robot_pose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, laser_params->angular_offset));

	tf::StampedTransform robot_to_laser_transform(laser_to_robot_pose, tf::Time(0), "/robot", "/laser");

	transformer.setTransform(robot_to_laser_transform, "laser_to_robot_transform");
}


void
tf_transform_robot_pose_to_laser_pose_update(carmen_point_t *robot_pose_with_laser_offset, const carmen_point_t *robot_pose_without_laser_offset)
{
	tf::Transform world_to_robot_pose;
	tf::StampedTransform world_to_laser_pose;

	//TODO definir altura do robô em relação ao mundo
	world_to_robot_pose.setOrigin(tf::Vector3(robot_pose_without_laser_offset->x, robot_pose_without_laser_offset->y, 0.0));

	world_to_robot_pose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, robot_pose_without_laser_offset->theta));

	tf::StampedTransform world_to_robot_transform(world_to_robot_pose, tf::Time(0), "/world", "/robot");
	transformer.setTransform(world_to_robot_transform, "world_to_robot_transform");

	transformer.lookupTransform("/world", "/laser", tf::Time(0), world_to_laser_pose);

	double roll, pitch, yaw;
	tf::Matrix3x3(world_to_laser_pose.getRotation()).getRPY(roll, pitch, yaw);

	robot_pose_with_laser_offset->x = world_to_laser_pose.getOrigin().x();
	robot_pose_with_laser_offset->y = world_to_laser_pose.getOrigin().y();
	robot_pose_with_laser_offset->theta = yaw;
}


void 
transform_robot_pose_to_laser_pose(carmen_point_t *robot_pose_with_laser_offset, const carmen_point_t *robot_pose_without_laser_offset)
{
	tf_transform_robot_pose_to_laser_pose_update(robot_pose_with_laser_offset, robot_pose_without_laser_offset);
}


// Convert the raw data (as float) to double
double *
convert_zt(double *zt_buffer, const float *zt, const int num_readings, double max_range)
{
	int i;

	if (!zt_buffer)
		zt_buffer = (double*)malloc(num_readings * sizeof(double));

	for (i = 0; i < num_readings; i++)
	{
		if ((zt[i] >= max_range) || (zt[i] == 0.0))
			zt_buffer[i] = max_range;
		else
			zt_buffer[i] = (double)zt[i];
	}

	return zt_buffer;
}
