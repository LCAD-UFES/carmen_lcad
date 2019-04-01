
#ifndef __SYNCHRONIZED_DATA_PACKAGE_H__
#define __SYNCHRONIZED_DATA_PACKAGE_H__

#include <string>
#include <Eigen/Geometry>
#include <carmen/segmap_pose2d.h>


class DataSample
{
public:
	DataSample();

	double velodyne_time;
	double image_time;
	double odom_time;
	double xsens_time;
	double gps_time;

	// timestamp of the sensor used for synchronization.
	double time;

	double v, phi;
	int gps_quality;
	int gps_orientation_quality;
	int image_width, image_height;
	int n_laser_shots;

	Eigen::Quaterniond xsens;

	Pose2d pose;
	Pose2d pose_with_loop_closure;
	Pose2d pose_registered_to_map;
	Pose2d gps;

	std::string velodyne_path;
	std::string image_path;
};


#endif
