
#include <carmen/segmap_pose2d.h>
#include <carmen/synchronized_data_package.h>


DataSample::DataSample()
{
	velodyne_time = 0.0;
	image_time = 0.0;
	odom_time = 0.0;
	xsens_time = 0.0;
	gps_time = 0.0;
	time = 0.0;

	v = 0;
	phi = 0;

	gps_quality = 0;
	gps_orientation_quality = 0;
	image_width = 0;
	image_height = 0;
	n_laser_shots = 0;

	xsens = Eigen::Quaterniond(0, 0, 0, 0);

	pose = Pose2d(0, 0, 0);
	pose_with_loop_closure = Pose2d(0, 0, 0);
	pose_registered_to_map = Pose2d(0, 0, 0);
	gps = Pose2d(0, 0, 0);

	velodyne_path = "";
	image_path = "";
}
