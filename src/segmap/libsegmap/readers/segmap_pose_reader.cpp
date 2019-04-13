
#include <carmen/command_line.h>
#include <carmen/segmap_pose_reader.h>
#include <carmen/util_io.h>
#include <cstdio>
#include <cstdlib>


PoseSet::PoseSet(std::string path, double offset_x, double offset_y)
{
	_offset_x = offset_x;
	_offset_y = offset_y;

	int n;
	FILE *f = safe_fopen(path.c_str(), "r");
	double gps_x, gps_y;

	while (!feof(f))
	{
		DatasetPose dpose;

		n = fscanf(f, "%d %lf %lf %lf %lf %lf %lf",
		           &dpose.id, &dpose.pose.x, &dpose.pose.y, &dpose.pose.th,
		           &dpose.time, &gps_x, &gps_y);

		if (n != 7)
			continue;

		if (_poses.size() > 1)
			assert(_poses[_poses.size() - 1].id == (_poses[_poses.size() - 2].id + 1));

		_poses.push_back(dpose);
	}

	fclose(f);
}


Pose2d
PoseSet::at(int i) const
{
	assert(i == _poses[i].id);
	assert(i < _poses.size());

	Pose2d out_pose(_poses[i].pose);
	out_pose.x -= _offset_x;
	out_pose.y -= _offset_y;

	return out_pose;
}


int
PoseSet::size() const
{
	return _poses.size();
}


Pose2d
PoseSet::operator[](int i) const
{
	return at(i);
}


void
PoseSet::set_offset(double offset_x, double offset_y)
{
	_offset_x = offset_x;
	_offset_y = offset_y;
}


PoseSet
read_dataset_poses(std::string log_path, PoseType type, CommandLineArguments &args, NewCarmenDataset &dataset)
{
	std::string path;

	if (type == POSE_TYPE_FUSED_ODOMETRY)
		path = default_fused_odom_path(log_path.c_str());
	else if (type == POSE_TYPE_GRAPHSLAM)
		path = default_graphslam_path(log_path.c_str());
	else if (type == POSE_TYPE_GRAPHSLAM_TO_MAP)
			path = default_graphslam_to_map_path(log_path.c_str());
	else
		exit(printf("Error: invalid pose type '%d'\n", (int) type));

	PoseSet poses(path, args.get<double>("offset_x"), args.get<double>("offset_y"));
	assert(poses.size() == dataset.size());

	return poses;
}

