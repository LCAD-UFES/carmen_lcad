
#ifndef __SEGMAP_POSE_READER_H__
#define __SEGMAP_POSE_READER_H__

#include <vector>
#include <string>
#include <carmen/command_line.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>


class DatasetPose
{
public:
	int id;
	double time;
	Pose2d pose;
};


class PoseSet
{
public:
	PoseSet(std::string path, double offset_x, double offset_y);
	~PoseSet() {}

	Pose2d at(int i) const;
	int size() const;
	Pose2d operator[](int i) const;

	void set_offset(double offset_x, double offset_y);

protected:

	double _offset_x;
	double _offset_y;

	std::vector<DatasetPose> _poses;
};


enum PoseType
{
	POSE_TYPE_FUSED_ODOMETRY = 0,
	POSE_TYPE_GRAPHSLAM,
	POSE_TYPE_GRAPHSLAM_TO_MAP
};


PoseSet read_dataset_poses(std::string log_path, PoseType type, CommandLineArguments &args, NewCarmenDataset &dataset);


#endif
