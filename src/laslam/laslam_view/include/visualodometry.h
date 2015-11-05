#ifndef CARMEN_LASLAM_VISUALODOMETRY_H
#define CARMEN_LASLAM_VISUALODOMETRY_H

#include <vector>
#include <carmen/carmen.h>
#include <carmen/glm.h>

class VisualOdometry {

private:

	int path_counter_, path_size_;
	std::vector<carmen_vector_3D_t> path_;

public:
	VisualOdometry(int path_size);
	void draw();

	void addVisualOdometryPoseToPath(carmen_vector_3D_t visual_odometry_pose);

	virtual ~VisualOdometry();
};

#endif /* CARMEN_LASLAM_VISUALODOMETRY_H */
