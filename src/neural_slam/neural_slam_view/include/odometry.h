#ifndef VISUALODOMETRY_H_
#define VISUALODOMETRY_H_

#include <vector>
#include <carmen/carmen.h>
#include <carmen/glm.h>

class Odometry {

private:

	int path_counter_, path_size_;
	std::vector<carmen_vector_3D_t> path_;

public:
	Odometry(int path_size);
	void draw();

	void addOdometryPoseToPath(carmen_vector_3D_t visual_odometry_pose);

	virtual ~Odometry();
};

#endif /* VISUALODOMETRY_H_ */
