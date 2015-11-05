#ifndef CARMEN_LASLAM_ROBOT_H
#define CARMEN_LASLAM_ROBOT_H

#include <carmen/carmen.h>
#include <carmen/glm.h>

class Robot {

private:

	carmen_vector_3D_t robot_size_;
	carmen_pose_3D_t robot_pose_;
	carmen_pose_3D_t camera_pose_;
	GLMmodel* robot_model_;

public:
	Robot(char* model_name);
	void draw();
	void setRobotParameters(carmen_vector_3D_t robot_size);
	carmen_pose_3D_t getRobotPose() { return robot_pose_; }
	void setRobotPose(carmen_pose_3D_t robot_pose) { this->robot_pose_ = robot_pose; }
	virtual ~Robot();
};

#endif /* CARMEN_LASLAM_ROBOT_H */
