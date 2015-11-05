#ifndef ROBOT_H_
#define ROBOT_H_

#include <carmen/carmen.h>
#include <carmen/glm.h>

class Robot {

private:

	carmen_vector_3D_t robot_size_;
	carmen_pose_3D_t robot_pose_[5000];
	carmen_pose_3D_t neural_pose_[5000];
	carmen_pose_3D_t localize_pose_[10000];
	int robot_pose_length;
	int neural_pose_length;
	int localize_pose_length;
	carmen_pose_3D_t camera_pose_;
	GLMmodel* robot_model_;

	double angle_, major_axis_, minor_axis_;

public:
	Robot(char* model_name);
	void draw();
	void drawPath(bool correction, bool neural_global);
	void drawLocalizePath();
	void drawEllipse();
	void setRobotParameters(carmen_vector_3D_t robot_size);
	carmen_pose_3D_t getRobotPose();
	carmen_pose_3D_t getNeuralPose();
	carmen_pose_3D_t getLocalizeAckermanPose();
	void setRobotPose(carmen_pose_3D_t robot_pose) { this->robot_pose_[robot_pose_length] = robot_pose; robot_pose_length++; }
	void setRobotCovarianceElipse(double angle, double major_axis, double minor_axis) { this->angle_ = angle; this->major_axis_ =  major_axis; this->minor_axis_ = minor_axis; }
	void setNeuralPose(carmen_pose_3D_t neural_pose) { this->neural_pose_[neural_pose_length] = neural_pose; neural_pose_length++; }
	void setLocalizeAckermanPose(carmen_pose_3D_t localize_pose) { this->localize_pose_[localize_pose_length] = localize_pose; localize_pose_length++; }
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
