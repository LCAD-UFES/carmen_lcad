#ifndef SALIENCY_H_
#define SALIENCY_H_

#include <vector>
#include <carmen/carmen.h>
#include <carmen/glm.h>
#include <Eigen/Core>
#include <tf.h>


typedef struct
{
	GLuint texture;
	carmen_vector_3D_t pose;
	double distance, phi, theta;
	unsigned char* cropped_image;
}carmen_saliency_opengl_t;


class Saliency {

private:

	int color_;
	carmen_pose_3D_t robot_pose_;
	carmen_pose_3D_t camera_pose_;

	int saliencies_counter_, saliencies_size_;
	std::vector<carmen_saliency_opengl_t> saliencies;

	void drawSaliency();
	void drawDistanceInformation();

	Eigen::Matrix<float, 3, 3>
	getSaliencyRotationMatrixPhi(carmen_saliency_opengl_t saliency);

	Eigen::Matrix<float, 3, 3>
	getSaliencyRotationMatrixTheta(carmen_saliency_opengl_t saliency);

public:
	Saliency(int saliencies_size, int color);

	void draw(bool drawSaliency, bool drawDistanceInformation);
	void addSaliencyPoseToSalienciesList(carmen_vector_3D_t saliency_pose);
	void setRobotPose(carmen_pose_3D_t robot_pose) { this->robot_pose_ = robot_pose; }
	void setCameraPose(carmen_pose_3D_t camera_pose) { this->camera_pose_ = camera_pose; }

	virtual ~Saliency();
};

#endif /* SALIENCY_H_ */
