#ifndef CARMEN_LASLAM_LANDMARK_H
#define CARMEN_LASLAM_LANDMARK_H

#include <vector>
#include <carmen/carmen.h>
#include <carmen/glm.h>
#include <Eigen/Core>


typedef struct
{
	GLuint texture;
	carmen_vector_3D_t pose;
	double distance, phi, theta;
	unsigned char* cropped_image;
}carmen_laslam_landmark_t;


class Landmark {

private:

	carmen_pose_3D_t robot_pose_;

	int landmarks_counter_, landmarks_size_;
	std::vector<carmen_laslam_landmark_t> landmarks;

	void drawLandmark();
	void drawDistanceInformation();

	Eigen::Matrix<float, 3, 3>
	getLandmarkRotationMatrixPhi(carmen_laslam_landmark_t landmark);

	Eigen::Matrix<float, 3, 3>
	getLandmarkRotationMatrixTheta(carmen_laslam_landmark_t landmark);

public:
	Landmark(int landmarks_size);

	void draw(bool drawLandmark, bool drawDistanceInformation);
	void addLandmarkPoseToLandmarksList(carmen_vector_3D_t landmark_pose, carmen_pose_3D_t robot_pose, unsigned char* crop);
	void setRobotPose(carmen_pose_3D_t robot_pose) { this->robot_pose_ = robot_pose; }

	virtual ~Landmark();
};

#endif /* CARMEN_LASLAM_LANDMARK_H */
