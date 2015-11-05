#ifndef DRAWER_3D_H
#define DRAWER_3D_H

#include <camera.h>
#include <gtk_gui.h>
#include <robot.h>
#include <odometry.h>
#include <saliency.h>

namespace View
{

class GtkGui;

const double COLOR_RED[3]  = {1.00, 0.00, 0.00};
const double COLOR_GREEN[3]  = {0.00, 1.00, 0.00};
const double COLOR_BLUE[3]  = {0.00, 0.00, 1.00};

const double COLOR_P_VOLUMES[3]  = {0.75, 0.00, 0.00};
const double COLOR_N_VOLUMES[3]  = {0.00, 0.00, 0.75};
const double COLOR_ML_VOLUMES[3] = {0.00, 0.75, 0.00};
const double GRID_COLOR[3]      = {0.70, 0.70, 0.70};

class Drawer3D
{
	private:
	GtkGui *gui_;
    Robot *robot_;
    Odometry *vodom_;
    Saliency *sal_;
    Saliency *corr_;

    Camera camera_;

    void drawAxes();
	void followCar();
	void driverCar();

	public:

	Drawer3D(GtkGui * gui);
	~Drawer3D();

    void draw();

    void setRobotParameters(carmen_vector_3D_t robot_size) { robot_->setRobotParameters(robot_size); }
    void setRobotPose(carmen_pose_3D_t robot_pose) { robot_->setRobotPose(robot_pose); }
    void setRobotCovarianceElipse(double angle, double major_axis, double minor_axis) { robot_->setRobotCovarianceElipse(angle, major_axis, minor_axis); }
    void setNeuralPose(carmen_pose_3D_t neural_pose) { robot_->setNeuralPose(neural_pose); }
    void setLocalizeAckermanPose(carmen_pose_3D_t localize_pose) { robot_->setLocalizeAckermanPose(localize_pose); }
    void setCameraPose(carmen_pose_3D_t camera_pose) { sal_->setCameraPose(camera_pose); corr_->setCameraPose(camera_pose);}
    void setNeuralSaliency(carmen_vector_3D_t neural_saliency) { sal_->setRobotPose(this->getNeuralPose()); sal_->addSaliencyPoseToSalienciesList(neural_saliency); }
    void setNeuralCorrespondence(carmen_vector_3D_t neural_correspondence) { corr_->setRobotPose(this->getRobotPose()); corr_->addSaliencyPoseToSalienciesList(neural_correspondence); }
    carmen_pose_3D_t getRobotPose() { return robot_->getRobotPose(); }
    carmen_pose_3D_t getNeuralPose() { return robot_->getNeuralPose(); }

    void addOdometryPose(carmen_vector_3D_t odometry_pose) { vodom_->addOdometryPoseToPath(odometry_pose); }

    double getCameraPosX() const { return camera_.getPosX(); }
    double getCameraPosY() const { return camera_.getPosY(); }
    double getCameraPosZ() const { return camera_.getPosZ(); }

    double getCameraLookX() const { return camera_.getLookX(); }
    double getCameraLookY() const { return camera_.getLookY(); }
    double getCameraLookZ() const { return camera_.getLookZ(); }

    void setCameraPosX(double x) { camera_.setPosX(x); }
    void setCameraPosY(double y) { camera_.setPosY(y); }
    void setCameraPosZ(double z) { camera_.setPosZ(z); }

    void setCameraLookX(double x) { camera_.setLookX(x); }
    void setCameraLookY(double y) { camera_.setLookY(y); }
    void setCameraLookZ(double z) { camera_.setLookZ(z); }

    void pan(double angle);
    void tilt(double angle);
    void zoom(double r);
    void move(double x, double y);

    void setView();
};

}

#endif
