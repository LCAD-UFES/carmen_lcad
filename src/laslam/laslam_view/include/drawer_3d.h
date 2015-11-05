#ifndef CARMEN_LASLAM_DRAWER_3D_H
#define CARMEN_LASLAM_DRAWER_3D_H

#include <camera.h>
#include <gtk_gui.h>
#include <robot.h>
#include <visualodometry.h>
#include <landmarks.h>

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
    VisualOdometry *vodom_;
    Landmark *landmark_;

    Camera camera_;

    void drawAxes();
	void followCar();
	void driverCar();

	public:

	Drawer3D(GtkGui * gui);
	~Drawer3D();

    void draw();

    void setRobotParameters(carmen_vector_3D_t robot_size) { robot_->setRobotParameters(robot_size); }
    void setRobotPose(carmen_pose_3D_t robot_pose) { robot_->setRobotPose(robot_pose); landmark_->setRobotPose(robot_pose); }
    carmen_pose_3D_t getRobotPose() { return robot_->getRobotPose(); }

    void addVisualOdometryPose(carmen_vector_3D_t visual_odometry_pose) { vodom_->addVisualOdometryPoseToPath(visual_odometry_pose); }
    void addLandmarkPoseToLandmarksList(carmen_vector_3D_t landmark_pose, unsigned char* crop) { landmark_->addLandmarkPoseToLandmarksList(landmark_pose, robot_->getRobotPose(), crop); }

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
