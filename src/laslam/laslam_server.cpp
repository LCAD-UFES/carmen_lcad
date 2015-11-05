#include "laslam_server.h"

LandmarkSlamServer::LandmarkSlamServer ()
{
	gui_ = NULL;
}

void
LandmarkSlamServer::InitializeGui(carmen_vector_3D_t robot_size)
{
	gui_ = new View::GtkGui();

	gui_->setUpGTK();
	gui_->setUpOpenGL();

	gui_->setRobotParameters(robot_size);
//	gui_->setVisualOdometryPathParameters();
//	gui_->setCorrectedPathParameters();
//	gui_->setLandmarksParameters();
//	gui_->setParticlesParameters();
}

LandmarkSlamServer::~LandmarkSlamServer()
{

}

void LandmarkSlamServer::SpinOnce()
{
	//gui_->spinOnce();
	gui_->start();
}


