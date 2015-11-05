#include "neural_slam_server.h"

NeuralSlamServer::NeuralSlamServer ()
{
	gui_ = NULL;
}

void
NeuralSlamServer::InitializeGui(carmen_vector_3D_t robot_size)
{
	gui_ = new View::GtkGui();

	gui_->setUpGTK();
	gui_->setUpOpenGL();

	gui_->setRobotParameters(robot_size);
//	gui_->setVisualOdometryPathParameters();
//	gui_->setCorrectedPathParameters();
//	gui_->setSalienciesParameters();
//	gui_->setParticlesParameters();
}

NeuralSlamServer::~NeuralSlamServer()
{

}

void NeuralSlamServer::SpinOnce()
{
	//gui_->spinOnce();
	gui_->start();
}


