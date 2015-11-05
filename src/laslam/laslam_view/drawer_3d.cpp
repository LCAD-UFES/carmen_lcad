#include <drawer_3d.h>
#include <btBulletDynamicsCommon.h>

namespace View
{
	Drawer3D::Drawer3D(GtkGui * gui)
	{
		gui_ = gui;
		robot_ = new Robot((char *) "Touareg2.obj");
		vodom_ = new VisualOdometry(10000);
		landmark_ = new Landmark(1000);
	}

	Drawer3D::~Drawer3D()
	{

	}

	void Drawer3D::draw()
	{
		glScalef(1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		setView();
		drawAxes();

		if(gui_->getOptions()->useDriverCamera)
			this->driverCar();

		if(gui_->getOptions()->useFollowCamera)
			this->followCar();

		if(gui_->getOptions()->drawRobot)
			robot_->draw();

		if(gui_->getOptions()->drawVisualOdometryPath)
			vodom_->draw();

		landmark_->draw(gui_->getOptions()->drawLandmarkPoints, gui_->getOptions()->drawDistanceInformation);
	}

	void Drawer3D::drawAxes()
	{
		glLineWidth(2.0);

		glColor3d(1.0, 0.0, 0.0);

		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(1.0, 0.0, 0.0);
		glEnd();

		glColor3d(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(0.0, 1.0, 0.0);
		glEnd();

		glColor3d(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 1.0);
		glEnd();

		glLineWidth(1.0);
	}

	void Drawer3D::followCar()
	{
		setCameraPosX(robot_->getRobotPose().position.x - 30);
		setCameraPosY(robot_->getRobotPose().position.y + 20);
		setCameraPosY(robot_->getRobotPose().position.z - 20);

		setCameraLookX(robot_->getRobotPose().position.x);
		setCameraLookY(robot_->getRobotPose().position.y);
		setCameraLookZ(robot_->getRobotPose().position.z);
	}

	void Drawer3D::driverCar()
	{

	}

	void Drawer3D::setView()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		glViewport(0,0, gui_->getCanvasWidth(), gui_->getCanvasHeight());

		gluPerspective(45.0, gui_->getCanvasWidth()/gui_->getCanvasHeight(), 0.01, 10000.0);

		// display camera view
		gluLookAt(camera_.getPosX(),  camera_.getPosY(),  camera_.getPosZ(),
							camera_.getLookX(), camera_.getLookY(), camera_.getLookZ(),
							0.0,               0.0,               1.0);
	}

	void Drawer3D::pan(double angle)
	{
		camera_.pan(angle);
		setView();
	}

	void Drawer3D::tilt(double angle)
	{
		camera_.tilt(angle);
		setView();
	}

	void Drawer3D::zoom(double r)
	{
		camera_.zoom(r);
		setView();
	}

	void Drawer3D::move(double x, double y)
	{
		camera_.move(x, y);
		setView();
	}

}
