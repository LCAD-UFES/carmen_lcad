#include <drawer_3d.h>
#include <btBulletDynamicsCommon.h>

namespace CVIS
{
	Drawer3D::Drawer3D(GTKGui * gui)
	{
		gui_ = gui;
	}

	Drawer3D::~Drawer3D()
	{

	}

	void Drawer3D::draw()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		setView();


		if(drawAxesEnabled)
			drawAxes();


		glPointSize(pointSize);
		drawPointCloud();
		//glPointSize(1.0);

	}

	void Drawer3D::drawPointCloud()
	{
		if(vbos_ != NULL)
		{
			glPushMatrix();
			glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbos_->vboPointCloud);

			glEnableClientState(GL_COLOR_ARRAY);
			glEnableClientState(GL_VERTEX_ARRAY);

			glColorPointer(3, GL_FLOAT, 0, (void*)((char*)NULL + (vbos_->pointCloud->pointCloudSize * vbos_->pointCloudNumber * vbos_->pointCloud->pointCloudDim * sizeof(GLfloat))));
			glVertexPointer(3, GL_FLOAT, 0, 0);

			glDrawArrays(GL_POINTS, 0, vbos_->pointCloud->pointCloudSize * vbos_->pointCloudNumber);

			glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
			glDisableClientState(GL_COLOR_ARRAY);

			glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

			glPopMatrix();
		}
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

	void Drawer3D::setView()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		glViewport(0,0, gui_->getCanvasWidth(), gui_->getCanvasHeight());

		gluPerspective(60.0, gui_->getCanvasHeight()/gui_->getCanvasHeight(), 0.0000001, 10000.0);

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
