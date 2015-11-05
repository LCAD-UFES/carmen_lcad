#include <mvog_gtk_gui/map_drawer_3d.h>
#include <btBulletDynamicsCommon.h>

namespace MVOG
{

MapDrawer3D::MapDrawer3D(GTKGui * gui)
{
  gui_ = gui;
  cutoffAbove = 0.0;
  cutoffBelow = 0.0;
  maxHeight = 60.0;
  robot_offset_x = 100;
  robot_offset_y = 100;
}

MapDrawer3D::~MapDrawer3D()
{

}


void MapDrawer3D::setRobot(double x, double y, double z,
		double yaw, double pitch, double roll)
{
	this->robot_.x = x;
	this->robot_.y = y;
	this->robot_.z = z;
	this->robot_.yaw = yaw;
	this->robot_.pitch = pitch;
	this->robot_.roll = roll;
}

Robot MapDrawer3D::getRobot()
{
	return this->robot_;
}

void MapDrawer3D::draw()
{
  setView();

  Robot robot = getRobot();

  drawGrid(robot);
  drawAxes();

  if(gui_->getDrawRobot())
  	drawRobot(robot);

  if (gui_->getDrawRawData())
  {
    //if (gui_->getDrawPVolumes()) drawPVolumes(robot);
    //if (gui_->getDrawNVolumes()) drawNVolumes(robot);
  }
  else
  {
  	drawMLolumes(robot);
  }
}

void MapDrawer3D::drawRobot(Robot robot)
{

	glLineWidth(10.0);

	glColor3d(1.0, 0.0, 0.0);

	double x, y, z;

	x = fmod(robot.x, map_->sizeX_ * map_->resolution_);
	y = fmod(robot.y, map_->sizeY_ * map_->resolution_);
	z = robot.z;

	glPushMatrix();
	//glScaled(map_->resolution_, map_->resolution_, map_->resolution_);
	//glTranslatef(robot.x, robot.y, 0.0);

	glBegin(GL_LINES);
		glVertex3d(x , y, z);
		glVertex3d(x+1.0, y, z);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(x, y, z);
		glVertex3d(x, y+1.0, z);
	glEnd();


	glBegin(GL_LINES);
		glVertex3d(x, y, z);
		glVertex3d(x, y, z+1.0);
	glEnd();

	glPopMatrix();

	glLineWidth(1.0);
}

void MapDrawer3D::drawMLolumes(Robot robot)
{

	for (int i = 0; i < map_->sizeX_; ++i)
	for (int j = 0; j < map_->sizeY_; ++j)
	{
    MLVolumeVector vect;
    map_->grid_[i][j].createMLVolumes(vect);

    double x = i - map_->offsetX_;
    double y = j - map_->offsetY_;

		glPushMatrix();

    glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
		glTranslatef(x, y, 0.0);

		for (size_t c = 0; c < vect.size(); ++c)
		{
			if(vect[c].top < ((1 - cutoffAbove) * maxHeight) && vect[c].bot > ((cutoffBelow - 1) * maxHeight))
			{
				if(gui_->getColorByHeight())
						drawHeightColorVolume(vect[c].top,  vect[c].bot);
				else
						drawSolidColorVolume(vect[c].top, vect[c].bot, COLOR_ML_VOLUMES);
			}
		}

    glPopMatrix();
  }

}

void MapDrawer3D::drawGrid(Robot robot)
{
  glClearColor(1.0, 1.0, 1.0, 0);
  glLineWidth(2.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3dv(GRID_COLOR);

  glPushMatrix();
  glScaled(map_->resolution_, map_->resolution_, 1.0);
  glTranslated(-map_->offsetX_, -map_->offsetY_, 0.0);

  glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(map_->sizeX_, 0.0, 0.0);
  glEnd();

	glBegin(GL_LINES);
		glVertex3d(map_->sizeX_, 0.0, 0.0);
		glVertex3d(map_->sizeX_, map_->sizeY_, 0.0);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(map_->sizeX_, map_->sizeY_, 0.0);
		glVertex3d(0.0, map_->sizeY_, 0.0);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(0.0, map_->sizeY_, 0.0);
		glVertex3d(0.0, 0.0, 0.0);
	glEnd();

//  for (int i = 0; i <= map_->sizeX_; ++i)
//  {
//    glBegin(GL_LINES);
//      glVertex3d(i, 0           , 0.0);
//      glVertex3d(i, map_->sizeY_, 0.0);
//    glEnd();
//  }
//
//  for (int j = 0; j <= map_->sizeY_; ++j)
//  {
//    glBegin(GL_LINES);
//      glVertex3d(0,            j, 0.0);
//      glVertex3d(map_->sizeX_, j, 0.0);
//    glEnd();
//  }

  glPopMatrix();
}

void MapDrawer3D::drawAxes()
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

void MapDrawer3D::drawPVolumes(Robot robot)
{
  for (int i = 0; i < map_->sizeX_; ++i)
  for (int j = 0; j < map_->sizeY_; ++j)
  {
    Volume * volumes = map_->grid_[i][j].getPVolumes();
    int count = map_->grid_[i][j].getPVolumesCount();

    for (int v = 0; v < count; ++v)
    {
      double x = i - map_->offsetX_;
      double y = j - map_->offsetY_;

  		glPushMatrix();

      glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
			glTranslatef(x, y, 0.0);


      //if (gui_->getColorByHeight())
      //  drawHeightColorVolume(volume->getBot(), volume->getTop());
      //else
			if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > (cutoffBelow * maxHeight))
				drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_P_VOLUMES);

			glPopMatrix();
    }
  }
}

void MapDrawer3D::drawNVolumes(Robot robot)
{
  for (int i = 0; i < map_->sizeX_; ++i)
  for (int j = 0; j < map_->sizeY_; ++j)
  {
    Volume * volumes = map_->grid_[i][j].getNVolumes();
    int count = map_->grid_[i][j].getNVolumesCount();

    for (int v = 0; v < count; ++v)
    {
      double x = i - map_->offsetX_;
      double y = j - map_->offsetY_;

  		glPushMatrix();

      glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
			glTranslatef(x, y, 0.0);


      //if (gui_->getColorByHeight())
      //  drawHeightColorVolume(volume->getBot(), volume->getTop());
      //else
			if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > (cutoffBelow * maxHeight))
        drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_N_VOLUMES);

			glPopMatrix();
    }
  }
}

void MapDrawer3D::drawSolidColorVolume(double bottom, double top, const double color[3])
{
	glEnable(GL_POLYGON_OFFSET_FILL); // Avoid Stitching!
	glPolygonOffset(1.0, 1.0);

	double eps  =  0.5;
	double eps1 =  0.0;
	double eps2 =  1.0;

	// top and bottom

	glColor3dv(color);

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps1, bottom);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, top);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, top);
	glEnd();

	// left and right

	glColor3dv(color);

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps1, eps2, top);
		glVertex3f(eps1, eps1, top);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps2, eps1, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps2, eps1, top);
	glEnd();

	// front and back

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps1, bottom);
		glVertex3f(eps2, eps1, bottom);
		glVertex3f(eps2, eps1, top);
		glVertex3f(eps1, eps1, top);
	glEnd();

	glBegin(GL_POLYGON);
		glVertex3f(eps1, eps2, bottom);
		glVertex3f(eps2, eps2, bottom);
		glVertex3f(eps2, eps2, top);
		glVertex3f(eps1, eps2, top);
	glEnd();

	glDisable(GL_POLYGON_OFFSET_FILL);

//	// ******* LINES
//
//	glColor3f(0.1, 0.1, 0.1);
//  glLineWidth(0.3	);
//
//	glBegin(GL_LINE_LOOP);
//		glVertex3f(eps1, eps1, bottom);
//		glVertex3f(eps1, eps2, bottom);
//		glVertex3f(eps2, eps2, bottom);
//		glVertex3f(eps2, eps1, bottom);
//	glEnd();
//
//	glBegin(GL_LINE_LOOP);
//		glVertex3f(eps1, eps1, top);
//		glVertex3f(eps1, eps2, top);
//		glVertex3f(eps2, eps2, top);
//		glVertex3f(eps2, eps1, top);
//	glEnd();
//
//	glBegin(GL_LINES);
//		glVertex3f(eps1, eps1, bottom);
//		glVertex3f(eps1, eps1, top);
//		glVertex3f(eps1, eps2, bottom);
//		glVertex3f(eps1, eps2, top);
//		glVertex3f(eps2, eps2, bottom);
//		glVertex3f(eps2, eps2, top);
//		glVertex3f(eps2, eps1, bottom);
//		glVertex3f(eps2, eps1, top);
//	glEnd();

}

void MapDrawer3D::drawHeightColorVolume(double bottom, double top)
{
	double color[3];

	int scale = (maxHeight * map_->resolution_)/10;

  int itop;

  if(top >= scale * 9)
  	itop = 9;
  else if(top >= scale * 8)
  	itop = 8;
  else if(top >= scale * 7)
  	itop = 7;
  else if(top >= scale * 6)
    	itop = 6;
  else if(top >= scale * 5)
    	itop = 5;
  else if(top >= scale * 4)
    	itop = 4;
  else if(top >= scale * 3)
    	itop = 3;
  else if(top >= scale * 2)
    	itop = 2;
  else if(top >= scale * 1)
    	itop = 1;
  else if(top >= scale * 0)
    	itop = 0;
  else
  	itop = 0;

	color[0] = RGB_PALLET_COLOR_VOLUMES[itop][0];
	color[1] = RGB_PALLET_COLOR_VOLUMES[itop][1];
	color[2] = RGB_PALLET_COLOR_VOLUMES[itop][2];

	drawSolidColorVolume(bottom, top, color);
}

void MapDrawer3D::setView()
{
	//printf("setView(): %6.2lf, %6.2lf", gui_->getCanvasWidth(), gui_->getCanvasWidth());

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0,0, gui_->getCanvasWidth(), gui_->getCanvasHeight());

	gluPerspective(45.0, gui_->getCanvasWidth()/gui_->getCanvasHeight(), 0.1, 2000.0);

	// display camera view
	gluLookAt(camera_.getPosX(),  camera_.getPosY(),  camera_.getPosZ(),
				    camera_.getLookX(), camera_.getLookY(), camera_.getLookZ(),
				    0.0,               0.0,               1.0);
}

void MapDrawer3D::pan(double angle)
{
  camera_.pan(angle);
  setView();
}

void MapDrawer3D::tilt(double angle)
{
  camera_.tilt(angle);
  setView();
}

void MapDrawer3D::zoom(double r)
{
  camera_.zoom(r);
  setView();
}

void MapDrawer3D::move(double x, double y)
{
  camera_.move(x, y);
  setView();
}

void MapDrawer3D::setCutoffAbove(double p)
{
	this->cutoffAbove = p;
}

double MapDrawer3D::getCutoffAbove()
{
	return this->cutoffAbove;
} // namespace MVOG

void MapDrawer3D::setCutoffBelow(double p)
{
	this->cutoffBelow = p;
}

double MapDrawer3D::getCutoffBelow()
{
	return this->cutoffBelow;
}

void MapDrawer3D::setMaxHeight(double p)
{
	this->maxHeight = p / map_->resolution_;
}

double MapDrawer3D::getMaxHeight()
{
	return this->maxHeight * map_->resolution_;
}

}

/*
void MapDrawer3D::drawRobot(Robot robot)
{
	glLineWidth(10.0);

	glColor3d(1.0, 0.0, 0.0);

  glPushMatrix();
	//glScaled(map_->resolution_, map_->resolution_, map_->resolution_);
	//glTranslatef(robot.x, robot.y, 0.0);

	glBegin(GL_LINES);
		glVertex3d(robot.x, robot.y, robot.z);
		glVertex3d(robot.x+1.0, robot.y, robot.z);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(robot.x, robot.y, robot.z);
		glVertex3d(robot.x, robot.y+1.0, robot.z);
	glEnd();


	glBegin(GL_LINES);
		glVertex3d(robot.x, robot.y, robot.z);
		glVertex3d(robot.x, robot.y, robot.z+1.0);
	glEnd();

  glPopMatrix();

	glLineWidth(1.0);

}

void MapDrawer3D::drawGrid(Robot robot)
{
  glClearColor(1.0, 1.0, 1.0, 0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3dv(GRID_COLOR);

  glPushMatrix();
  glScaled(map_->resolution_, map_->resolution_, 1.0);

  glTranslated(-robot_offset_x, -robot_offset_y, 0.0);

  for (int i = 0; i <= 2 * robot_offset_x; ++i)
  {
    glBegin(GL_LINES);
      glVertex3d(i, 0, 0.0);
      glVertex3d(i, 2 * robot_offset_y, 0.0);
    glEnd();
  }

  for (int j = 0; j <= 2* robot_offset_y; ++j)
  {
    glBegin(GL_LINES);
      glVertex3d(0, j, 0.0);
      glVertex3d(2* robot_offset_x, j, 0.0);
    glEnd();
  }

  glPopMatrix();
}

void MapDrawer3D::drawMLolumes(Robot robot)
{
	//printf("x: %6.2f, y:%6.2f\n", robot.x, robot.y);

	int k,l;

	for (int i = (floor(robot.x)/map_->resolution_ + map_->offsetX_ - robot_offset_x); i < (floor(robot.x)/map_->resolution_ + map_->offsetX_ + robot_offset_x); ++i)
	{
		for (int j = (floor(robot.y)/map_->resolution_ + map_->offsetY_ - robot_offset_y); j <  (floor(robot.y)/map_->resolution_ + map_->offsetY_ + robot_offset_y); ++j)
		{
			MLVolumeVector vect;
			map_->grid_[i][j].createMLVolumes(vect);

			double x = i - map_->offsetX_;
			double y = j - map_->offsetY_;

			glPushMatrix();

			glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
			//glTranslatef((-robot_offset_x + k)*map_->resolution_, (-robot_offset_y + l)*map_->resolution_, 0.0);
			glTranslatef(x, y, 0.0);

			for (size_t c = 0; c < vect.size(); ++c)
			{
				if(vect[c].top < ((1 - cutoffAbove) * maxHeight) && vect[c].bot > ((cutoffBelow - 1) * maxHeight))
				{
					if(gui_->getColorByHeight())
							drawHeightColorVolume(vect[c].top,  vect[c].bot);
					else
							drawSolidColorVolume(vect[c].top, vect[c].bot, COLOR_ML_VOLUMES);
				}
			}

			glPopMatrix();
		}
	}
}


void MapDrawer3D::drawMLolumes(Robot robot)
{
	int x_offset = 50;
	int y_offset = 50;

//	glLineWidth(2.0);
//	glColor3dv(COLOR_RED);
//
//	glPushMatrix();
//		//glScaled(map_->resolution_, map_->resolution_, 1.0);
//		//glTranslatef(robot.x, robot.y, 0.0);
//		glBegin(GL_LINES);
//			glVertex3d(robot.x-x_offset, robot.y-y_offset, 0.0);
//			glVertex3d(robot.x+x_offset, robot.y-y_offset, 0.0);
//		glEnd();
//
//		glBegin(GL_LINES);
//		glVertex3d(robot.x+x_offset, robot.y-y_offset, 0.0);
//			glVertex3d(robot.x+x_offset, robot.y+y_offset, 0.0);
//		glEnd();
//
//		glBegin(GL_LINES);
//			glVertex3d(robot.x+x_offset, robot.y+y_offset, 0.0);
//			glVertex3d(robot.x-x_offset, robot.y+y_offset, 0.0);
//		glEnd();
//
//		glBegin(GL_LINES);
//			glVertex3d(robot.x-x_offset, robot.y+y_offset, 0.0);
//			glVertex3d(robot.x-x_offset, robot.y-y_offset, 0.0);
//		glEnd();
//	glPopMatrix();
//
//	glLineWidth(1.0);

	for (int i = 0; i < map_->sizeX_; ++i)
	for (int j = 0; j < map_->sizeY_; ++j)
	{
    MLVolumeVector vect;
    map_->grid_[i][j].createMLVolumes(vect);

    double x = i - map_->offsetX_;
    double y = j - map_->offsetY_;

		glPushMatrix();

    glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
		glTranslatef(x, y, 0.0);

    for (size_t c = 0; c < vect.size(); ++c)
    {
    	if(gui_->getColorByHeight())
    	{
    		if(vect[c].top < ((1 - cutoffAbove) * maxHeight) && vect[c].bot > ((cutoffBelow - 1) * maxHeight))
    			drawHeightColorVolume(vect[c].top,  vect[c].bot);
    	}
    	else
    	{
				if(vect[c].top < ((1 - cutoffAbove) * maxHeight) && vect[c].bot > ((cutoffBelow - 1) * maxHeight))
					drawSolidColorVolume(vect[c].top, vect[c].bot, COLOR_ML_VOLUMES);
    	}
    }

    glPopMatrix();
  }
}

void MapDrawer3D::drawAxes()
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

void MapDrawer3D::drawPVolumes(Robot robot)
{
  for (int i = (ceil(robot.x) - robot_offset_x); i <= (ceil(robot.x) + robot_offset_x); ++i)
  {
	  for (int j = (ceil(robot.y) - robot_offset_y); j <= (ceil(robot.y) - robot_offset_y); ++j)
	  {
		Volume * volumes = map_->grid_[i][j].getPVolumes();
		int count = map_->grid_[i][j].getPVolumesCount();

		for (int v = 0; v < count; ++v)
		{
		  double x = i - map_->offsetX_;
		  double y = j - map_->offsetY_;

			glPushMatrix();

		  glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
		  glTranslatef(x, y, 0.0);


		  if (gui_->getColorByHeight())
		  {
		  	if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > ((cutoffBelow - 1) * maxHeight))
		  		drawHeightColorVolume(getBot(volumes[v]), getTop(volumes[v]));
		  }
		  else
		  {
				if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > ((cutoffBelow - 1) * maxHeight))
					drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_P_VOLUMES);
		  }

				glPopMatrix();
		}
	  }
  }
}

void MapDrawer3D::drawNVolumes(Robot robot)
{
  for (int i = (ceil(robot.x) - robot_offset_x); i <= (ceil(robot.x) + robot_offset_x); ++i)
  {
	  for (int j = (ceil(robot.y) - robot_offset_y); j <= (ceil(robot.y) - robot_offset_y); ++j)
	  {
		Volume * volumes = map_->grid_[i][j].getNVolumes();
		int count = map_->grid_[i][j].getNVolumesCount();

		for (int v = 0; v < count; ++v)
		{
		  double x = i - map_->offsetX_;
		  double y = j - map_->offsetY_;

			glPushMatrix();

		  glScalef(map_->resolution_, map_->resolution_, map_->resolution_);
				glTranslatef(x, y, 0.0);


		  if (gui_->getColorByHeight())
		  {
		  	if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > (cutoffBelow * maxHeight))
		  		drawHeightColorVolume(getBot(volumes[v]), getTop(volumes[v]));
		  }
		  else
		  {
				if(getTop(volumes[v]) < ((1 - cutoffAbove) * maxHeight) && getBot(volumes[v]) > (cutoffBelow * maxHeight))
					drawSolidColorVolume(getBot(volumes[v]), getTop(volumes[v]), COLOR_N_VOLUMES);
		  }

				glPopMatrix();
		}
	  }
  }
}
*/
