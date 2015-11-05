#include <mvog_gtk_gui/map_drawer_2d.h>

namespace MVOG
{

MapDrawer2D::MapDrawer2D(GTKGui * gui)
{
	gui_ = gui;
	drawProbability = true;
	visSize = 1.0;
}

MapDrawer2D::~MapDrawer2D()
{

}

void MapDrawer2D::draw()
{
  //setView();
  drawGrid();

  if(drawProbability)
  	drawProbabilities(visSize);
}

void MapDrawer2D::setView()
{
	 glMatrixMode(GL_PROJECTION);
	 glLoadIdentity();
	 glOrtho(0, gui_->getCanvasWidth(), gui_->getCanvasHeight(), 0, 0, 1);
	 //glMatrixMode(GL_MODELVIEW);
	 //glLoadIdentity();

	 //glTranslatef(0.375, 0.375, 0);

	 //glPushAttrib( GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT );
	 //glDisable( GL_DEPTH_TEST );
   //glDisable( GL_LIGHTING );



}

void MapDrawer2D::drawGrid()
{
	glClearColor(0.5, 0.5, 0.5, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

void MapDrawer2D::drawProbabilities(float z)
{
	float occ_prob = 1.0;

	for (int i = 0; i < map_->sizeX_; i++)
	{
	  for (int j = 0; j < map_->sizeY_; j++)
	  {
			if(map_->getCellStatic(i, (map_->sizeY_ - 1) - j)->getOccDensity(z, occ_prob))
			{
					//sprintf("occ_prob: %6.2f\n", occ_prob);
				 glBegin(GL_POINTS);
					 glColor3f(1.0 - occ_prob, 1.0 - occ_prob, 1.0 - occ_prob);
					 glVertex2f(i,j);
				 glEnd();
			}
			else
			{
				glBegin(GL_POINTS);
				 glColor3f(0.5, 0.5, 0.5);
				 glVertex2f(i,j);
			  glEnd();
			}
	  }
	}
}


} // namespace MVOG
