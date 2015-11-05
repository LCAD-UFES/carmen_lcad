#include <GL/glut.h>
#include "car_panel.h"

car_panel *car_panel_gl = NULL;


void
display_gui(void)
{
  	car_panel_gl->draw();
}


void
reshape_gui(int w, int h)
{
	car_panel_gl->set_view(w, h);
}


int
main(int argc, char *argv[])
{

	car_panel_gl = car_panel::get_instance(argc, argv);

	glutInit(&argc, argv);
	glutCreateWindow("CAR PANEL GUI");
	glutReshapeFunc(reshape_gui);
	glutIdleFunc(display_gui);
	glutDisplayFunc(display_gui);
	glutMainLoop();

	return 0;
}