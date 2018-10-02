#include <GL/glut.h>
#include <carmen/carmen.h>
#include "panel.h"
#include "car_panel.h"

car_panel *car_panel_gl = NULL;


void
display_gui(void)
{
  	car_panel_gl->draw();
  	carmen_ipc_sleep(0.01);
}


void
reshape_gui(int w, int h)
{
	car_panel_gl->set_view(w, h);
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);

	car_panel_gl = car_panel::get_instance(argc, argv);

	glutInit(&argc, argv);
	glutCreateWindow("CAR PANEL GUI");
	glutReshapeWindow(600, 300);
	glutReshapeFunc(reshape_gui);
	glutIdleFunc(display_gui);
	glutDisplayFunc(display_gui);
	glutKeyboardFunc(keypress);
	glutMainLoop();

	return (0);
}
