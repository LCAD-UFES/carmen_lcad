
#include <math.h>
#include <carmen/carmen.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <csetjmp>

#include "panel.h"

#include "accelerator.h"
#include "arrow.h"
#include "lights.h"
#include "speedometer.h"
#include "steering.h"
#include "without_time.h"


class CarPanel_INNER
{
	float FarNear;
	float angleSteering;
	float angleTireToSteering;

	Steering *steering;
	Arrow *arrowLeft;
	Arrow *arrowRight;
	Lights *lights;
	Speedometer *speedometer;
	Accelerator *accelerator;

	double _v, _phi, _time, _prev_time;

public:
	CarPanel_INNER();
	void update(double v, double phi, double time);
	void draw();
	void reshape(int w, int h);
	~CarPanel_INNER();
};


CarPanel_INNER::CarPanel_INNER()
{
	FarNear = 100.0f;
	angleSteering = 0.0;
	angleTireToSteering = 16.0;

	steering = new Steering();
	speedometer = new Speedometer();
	arrowRight = new Arrow();
	arrowLeft = new Arrow();
	lights = new Lights();
	accelerator = new withoutTime();

	_v = _phi = _time = _prev_time = 0.;
}


void
CarPanel_INNER::update(double v, double phi, double time)
{
	if (time != _prev_time)
	{
		_v = v;
		_phi = phi;
		_prev_time = _time;
		_time = time;
	}
}


void
CarPanel_INNER::draw()
{
	angleSteering = carmen_radians_to_degrees(_phi) * angleTireToSteering;
	speedometer->update(_v);
	accelerator->update(_v, _time);

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	glPushMatrix();

	/* Display lights */
	glTranslatef(-20.0f, 80.0f, 0.0f);
	lights->drawLight();
	glTranslatef(15.0f, 0.0f, 0.0f);
	glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
	lights->drawLight();
	glRotatef(-180.0f, 0.0f, 0.0f, 1.0f);
	glTranslatef(-15.0f, 0.0f, 0.0f);
	glTranslatef(20.0f, -80.0f, 0.0f);

	glTranslatef(-60.0f, 80.0f, 0.0f);
	lights->drawHighLight();
	glTranslatef(60.0f, -80.0f, 0.0f);

	glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
	glTranslatef(60.0f, 90.0f, 0.0f);
	arrowLeft->draw();
	glTranslatef(-60.0f, -90.0f, 0.0f);
	glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);

	glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);
	glTranslatef(-60.0f, 90.0f, 0.0f);
	arrowRight->draw();
	glTranslatef(60.0f, -90.0f, 0.0f);
	glRotatef(90.0f, 0.0f, 0.0f, 1.0f);

	/* Display Accelerometer */
	glTranslatef(45.0f, -13.0f, 0.0f);
	accelerator->draw();
	glTranslatef(-45.0f, 13.0f, 0.0f);

	glScalef(GLfloat(1 * 0.7), GLfloat(1 * 0.7), GLfloat(1 * 0.7));

	/* Display steering wheel */
	glTranslatef(190.0f, -45.0f, 0.0f);
	steering->draw(angleSteering);
	glTranslatef(-190.0f, 45.0f, 0.0f);

	/* Display speedometer */
	glTranslatef(-80.0f, -50.0f, 0.0f);
	speedometer->draw();
	glTranslatef(80.0f, 50.0f, 0.0f);

	glScalef(GLfloat(1 / 0.7), GLfloat(1 / 0.7), GLfloat(1 / 0.7));

	glPopMatrix();
	glFlush();
}


CarPanel_INNER::~CarPanel_INNER()
{
	delete steering;
	delete speedometer;
	delete arrowRight;
	delete arrowLeft;
	delete lights;
	delete accelerator;
}


void
CarPanel_INNER::reshape(GLsizei w, GLsizei h)
{
	GLfloat aspectRatio;

	if (h == 0)
		h = 1;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	aspectRatio = (GLfloat)w / (GLfloat)h;
	if (w <= h)
		glOrtho(-FarNear, FarNear, -FarNear / aspectRatio, FarNear / aspectRatio, 1.0, -1.0);
	else
		glOrtho(-FarNear * aspectRatio, FarNear * aspectRatio, -FarNear, FarNear, 1.0, -1.0);

	glMatrixMode(GL_MODELVIEW);
}


CarPanel_INNER *_panel = NULL;
std::jmp_buf jump_buffer;


void
_panel_display()
{
	_panel->draw();
}


void
_panel_reshape(int w, int h)
{
	_panel->reshape(w, h);
}


void
_panel_update(double v, double phi, double time)
{
	_panel->update(v, phi, time);
}


void
_panel_idle()
{
    glutPostRedisplay();
    longjmp(jump_buffer, 0);
}


void
_panel_init()
{
	_panel = new CarPanel_INNER();

	int argc = 1;
	char **argv = (char **) calloc (1, sizeof(char*));
	argv[0] = (char *) calloc (64, sizeof(char));
	strcpy(argv[0], "mypanel");

	glutInit(&argc, argv);
	glutCreateWindow("CAR PANEL GUI");
	glutReshapeWindow(600, 300);
	glutReshapeFunc(_panel_reshape);
	glutIdleFunc(_panel_idle);
	glutDisplayFunc(_panel_display);
}


void
_panel_draw_step()
{
    if(setjmp(jump_buffer) == 0)
        glutMainLoop();
}


void
_panel_destroy()
{
	delete _panel;
}


CarPanel::CarPanel()
{
	_panel_init();

}


void
CarPanel::draw(double v, double phi, double time)
{
	_panel_update(v, phi, time);
	_panel_draw_step();
}


CarPanel::~CarPanel()
{
	_panel_destroy();
}


