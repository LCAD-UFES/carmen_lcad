/*
 * pi_imu_interface.cpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Marcelo
 */
#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <carmen/xsenscore.h>
#include <GL/glut.h>
#include <math.h>

#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846
#define AA 0.97 // complementary filter constant

carmen_xsens_global_quat_message *xsens_quat_message;
xsens_global data;
float matCol[] = {1,0,0,0};

void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Imu Interface: disconnected.\n");
        exit(0);
    }
}


void
xsens_message_handler(carmen_xsens_global_quat_message *xsens_quat_message)
{
	data.acc.x = xsens_quat_message->m_acc.x;
	data.acc.y = xsens_quat_message->m_acc.y;
	data.acc.z = xsens_quat_message->m_acc.z;

	data.gyr.x = xsens_quat_message->m_gyr.x;
	data.gyr.y = xsens_quat_message->m_gyr.y;
	data.gyr.z = xsens_quat_message->m_gyr.z;

	data.mag.x = xsens_quat_message->m_mag.x;
	data.mag.y = xsens_quat_message->m_mag.y;
	data.mag.z = xsens_quat_message->m_mag.z;

	printf("ACELEROMETRO = X:%f m/s^2 Y:%f m/s^2 Z:%f m/s^2\n", data.acc.x, data.acc.y, data.acc.z);
	printf("GIROSCÓPIO = X:%f radps Y:%f radps Z:%f radps\n", data.gyr.x, data.gyr.y, data.gyr.z);
	printf("MAGNETOMETRO = X:%f mgauss Y:%f mgauss Z:%f mgauss\n", data.mag.x, data.mag.y, data.mag.z);
}


void
carmen_xsens_subscribe_xsens_global_quat_message(carmen_xsens_global_quat_message
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME,
    						 (char *) CARMEN_XSENS_GLOBAL_QUAT_FMT,
                             xsens_global, sizeof(carmen_xsens_global_quat_message),
            			     handler, subscribe_how);
}


void
carmen_xsens_unsubscribe_xsens_global_quat_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME, handler);
}


void
display (void)
{
	float AccYangle = 0.0;
	float AccXangle = 0.0;
	float AccZangle = 0.0;

	float CFangleX = 0.0;
	float CFangleY = 0.0;
	float CFangleZ = 0.0;

	//Convert Accelerometer values to degrees
	AccXangle = (float) (atan2(data.acc.y, data.acc.z)) * RAD_TO_DEG;
	AccYangle = (float) (atan2(data.acc.z, data.acc.x)) * RAD_TO_DEG;
	AccZangle = (float) (atan2(data.acc.x, data.acc.y)) * RAD_TO_DEG;

	if (AccXangle >180)
		AccXangle -= (float)360.0;
	AccYangle -= 90;
	if (AccYangle >180)
		AccYangle -= (float)360.0;

	//Complementary filter used to combine the accelerometer and gyro values.
	CFangleX=AA*(CFangleX + data.gyr.x) + (1 - AA) * AccXangle;
	CFangleY=AA*(CFangleY + data.gyr.y) + (1 - AA) * AccYangle;
	CFangleZ=AA*(CFangleZ + data.gyr.z) + (1 - AA) * AccZangle;

	//glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Clear the background of our window to red

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

		glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations

		glColor3f(0.0, 1.0, 0.0);

		glRotatef(CFangleX, 1.0f, 0.0f, 0.0f); // Rotate our object around the x axis

		glRotatef(CFangleY, 0.0f, 1.0f, 0.0f); // Rotate our object around the y axis

		glRotatef(CFangleZ, 0.0f, 0.0f, 1.0f); // Rotate our object around the z axis

		glBegin(GL_QUADS);

			glVertex3f(-0.5, 0.0, -0.5);

			glVertex3f (-0.5, 0.0, 0.5);

			glVertex3f(0.5, 0.0, 0.5);

			glVertex3f(0.5, 0.0, -0.5);

		glEnd();

	glPopMatrix();

	glFlush();

	glutSwapBuffers();
}

void
sleep_ipc()
{
	carmen_ipc_sleep(0.033333333);

	glutPostRedisplay();
}

void init(){
	glClearColor(0.0, 0.0, 0.0, 1.0);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);

	gluPerspective(60.0, 1.0, 1.0, 20.0);

	gluLookAt(7.0, 7.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	carmen_xsens_subscribe_xsens_global_quat_message(xsens_quat_message, (carmen_handler_t) xsens_message_handler, CARMEN_SUBSCRIBE_LATEST);

	glutInit(&argc, argv); // Initialize GLUT

	glutInitDisplayMode (GLUT_SINGLE); // Set up a basic display buffer (only single buffered for now)

	glutInitWindowSize (600, 500); // Set the width and height of the window

	glutInitWindowPosition (100, 100); // Set the position of the window

	glutCreateWindow ("Your first OpenGL Window"); // Set the title for the window

	glutDisplayFunc(display); // Tell GLUT to use the method "display" for rendering

	//glutReshapeFunc(reshape); // Tell GLUT to use the method "reshape" for reshaping

	glutIdleFunc(sleep_ipc); // Tell GLUT t#include <carmen/Window.h>

	init();

	glutMainLoop(); // Enter GLUT's main loop

	//carmen_ipc_dispatch();

	return (0);
}
