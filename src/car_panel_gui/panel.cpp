#include "panel.h"


void 
displayLights(void)
{
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
}


void 
displaySteering(void)
{	
	steering->draw(angleSteering);
}


void 
displaySpeedometer(void)
{
	speedometer->draw();
}


void
displayAccelerator(void)
{
	accelerator->draw();
}


void 
display(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	glPushMatrix();

	displayLights();

	glTranslatef(45.0f, -13.0f, 0.0f);
	displayAccelerator();
	glTranslatef(-45.0f, 13.0f, 0.0f);

	glScalef(GLfloat(1 * 0.7), GLfloat(1 * 0.7), GLfloat(1 * 0.7));

	glTranslatef(190.0f, -45.0f, 0.0f);
	displaySteering();
	glTranslatef(-190.0f, 45.0f, 0.0f);

	glTranslatef(-80.0f, -50.0f, 0.0f);
	displaySpeedometer();
	glTranslatef(80.0f, 50.0f, 0.0f);

	glScalef(GLfloat(1 / 0.7), GLfloat(1 / 0.7), GLfloat(1 / 0.7));

	glPopMatrix();
	glFlush();
}


void 
keypress(unsigned char key, int x __attribute__ ((unused)), int y __attribute__ ((unused)))
{
	switch (key)
	{
		case 'l':
			arrowLeft->blink();
			if (arrowRight->getIsFire())
				arrowRight->blink();
			break;
		case 'r':
			arrowRight->blink();
			if (arrowLeft->getIsFire())
				arrowLeft->blink();
			break;
		case 'h':
			lights->setHighLight(lights->getHighLight());
			break;
		case 't':
			lights->setLight(lights->getLight());
			break;
	}
}


void
reshape(GLsizei w, GLsizei h)
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


void
fused_dometry_handler(carmen_fused_odometry_message *fused_odometry)
{
	if (handler_message == fused_odometry_t)
	{
		angleSteering = carmen_radians_to_degrees(fused_odometry->phi) * angleTireToSteering;
		speedometer->update(fused_odometry->velocity.x);
		accelerator->update(fused_odometry->velocity.x, fused_odometry->timestamp);
	}
}


void
robot_ackerman_motion_command_handler(carmen_robot_ackerman_motion_command_message *motion_command)
{
	if (handler_message == robot_ackerman_motion_command_t && motion_command->num_motion_commands > 0)
	{
		angleSteering = carmen_radians_to_degrees(motion_command->motion_command[0].phi) * angleTireToSteering;
		speedometer->update(motion_command->motion_command[0].v);
		accelerator->update(motion_command->motion_command[0].v, motion_command->motion_command[0].time);
	}
}


void
base_ackerman_motion_command_handler(carmen_robot_ackerman_motion_command_message *motion_command)
{
	if (handler_message == base_ackerman_motion_command_t && motion_command->num_motion_commands > 0)
	{
		angleSteering = carmen_radians_to_degrees(motion_command->motion_command[0].phi) * angleTireToSteering;
		speedometer->update(motion_command->motion_command[0].v);
		accelerator->update(motion_command->motion_command[0].v, motion_command->motion_command[0].time);
	}
}


void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *base_ackerman_odometry)
{
	if (handler_message == base_ackerman_odometry_t)
	{
		angleSteering = carmen_radians_to_degrees(base_ackerman_odometry->phi) * angleTireToSteering;
		speedometer->update(base_ackerman_odometry->v);
		accelerator->update(base_ackerman_odometry->v, base_ackerman_odometry->timestamp);
	}
}


void
localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *localize_ackerman_globalpos)
{
	if (handler_message == localize_ackerman_globalpos_t)
	{
		angleSteering = carmen_radians_to_degrees(localize_ackerman_globalpos->phi) * angleTireToSteering;
		speedometer->update(localize_ackerman_globalpos->v);
		accelerator->update(localize_ackerman_globalpos->v, localize_ackerman_globalpos->timestamp);
	}
}


void
subscribe_messages(int msg_type, double interval)
{
	switch (msg_type)
	{
		case 0:
			handler_message = fused_odometry_t;
			accelerator = new withoutTime(interval);
			carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t)fused_dometry_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		case 1:
			handler_message = robot_ackerman_motion_command_t;
			accelerator = new withTime(interval);
			carmen_robot_ackerman_subscribe_motion_command(NULL, (carmen_handler_t)robot_ackerman_motion_command_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		case 2:
			handler_message = base_ackerman_motion_command_t;
			accelerator = new withTime(interval);
			carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t)base_ackerman_motion_command_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		case 3:
			handler_message = base_ackerman_odometry_t;
			accelerator = new withoutTime(interval);
			carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t)base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		case 4:
			handler_message = localize_ackerman_globalpos_t;
			accelerator = new withoutTime(interval);
			carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t)localize_ackerman_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
	}
}


void
idle(void)
{
	IPC_listen(1);

	glutPostRedisplay();
}


void
initializeComponents(void)
{
	steering = new Steering();
	speedometer = new Speedometer();
	arrowRight = new Arrow();
	arrowLeft = new Arrow();
	lights = new Lights();
}


void
setTypeMessage(int type_message)
{
	subscribe_messages(type_message, interval);
}


int
checkArguments(int argc, char *argv[])
{
	int i;
	int exist_msg = 0;

	if (argc > 1)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);

		initializeComponents();
		for (i = 1; i < argc; i++)
		{
			if (strcmp(argv[i], "-interval_msg") == 0 && i < argc - 1 && argv[i + 1][0] != '-')
				interval = atof(argv[i] + 1);
			if (strcmp(argv[i], "-type_msg") == 0 && i < argc - 1 && argv[i + 1][0] != '-')
			{
				exist_msg = 1;
				subscribe_messages(atoi(argv[i + 1]), interval);
			}
		}
	}

	return (exist_msg);
}


void
freeComponents(void)
{
	delete steering;
	delete speedometer;
	delete arrowRight;
	delete arrowLeft;
	delete lights;
	delete accelerator;
}
