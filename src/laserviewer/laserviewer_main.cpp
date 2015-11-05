#include <carmen/carmen.h>
#include <carmen/laser_interface.h>
#include <GL/glut.h>

const int redraw_update_period = 30;

float laser_max_range = 0;
int laser_viewer_zoom = 0;

int laser_id;
int laser_window_id;
int first_laser_message = 1;

carmen_laser_laser_message laser_message;

int num_messages_per_second = 0;
double time_last_report = 0;

void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("laserviewer: disconnected.\n");
		exit(0);
	}
}


void
initialize_ipc(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
}


void
carmen_laser_message_handler()
{
	num_messages_per_second++;

	if (time(NULL) - time_last_report > 1)
	{
		time_last_report = time(NULL);
		printf("VIEWER%d Num messages per second: %d\n", laser_id, num_messages_per_second);
		num_messages_per_second = 0;
	}
}


void
subcribe_messages()
{
	if (laser_id == 1)
	{
		carmen_laser_subscribe_laser1_message(&laser_message,
			(carmen_handler_t) carmen_laser_message_handler,
			CARMEN_SUBSCRIBE_ALL);
	}

	if (laser_id == 2)
	{
		carmen_laser_subscribe_laser2_message(&laser_message,
			(carmen_handler_t)carmen_laser_message_handler,
			CARMEN_SUBSCRIBE_ALL);
	}

	if (laser_id == 3)
	{
		carmen_laser_subscribe_laser3_message(&laser_message,
			(carmen_handler_t)carmen_laser_message_handler,
			CARMEN_SUBSCRIBE_ALL);
	}

	if (laser_id == 4)
	{
		carmen_laser_subscribe_laser4_message(&laser_message,
		(carmen_handler_t)carmen_laser_message_handler,
		CARMEN_SUBSCRIBE_ALL);
	}
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	glutPostWindowRedisplay(laser_window_id);
	glutTimerFunc(redraw_update_period, Timer, 1);
}


void
draw_laser_message()
{
	// glBegin(GL_POINTS);
	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 0.0);

	int i;
	int first = 1;
	double x;
	double y;
	double angle;
	double angle_rad;
	double range;
	double last_x;
	double last_y;

	angle = -90;

	for (i = 0; i < laser_message.num_readings; i++)
	{
		angle += 0.5;
		range = laser_message.range[i];

		angle_rad = carmen_degrees_to_radians(angle);

		if (range > laser_max_range)
			range = laser_max_range;

		last_x = x;
		last_y = y;

		x = range * sin(angle_rad) + laser_max_range;
		y = range * cos(angle_rad);

		if (!first)
		{
			glVertex2i(laser_viewer_zoom * last_x + laser_viewer_zoom / 2, laser_viewer_zoom * last_y  + laser_viewer_zoom / 2);
			glVertex2i(laser_viewer_zoom * x + laser_viewer_zoom / 2, laser_viewer_zoom * y  + laser_viewer_zoom / 2);
		}
		else
			first = 0;

	}

	glEnd();
}


void
draw_laser()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
	draw_laser_message();

	glutSwapBuffers();
}


void
handle_laser_viewer_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) 0),
		((double) 2 * laser_max_range * laser_viewer_zoom),
		((double) 0),
		((double) laser_max_range * laser_viewer_zoom)
	);
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	char window_name[128];
	sprintf(window_name, "LASER Viewer %d", laser_id);

	// LASER VIEWER WINDOW
	glutInitWindowSize(2 * laser_max_range * laser_viewer_zoom, laser_max_range * laser_viewer_zoom);
	glutInitWindowPosition(0, 0);
	laser_window_id = glutCreateWindow(window_name);
	glutReshapeFunc(handle_laser_viewer_resize);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glutDisplayFunc(draw_laser);

	glutTimerFunc(redraw_update_period, Timer, 1);
}


int
main(int argc, char **argv)
{
	if (argc < 4)
		exit(printf("Use %s <laser-id> <max-range> <zoom>\n", argv[0]));

	laser_id = atoi(argv[1]);
	laser_max_range = atof(argv[2]);
	laser_viewer_zoom = atof(argv[3]);

	initialize_ipc(argc, argv);
	initialize_viewer(argc, argv);
	subcribe_messages();
	glutMainLoop();

	return 0;
}

