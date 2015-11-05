#include <carmen/carmen.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <GL/glut.h>

const int redraw_update_period = 30;

float ultrasonic_max_range = 0;
int ultrasonic_viewer_zoom = 1;

int ultrasonic_window_id;
int first_ultrasonic_message = 1;

carmen_ultrasonic_sonar_sensor_message ultrasonic_message;

int num_messages_per_second = 0;
double time_last_report = 0;

void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("ultrasonic_viewer: disconnected.\n");
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
carmen_ultrasonic_message_handler()
{
	num_messages_per_second++;
	printf("%lf\n",ultrasonic_message.sensor[0]);

	if (time(NULL) - time_last_report > 1)
	{
		time_last_report = time(NULL);
		printf("ULTRASONIC_VIEWER Num messages per second: %d\n", num_messages_per_second);
		num_messages_per_second = 0;
	}
}


void
subcribe_messages()
{
	carmen_ultrasonic_sonar_sensor_subscribe(&ultrasonic_message,
		(carmen_handler_t) carmen_ultrasonic_message_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	glutPostWindowRedisplay(ultrasonic_window_id);
	glutTimerFunc(redraw_update_period, Timer, 1);
}


void
draw_ultrasonic_message()
{
	// DRAW CAR
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(-1, -2);
	glVertex2f(-1, 2);
	glVertex2f(1, 2);
	glVertex2f(1, -2);
	glEnd();

	// DRAW SENSOR DATA
	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);

	// L1 - Traseiro
	glVertex2f(0.0, -2.0);
	glVertex2f(0.0, -2.0 - ultrasonic_message.sensor[0]);

	// L2 - Lateral Tras
	glVertex2f(1.0, -1.5);
	glVertex2f(1.0 + ultrasonic_message.sensor[1], -1.5);

	// R2 - Lateral Frente
	glVertex2f(1.0, 1.5);
	glVertex2f(1.0 + ultrasonic_message.sensor[2], 1.5);

	// R1 - Frontal
	glVertex2f(0, 2.0);
	glVertex2f(0, 2.0 + ultrasonic_message.sensor[3]);

	glEnd();
}


void
draw_ultrasonic()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
	draw_ultrasonic_message();

	glutSwapBuffers();
}


void
handle_ultrasonic_viewer_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) -10),
		((double) 10),
		((double) -10),
		((double) 10)
	);
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	char window_name[128];
	sprintf(window_name, "Ultrasonic Viewer");

	// ULTRASONIC VIEWER WINDOW
	glutInitWindowSize(400, 400);
	glutInitWindowPosition(0, 0);
	ultrasonic_window_id = glutCreateWindow(window_name);
	glutReshapeFunc(handle_ultrasonic_viewer_resize);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glutDisplayFunc(draw_ultrasonic);

	glutTimerFunc(redraw_update_period, Timer, 1);
}


int
main(int argc, char **argv)
{
	if (argc < 3)
		exit(printf("Use %s <max-range> <zoom>\n", argv[0]));

	ultrasonic_max_range = atof(argv[1]);
	ultrasonic_viewer_zoom = atof(argv[2]);

	ultrasonic_message.sensor[0] = 0;
	ultrasonic_message.sensor[1] = 0;
	ultrasonic_message.sensor[2] = 0;
	ultrasonic_message.sensor[3] = 0;

	initialize_ipc(argc, argv);
	initialize_viewer(argc, argv);
	subcribe_messages();
	glutMainLoop();

	return 0;
}

