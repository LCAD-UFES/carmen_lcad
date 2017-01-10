#include <GL/glut.h>
#include <carmen/carmen.h>
#include <carmen/polar_point.h>
#include <carmen/moving_objects3_interface.h>

#include "moving_objects3_utils.h"

int NUM_SPHERES = 12; // TODO: ler do param daemon
const int REDRAW_UPDATE_PERIOD = 40;
const int MAP_VIEWER_SIZE = 600; // pixels
const int ONE_DIMENSIONAL_PLOT_WIDTH = 900; // pixels
const int ONE_DIMENSIONAL_PLOT_HEIGHT = 200; // pixels

const int show_velodyne_on_ground_window = 1;
int show_laser_rays = 0;
const int show_one_dimension_window = 0;

double pixels_per_meter_x;
double pixels_per_meter_y;

int velodyne_on_ground_window_id;
int one_dimension_window_id;

carmen_velodyne_projected_on_ground_message velodyne_on_ground_message;
carmen_moving_objects3_particles_message particles_message;
carmen_moving_objects3_virtual_scan_message virtual_scan_message;

int num_of_rays = 0;
double *current_virtual_scan;
double *last_virtual_scan;

double rotate_y=0;
double rotate_x=0;
double zoom=1.0;

void
draw_circle(double radius, double x_center, double y_center, double r, double g, double b)
{
	double angle, angle_rad;

	glBegin(GL_LINE_LOOP);
	glColor3f(r, g, b);

	for (angle = 0; angle < 360; angle += 5)
	{
		angle_rad = carmen_normalize_theta(carmen_degrees_to_radians(angle));
		glVertex2f((x_center + sin(angle_rad) * radius) * pixels_per_meter_x, (y_center + cos(angle_rad) * radius) * pixels_per_meter_y);
	}

	glEnd();
}


void
draw_car_centralized()
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0, 0.0, 1.0);

	glVertex2f(-2.2 * pixels_per_meter_x, -0.8 * pixels_per_meter_y);
	glVertex2f(2.2 * pixels_per_meter_x, -0.8 * pixels_per_meter_y);
	glVertex2f(2.2 * pixels_per_meter_x, 0.8 * pixels_per_meter_y);
	glVertex2f(-2.2 * pixels_per_meter_x, 0.8 * pixels_per_meter_y);

	glEnd();
}


void
draw_spheres()
{
	for (int i = 0; i < NUM_SPHERES; i++)
	{
		draw_circle(5*(i+1), 0, 0, 0.88, 0.88, 0.88);
	}
}


void
draw_particles(carmen_moving_objects3_particles_message particles_message)
{
	for (int i = 0; i < particles_message.num_particles; i++)
	{
		rectangle_points rect[3];

		generate_rectangles_points(particles_message.particles[i].pose,
				particles_message.particles[i].geometry.width,
				particles_message.particles[i].geometry.length,
				rect, rect+1, rect+2, 0.25);

		// draw inside car rectangle
		glBegin(GL_LINE_LOOP);
		glColor3f(0.0, 0.0, 0.9);

		glVertex2f(rect[0].p1.x * pixels_per_meter_x, rect[0].p1.y * pixels_per_meter_y);
		glVertex2f(rect[0].p2.x * pixels_per_meter_x, rect[0].p2.y * pixels_per_meter_y);
		glVertex2f(rect[0].p3.x * pixels_per_meter_x, rect[0].p3.y * pixels_per_meter_y);
		glVertex2f(rect[0].p4.x * pixels_per_meter_x, rect[0].p4.y * pixels_per_meter_y);

		glEnd();


		// draw car surface rectangle
		glBegin(GL_LINE_LOOP);
		glColor3f(0.0, 1.0, 0.0);

		glVertex2f(rect[1].p1.x * pixels_per_meter_x, rect[1].p1.y * pixels_per_meter_y);
		glVertex2f(rect[1].p2.x * pixels_per_meter_x, rect[1].p2.y * pixels_per_meter_y);
		glVertex2f(rect[1].p3.x * pixels_per_meter_x, rect[1].p3.y * pixels_per_meter_y);
		glVertex2f(rect[1].p4.x * pixels_per_meter_x, rect[1].p4.y * pixels_per_meter_y);

		glEnd();

		// draw car surface rectangle
		glBegin(GL_LINE_LOOP);
		glColor3f(1.0, 0.0, 0.0);

		glVertex2f(rect[2].p3.x * pixels_per_meter_x, rect[2].p3.y * pixels_per_meter_y);
		glVertex2f(rect[2].p2.x * pixels_per_meter_x, rect[2].p2.y * pixels_per_meter_y);
		glVertex2f(rect[2].p1.x * pixels_per_meter_x, rect[2].p1.y * pixels_per_meter_y);
		glVertex2f(rect[2].p4.x * pixels_per_meter_x, rect[2].p4.y * pixels_per_meter_y);

		glEnd();
	}
}


void
draw_virtual_scan(double *current_virtual_scan, double *last_virtual_scan, int num_of_rays)
{
	int i;
	double x;
	double y;

	double r = 0.0, g = 0.0, b  = 0.0;

	double virtual_scan_resolution = (2*M_PI)/num_of_rays;

	if(show_laser_rays)
	{
		for(i = 0; i < num_of_rays; i++)
		{
			// compute angle
			double angle = (((double) i) * virtual_scan_resolution) - M_PI;
			transform_polar_coordinates_to_cartesian_coordinates(current_virtual_scan[i], angle, &x, &y);

			double pixel_x = (x * pixels_per_meter_x);
			double pixel_y = (y * pixels_per_meter_y);

			// quanto maior a possibilidade de ser obstáculo mais preto é o ponto
			double red = 0.88;
			double green = 0.88;
			double blue = 0.0;

			glBegin(GL_LINES);
			glColor3f(red, green, blue);
			glVertex2f(0.0, 0.0);
			glVertex2f(pixel_x, pixel_y);
			glEnd();
		}
	}

	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (i = 0; i < num_of_rays; i++)
	{
		// compute angle
		double angle = (((double) i) * virtual_scan_resolution) - M_PI;
		transform_polar_coordinates_to_cartesian_coordinates(current_virtual_scan[i], angle, &x, &y);

		double pixel_x = (x * pixels_per_meter_x);
		double pixel_y = (y * pixels_per_meter_y);

		if ((last_virtual_scan[i] == 0.0 || last_virtual_scan[i] == 50.0) &&
				(current_virtual_scan[i] != 0 || current_virtual_scan[i] != 50.0))
		{
			r = 1.0;
			g = 0.0;
			b = 0.0;
		}

		if ( current_virtual_scan[i] > last_virtual_scan[i] )
		{
			r = 1.0;
			g = 1.0;
			b = 1.0;

			transform_polar_coordinates_to_cartesian_coordinates(last_virtual_scan[i], angle, &x, &y);
			double lp_x = (x * pixels_per_meter_x);
			double lp_y = (y * pixels_per_meter_y);

			glColor3f(0.0, 1.0, 0.0);
			glVertex2f(lp_x, lp_y);
		}

		glColor3f(r, g, b);
		glVertex2f(pixel_x, pixel_y);
	}
	glEnd();
}


void
draw_velodyne_on_ground()
{
//	draw_spheres();
	draw_circle(50.0, 0, 0, 0, 0, 0);
	//draw_virtual_scan(last_virtual_scan, num_of_rays);
	draw_virtual_scan(current_virtual_scan, last_virtual_scan, num_of_rays);
	draw_particles(particles_message);
	draw_car_centralized();
}


void
handle_velodyne_on_ground_display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);

	glRotatef( rotate_x, 1.0, 0.0, 0.0 );
	glRotatef( rotate_y, 0.0, 1.0, 0.0 );

	glScalef(zoom, zoom, 1.0f);

	draw_velodyne_on_ground();

	glutSwapBuffers();
}


void
handle_velodyne_on_ground_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) -MAP_VIEWER_SIZE / 2),
		((double) MAP_VIEWER_SIZE / 2),
		((double) -MAP_VIEWER_SIZE / 2),
		((double) MAP_VIEWER_SIZE / 2)
	);
}


void
handle_one_dimension_view_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) -ONE_DIMENSIONAL_PLOT_WIDTH / 2),
		((double) ONE_DIMENSIONAL_PLOT_WIDTH / 2),
		((double) 0),
		((double) ONE_DIMENSIONAL_PLOT_HEIGHT + 20)
	);
}


void
write_text(double x, double y, char *text)
{
	uint i;

	glPushMatrix();
	glTranslatef(x - 10.0, y, 0.0);
	glScalef(0.05, 0.1, 0.7);

	for (i = 0; i < strlen(text); i++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i]);

	glPopMatrix();

}


void
handle_one_dimension_view_display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT);

	double jmp;

	jmp = ONE_DIMENSIONAL_PLOT_HEIGHT;

	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(-ONE_DIMENSIONAL_PLOT_WIDTH / 2, jmp);
	glVertex2f(ONE_DIMENSIONAL_PLOT_WIDTH / 2, jmp);
	glVertex2f(-ONE_DIMENSIONAL_PLOT_WIDTH / 2, 2 * jmp);
	glVertex2f(ONE_DIMENSIONAL_PLOT_WIDTH / 2, 2 * jmp);
	glEnd();

	glutSwapBuffers();
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);

	if (show_velodyne_on_ground_window)
		glutPostWindowRedisplay(velodyne_on_ground_window_id);

	if (show_one_dimension_window)
		glutPostWindowRedisplay(one_dimension_window_id);

	glutTimerFunc(REDRAW_UPDATE_PERIOD, Timer, 1);
}


void
keyboard_handler(unsigned char key, int x, int y)
{

	(void) x;
	(void) y;

	if(key == 'l')
	{
		show_laser_rays = show_laser_rays ? 0 : 1;
	}

	if(key == '-')
	{
		if(zoom > 0)
			zoom -= 0.25;
	}

	if(key == '=')
	{
		zoom += 0.25;
	}
}


void
handle_mouse( int button, int state, int x, int y)
{
	(void) x;
	(void) y;
	(void) state;

	if (button == 3)
	{
		zoom += 0.0625;
	}
	else if (button == 4)
	{
		if(zoom > 0)
			zoom -= 0.0625;
	}

    glutPostRedisplay();
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	// VELODYNE ON GROUND
	if (show_velodyne_on_ground_window)
	{
		glutInitWindowSize(MAP_VIEWER_SIZE, MAP_VIEWER_SIZE);
		glutInitWindowPosition(0, 0);
		velodyne_on_ground_window_id = glutCreateWindow("Virtual scan");
		glutReshapeFunc(handle_velodyne_on_ground_resize);
		glClearColor(0.26f, 0.26f, 0.26f, 1.0f);
		glutDisplayFunc(handle_velodyne_on_ground_display);
		glutMouseFunc(handle_mouse);
		glutKeyboardFunc(keyboard_handler);
	}

	// VELODYNE 1D PLOT
	if (show_one_dimension_window)
	{
		glutInitWindowSize(ONE_DIMENSIONAL_PLOT_WIDTH, ONE_DIMENSIONAL_PLOT_HEIGHT);
		glutInitWindowPosition(0, 0);
		one_dimension_window_id = glutCreateWindow("One dimesion plot");
		glutReshapeFunc(handle_one_dimension_view_resize);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glutDisplayFunc(handle_one_dimension_view_display);
		glutKeyboardFunc(keyboard_handler);
	}

	glutTimerFunc(REDRAW_UPDATE_PERIOD, Timer, 1);
}


void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("Moving Objects 3 Viewer: disconnected.\n");
		exit(0);
	}
}


int
read_parameters(int argc __attribute__((unused)), char **argv __attribute__((unused)))
{
	carmen_param_t polar_slam_param_list[] =
	{
		{(char *) "polar_slam", (char *) "num_spheres", CARMEN_PARAM_INT, &(NUM_SPHERES), 0, NULL},
	};

	carmen_param_install_params(argc, argv, polar_slam_param_list, sizeof(polar_slam_param_list) / sizeof(polar_slam_param_list[0]));
	return 0;
}


void
initialize_ipc(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
}


void
moving_objects3_particles_message_handler()
{

}


void
moving_objects3_virtual_scan_handler()
{
	if (num_of_rays != virtual_scan_message.num_rays)
	{
		num_of_rays = virtual_scan_message.num_rays;
		last_virtual_scan = (double *) realloc(last_virtual_scan, num_of_rays * sizeof(double));
		current_virtual_scan = (double *) realloc(current_virtual_scan, num_of_rays * sizeof(double));
	}

	memcpy(last_virtual_scan, current_virtual_scan, num_of_rays * sizeof(double));
	memcpy(current_virtual_scan, virtual_scan_message.virtual_scan, num_of_rays * sizeof(double));

}


void
subcribe_messages()
{
	carmen_subscribe_moving_objects3_particles_message(
		&particles_message,
		(carmen_handler_t) moving_objects3_particles_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_moving_objects3_virtual_scan_message(
		&virtual_scan_message,
		(carmen_handler_t) moving_objects3_virtual_scan_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_module(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	pixels_per_meter_x = ((double) MAP_VIEWER_SIZE) / (100);
	pixels_per_meter_y = ((double) MAP_VIEWER_SIZE) / (100);
}


int
main(int argc, char **argv)
{
	initialize_ipc(argc, argv);
//	read_parameters(argc, argv);
	initialize_module(argc, argv);
	initialize_viewer(argc, argv);
	subcribe_messages();
	glutMainLoop();

	return 0;
}

