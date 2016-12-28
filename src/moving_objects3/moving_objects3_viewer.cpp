#include <GL/glut.h>
#include <carmen/carmen.h>
#include <carmen/polar_point.h>
#include <carmen/moving_objects3_interface.h>

#include "moving_objects3_utils.h"

int NUM_SPHERES = 6; // TODO: ler do param daemon
const int REDRAW_UPDATE_PERIOD = 40;
const int MAP_VIEWER_SIZE = 720; // pixels
const int NUM_POINTS_PER_VELODYNE = 36000;
const int ONE_DIMENSIONAL_PLOT_WIDTH = 900; // pixels
const int ONE_DIMENSIONAL_PLOT_HEIGHT = 200; // pixels

const int show_velodyne_on_ground_window = 1;
int show_laser_rays = 0;
const int show_one_dimension_window = 1;

double pixels_per_meter_x;
double pixels_per_meter_y;

int velodyne_on_ground_window_id;
int one_dimension_window_id;

carmen_velodyne_projected_on_ground_message velodyne_on_ground_message;
carmen_moving_objects3_particles_message particles_message;

double last_velodyne_on_ground_angles[NUM_POINTS_PER_VELODYNE];
double last_velodyne_on_ground_ranges[NUM_POINTS_PER_VELODYNE];
double last_velodyne_on_ground_intensities[NUM_POINTS_PER_VELODYNE];

double current_velodyne_on_ground_angles[NUM_POINTS_PER_VELODYNE];
double current_velodyne_on_ground_ranges[NUM_POINTS_PER_VELODYNE];
double current_velodyne_on_ground_intensities[NUM_POINTS_PER_VELODYNE];


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
		draw_circle(10*(i+1), 0, 0, 0.88, 0.88, 0.88);
	}
}


void
draw_velodyne_points(double *ranges, double *angles, double *intensities, int num_points)
{
	int i;
	double x;
	double y;

	if ((ranges == NULL) || (angles == NULL))
		return;

	glBegin(GL_POINTS);

	for(i = 0; i < num_points; i++)
	{
		transform_polar_coordinates_to_cartesian_coordinates(ranges[i], angles[i], &x, &y);

		double pixel_x = (x * pixels_per_meter_x);
		double pixel_y = (y * pixels_per_meter_y);


		// quanto maior a possibilidade de ser obstáculo mais preto é o ponto
		double red = 1.0 - intensities[i];
		double green = 1.0 - intensities[i];
		double blue = 1.0 - intensities[i];

		glColor3f(red, green, blue);
		glVertex2f(pixel_x, pixel_y);
	}

	glEnd();

	if(show_laser_rays)
	{
		for(i = 0; i < num_points; i++)
		{
			transform_polar_coordinates_to_cartesian_coordinates(ranges[i], angles[i], &x, &y);

			double pixel_x = (x * pixels_per_meter_x);
			double pixel_y = (y * pixels_per_meter_y);


			// quanto maior a possibilidade de ser obstáculo mais preto é o ponto
			double red = 0.5;
			double green = 0.5;
			double blue = 0.0;

			glBegin(GL_LINES);
			glColor3f(red, green, blue);
			glVertex2f(0.0, 0.0);
			glVertex2f(pixel_x, pixel_y);
			glEnd();
		}
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
				rect, rect+1, rect+2, 2.25);

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
draw_velodyne_on_ground()
{
	draw_spheres();
	draw_circle(60.0, 0, 0, 0, 0, 0);
//	draw_velodyne_points(last_velodyne_on_ground_ranges, last_velodyne_on_ground_angles, last_velodyne_on_ground_intensities, velodyne_on_ground_message.num_rays);
	draw_velodyne_points(current_velodyne_on_ground_ranges, current_velodyne_on_ground_angles, current_velodyne_on_ground_intensities, velodyne_on_ground_message.num_rays);
	draw_particles(particles_message);
	draw_car_centralized();
}


void
handle_velodyne_on_ground_display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
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

	int i;
//	char y_label[64];
	double px, py, jmp;
	double MAX_RANGE = 50.0;

	jmp = ONE_DIMENSIONAL_PLOT_HEIGHT;

	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(-ONE_DIMENSIONAL_PLOT_WIDTH / 2, jmp);
	glVertex2f(ONE_DIMENSIONAL_PLOT_WIDTH / 2, jmp);
	glVertex2f(-ONE_DIMENSIONAL_PLOT_WIDTH / 2, 2 * jmp);
	glVertex2f(ONE_DIMENSIONAL_PLOT_WIDTH / 2, 2 * jmp);
	glEnd();

	for (i = 0; i < velodyne_on_ground_message.num_rays; i++)
	{
		px = (current_velodyne_on_ground_angles[i] / M_PI) * (ONE_DIMENSIONAL_PLOT_WIDTH / 2 - 40);
		py = (current_velodyne_on_ground_ranges[i] / MAX_RANGE) * jmp;

		// draw measurement
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 1.0);
		glVertex2f(px, 0);
		glVertex2f(px, py);
		glEnd();

//		sprintf(y_label, "%.2lf", current_velodyne_on_ground_ranges[i]);
//		write_text(px, py, y_label);
	}

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
		velodyne_on_ground_window_id = glutCreateWindow("Velodyne on ground");
		glutReshapeFunc(handle_velodyne_on_ground_resize);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glutDisplayFunc(handle_velodyne_on_ground_display);
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
moving_objects3_velodyne_on_ground_message_handler()
{
	if (velodyne_on_ground_message.num_rays > 0)
	{
		memcpy(last_velodyne_on_ground_angles, current_velodyne_on_ground_angles, velodyne_on_ground_message.num_rays * sizeof(double));
		memcpy(last_velodyne_on_ground_ranges, current_velodyne_on_ground_ranges, velodyne_on_ground_message.num_rays * sizeof(double));
		memcpy(last_velodyne_on_ground_intensities, current_velodyne_on_ground_intensities, velodyne_on_ground_message.num_rays * sizeof(double));

		memcpy(current_velodyne_on_ground_angles, velodyne_on_ground_message.angles, velodyne_on_ground_message.num_rays * sizeof(double));
		memcpy(current_velodyne_on_ground_ranges, velodyne_on_ground_message.ranges, velodyne_on_ground_message.num_rays * sizeof(double));
		memcpy(current_velodyne_on_ground_intensities, velodyne_on_ground_message.intensity, velodyne_on_ground_message.num_rays * sizeof(double));
	}
}


void
moving_objects3_particles_message_handler()
{

}


void
subcribe_messages()
{
	carmen_subscribe_velodyne_projected_message(
			&velodyne_on_ground_message,
			(carmen_handler_t) moving_objects3_velodyne_on_ground_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_moving_objects3_particles_message(
				&particles_message,
				(carmen_handler_t) moving_objects3_particles_message_handler,
				CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_module(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	pixels_per_meter_x = ((double) MAP_VIEWER_SIZE) / (2 * pow(2.0, NUM_SPHERES));
	pixels_per_meter_y = ((double) MAP_VIEWER_SIZE) / (2 * pow(2.0, NUM_SPHERES));
}


int
main(int argc, char **argv)
{
	initialize_ipc(argc, argv);
	read_parameters(argc, argv);
	initialize_module(argc, argv);
	initialize_viewer(argc, argv);
	subcribe_messages();
	glutMainLoop();

	return 0;
}

