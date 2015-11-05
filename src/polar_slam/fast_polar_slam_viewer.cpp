#include <GL/glut.h>
#include <carmen/carmen.h>
#include <carmen/polar_point.h>
#include <carmen/fast_polar_slam_interface.h>

int NUM_SPHERES = 6; // TODO: ler do param daemon
const int REDRAW_UPDATE_PERIOD = 40;
const int BEST_PARTICLE_MAP_VIEWER_SIZE = 600; // pixels
const int NUM_POINTS_PER_VELODYNE = 36000;
const int MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH = 900; // pixels
const int MEASUREMENT_PROBABILITY_VIEWER_SIZE_HEIGHT = 200; // pixels

const int show_best_particle_window = 1;
const int show_velodyne_on_grond_window = 0;
const int show_measurement_probability_window = 1;

double pixels_per_meter_x;
double pixels_per_meter_y;

int best_particle_window_id;
int velodyne_on_ground_window_id;
int measurement_probability_window_id;

carmen_fast_polar_slam_particles_message particles_message;
carmen_fast_polar_slam_best_particle_message best_particle_message;
carmen_fast_polar_slam_velodyne_projected_on_ground_message velodyne_on_ground_message;
carmen_fast_polar_slam_measurement_model_message measurement_model_message;

double last_velodyne_on_ground_angles[NUM_POINTS_PER_VELODYNE];
double last_velodyne_on_ground_ranges[NUM_POINTS_PER_VELODYNE];
double last_velodyne_on_ground_intensities[NUM_POINTS_PER_VELODYNE];

double current_velodyne_on_ground_angles[NUM_POINTS_PER_VELODYNE];
double current_velodyne_on_ground_ranges[NUM_POINTS_PER_VELODYNE];
double current_velodyne_on_ground_intensities[NUM_POINTS_PER_VELODYNE];

double timestamp_last_update = 0;

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
draw_best_particle_map_points()
{
	int i;
	double x, y, range, angle;

	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);

	for(i = 0; i < best_particle_message.map_size; i++)
	{
		if (best_particle_message.particle_map_occupation[i] != 0)
		{
			range = best_particle_message.particle_map[i].radius;
			angle = best_particle_message.particle_map[i].angle;

			transform_polar_coordinates_to_cartesian_coordinates(range, angle, &x, &y);

			double pixel_x = x * pixels_per_meter_x;
			double pixel_y = y * pixels_per_meter_y;

			glVertex2f(pixel_x, pixel_y);
		}
	}

	glEnd();
}


void
draw_estimated_and_measured_lasers()
{
	int i;
	double ex, ey, mx, my, estimated_range, measured_range, angle;
	double MAX_RANGE = 50.0; // TODO: ler do ini

	for (i = 0; i < measurement_model_message.num_rays; i++)
	{
		estimated_range = measurement_model_message.estimated_ranges[i];
		measured_range = measurement_model_message.measured_ranges[i];
		angle = measurement_model_message.angles[i];

		if (estimated_range == -1)
			estimated_range = MAX_RANGE;

		transform_polar_coordinates_to_cartesian_coordinates(estimated_range, angle, &ex, &ey);
		transform_polar_coordinates_to_cartesian_coordinates(measured_range, angle, &mx, &my);

		ex = ex * pixels_per_meter_x;
		ey = ey * pixels_per_meter_y;

		mx = mx * pixels_per_meter_x;
		my = my * pixels_per_meter_y;

		// draw estimated ray (green)
		glBegin(GL_LINE_LOOP);
		glColor3f(0.0, 1.0, 0.0);
		glVertex2f(0, 0);
		glVertex2f(ex, ey);
		glEnd();

		// draw measured ray (red)
		glBegin(GL_LINE_LOOP);
		glColor3f(1.0, 0.0, 0.0);
		glVertex2f(0, 0);
		glVertex2f(mx, my);
		glEnd();

		// draw a dot in the end of the estimated ray
		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(0.0, 1.0, 0.0);
		glVertex2f(ex, ey);
		glEnd();

		// draw a dot in the end of the measured ray
		glPointSize(5);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);
		glVertex2f(mx, my);
		glEnd();
	}

	glPointSize(1);
}


void
draw_spheres()
{
	draw_circle(pow(2, NUM_SPHERES), 0, 0, 0, 0, 0);

//	for (int i = 0; i < NUM_SPHERES; i++)
//	{
//		draw_circle(pow(2, i + 1), 0, 0, 0, 0, 0);
//	}
}

void
draw_best_particle_map()
{
	draw_spheres();
	draw_car_centralized();
	draw_best_particle_map_points();
	draw_estimated_and_measured_lasers();
}


void
draw_particles()
{
	double x, y, b, r;
	// double weight;

	for (int i = 0; i < particles_message.num_particles; i++)
	{
		x = particles_message.particle_pose[i].position.x - best_particle_message.particle_pose.position.x;
		y = particles_message.particle_pose[i].position.y - best_particle_message.particle_pose.position.y;

		x = x * pixels_per_meter_x;
		y = y * pixels_per_meter_y;

		// weight = particles_message.weight[i];

		// quanto maior o peso mais vermelha a particula sera,
		// quanto menor o peso mais azul a particula sera
		b = 1.0; //1.0 - weight;
		r = 0.0; // weight;

		draw_circle(0.3, x, y, r, 0, b);
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

		// quanto mais azul menor a chance de ser obstaculo,
		// quanto mais vermelhor maior a chance de ser obstaculo
		double blue = 1.0 - intensities[i];
		double red = intensities[i];

		glColor3f(red, 0.0, blue);
		glVertex2f(pixel_x, pixel_y);
	}

	glEnd();
}


void
draw_reading_on_ground()
{
	int i;
	double ex, ey, mx, my, estimated_range, measured_range, angle;
	double MAX_RANGE = 50.0; // TODO: ler do ini

	for (i = 0; i < measurement_model_message.num_rays; i++)
	{
		estimated_range = measurement_model_message.estimated_ranges[i];
		measured_range = measurement_model_message.measured_ranges[i];
		angle = measurement_model_message.angles[i];

		if (estimated_range == -1)
			estimated_range = MAX_RANGE;

		transform_polar_coordinates_to_cartesian_coordinates(estimated_range, angle, &ex, &ey);
		transform_polar_coordinates_to_cartesian_coordinates(measured_range, angle, &mx, &my);

		ex = ex * pixels_per_meter_x;
		ey = ey * pixels_per_meter_y;

		mx = mx * pixels_per_meter_x;
		my = my * pixels_per_meter_y;

		// draw estimated ray (green)
		glBegin(GL_LINE_LOOP);
		glColor3f(0.0, 1.0, 0.0);
		glVertex2f(0, 0);
		glVertex2f(ex, ey);
		glEnd();

		// draw measured ray (blue)
		glBegin(GL_LINE_LOOP);
		glColor3f(0.0, 0.0, 1.0);
		glVertex2f(0, 0);
		glVertex2f(mx, my);
		glEnd();
	}

	glPointSize(1);
}


void
draw_velodyne_on_ground()
{
	draw_car_centralized();
	draw_circle(pow(2, NUM_SPHERES), 0, 0, 0, 0, 0);
	draw_velodyne_points(last_velodyne_on_ground_ranges, last_velodyne_on_ground_angles, last_velodyne_on_ground_intensities, velodyne_on_ground_message.num_rays);
	draw_velodyne_points(current_velodyne_on_ground_ranges, current_velodyne_on_ground_angles, current_velodyne_on_ground_intensities, velodyne_on_ground_message.num_rays);
	draw_reading_on_ground();
}


void
handle_best_particle_map_display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
	draw_best_particle_map();
	draw_particles();

	glutSwapBuffers();
}


void
handle_best_particle_map_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) -BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) -BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) BEST_PARTICLE_MAP_VIEWER_SIZE / 2)
	);
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
		((double) -BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) -BEST_PARTICLE_MAP_VIEWER_SIZE / 2),
		((double) BEST_PARTICLE_MAP_VIEWER_SIZE / 2)
	);
}


void
handle_measurement_probability_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) -MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2),
		((double) MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2),
		((double) 0),
		((double) MEASUREMENT_PROBABILITY_VIEWER_SIZE_HEIGHT + 20)
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
handle_measurement_probability_display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT);

	int i;
	char y_label[64];
	double px, py, jmp;
	double MAX_RANGE = 50.0;

	jmp = MEASUREMENT_PROBABILITY_VIEWER_SIZE_HEIGHT / 3;

	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 0.0);
	glVertex2f(-MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2, jmp);
	glVertex2f(MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2, jmp);
	glVertex2f(-MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2, 2 * jmp);
	glVertex2f(MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2, 2 * jmp);
	glEnd();

	for (i = 0; i < measurement_model_message.num_rays; i++)
	{
		px = (measurement_model_message.angles[i] / M_PI) * (MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH / 2 - 40);
		py = (measurement_model_message.measured_ranges[i] / MAX_RANGE) * jmp;

		// draw measurement
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 1.0);
		glVertex2f(px, 0);
		glVertex2f(px, py);
		glEnd();

		sprintf(y_label, "%.2lf", measurement_model_message.measured_ranges[i]);
		write_text(px, py, y_label);

		if (measurement_model_message.estimated_ranges[i] == -1)
			measurement_model_message.estimated_ranges[i] = MAX_RANGE;

		py = (measurement_model_message.estimated_ranges[i] / MAX_RANGE) * jmp + jmp;

		// draw estimated reading
		glBegin(GL_LINES);
		glColor3f(0.0, 1.0, 0.0);
		glVertex2f(px, jmp);
		glVertex2f(px, py);
		glEnd();

		sprintf(y_label, "%.2lf", measurement_model_message.estimated_ranges[i]);
		write_text(px, py, y_label);

		py = measurement_model_message.ray_probability[i] * jmp + 2 * jmp;

		// draw prob
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 0.0);
		glVertex2f(px, 2 * jmp);
		glVertex2f(px, py);
		glEnd();

		sprintf(y_label, "%.2lf", measurement_model_message.ray_probability[i]);
		write_text(px, py, y_label);
	}

	glutSwapBuffers();
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);

	if (show_best_particle_window)
		glutPostWindowRedisplay(best_particle_window_id);

	if (show_velodyne_on_grond_window)
		glutPostWindowRedisplay(velodyne_on_ground_window_id);

	if (show_measurement_probability_window)
		glutPostWindowRedisplay(measurement_probability_window_id);

	glutTimerFunc(REDRAW_UPDATE_PERIOD, Timer, 1);
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	// BEST PARTICLE MAP WINDOW
	if (show_best_particle_window)
	{
		glutInitWindowSize(BEST_PARTICLE_MAP_VIEWER_SIZE, BEST_PARTICLE_MAP_VIEWER_SIZE);
		glutInitWindowPosition(0, 0);
		best_particle_window_id = glutCreateWindow("Best Particle Map");
		glutReshapeFunc(handle_best_particle_map_resize);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glutDisplayFunc(handle_best_particle_map_display);
	}

	// VELODYNE ON GROUND
	if (show_velodyne_on_grond_window)
	{
		glutInitWindowSize(BEST_PARTICLE_MAP_VIEWER_SIZE, BEST_PARTICLE_MAP_VIEWER_SIZE);
		glutInitWindowPosition(0, 0);
		velodyne_on_ground_window_id = glutCreateWindow("Velodyne on ground");
		glutReshapeFunc(handle_velodyne_on_ground_resize);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glutDisplayFunc(handle_velodyne_on_ground_display);
	}

	// MEASUREMENT PROBABILITY
	if (show_measurement_probability_window)
	{
		glutInitWindowSize(MEASUREMENT_PROBABILITY_VIEWER_SIZE_WIDTH, MEASUREMENT_PROBABILITY_VIEWER_SIZE_HEIGHT);
		glutInitWindowPosition(0, 0);
		measurement_probability_window_id = glutCreateWindow("Measurement Probability");
		glutReshapeFunc(handle_measurement_probability_resize);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glutDisplayFunc(handle_measurement_probability_display);
	}

	glutTimerFunc(REDRAW_UPDATE_PERIOD, Timer, 1);
}


void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("fast_polar_slam_viewer: disconnected.\n");
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
fast_polar_slam_best_particle_message_handler()
{
}


void
fast_polar_slam_velodyne_on_ground_message_handler()
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
fast_polar_slam_particles_message_handler()
{
}


void
fast_polar_measurement_model_message_handler()
{
}


void
subcribe_messages()
{
	carmen_fast_polar_slam_subscribe_best_particle_message(
			&best_particle_message,
			(carmen_handler_t) fast_polar_slam_best_particle_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_fast_polar_slam_subscribe_velodyne_projected_message(
			&velodyne_on_ground_message,
			(carmen_handler_t) fast_polar_slam_velodyne_on_ground_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_fast_polar_slam_subscribe_particles_message(
			&particles_message,
			(carmen_handler_t) fast_polar_slam_particles_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_fast_polar_slam_subscribe_measurement_model_message(
			&measurement_model_message,
			(carmen_handler_t) fast_polar_measurement_model_message_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_module(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	pixels_per_meter_x = ((double) BEST_PARTICLE_MAP_VIEWER_SIZE) / (2 * pow(2.0, NUM_SPHERES));
	pixels_per_meter_y = ((double) BEST_PARTICLE_MAP_VIEWER_SIZE) / (2 * pow(2.0, NUM_SPHERES));
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

