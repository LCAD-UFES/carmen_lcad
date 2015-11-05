/*
 * post_localize_main.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */
#include <vector>
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <GL/glut.h>
#include <tf.h>

#include "landmark.h"
#include "landmark_map.h"
#include "post_localize.h"

using namespace post_slam;
using namespace std;

/********************************/
/********* PARAMETROS ***********/
/********************************/

const double min_range = 2.0;
const double max_range = 20.0;

const int zoom = 3;
const int view_width = 360;
const int view_height = 32;
const int redraw_update_period = 30;

const int column_correspondence[32] =
{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
};

/********************************/
/*********** GLOBAIS ************/
/********************************/

int velodyne_window_id;
int landmark_map_window_id;

int first_landmak_draw = 1;
int first_fused_odometry = 1;
int first_timer_function_call = 1;
int first_velodyne_message_received = 0;

double fused_odometry_first_pose_x;
double fused_odometry_first_pose_y;
double fused_odometry_first_pose_theta;

carmen_fused_odometry_message fused_odometry_message;
carmen_velodyne_partial_scan_message velodyne_message_g;

carmen_pose_3D_t sensor_board_pose_g;
carmen_pose_3D_t velodyne_pose_g;
carmen_pose_3D_t car_pose_g;

post_slam::LandmarkMap landmark_map;
vector<post_slam::Landmark*> landmarks;
vector<post_slam::Landmark*> matched_landmarks;

post_slam::PostLocalize localizer;
double time_last_fused_odometry = 0;

void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("velodyne viewer: disconnected.\n");
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
free_landmarks(vector<post_slam::Landmark*> *landmarks)
{
	unsigned int i;

	for (i = 0; i < landmarks->size(); i++)
		delete(landmarks->at(i));
}


// TODO: checar pq quando eu reprojeto a landmark o desenho fica ruim!
// Deve ter algo a ver com o jeito que eu estava calculando o angulo.
// => na verdade eh pq eu tenho que fazer a transformada para a projecao
// usando a TF!
void
draw_landmarks(vector<post_slam::Landmark*> *landmarks __attribute__ ((unused)))
{
	int i, j, k, l;

	glBegin(GL_POINTS);

	for (i = 0; i < velodyne_message_g.number_of_32_laser_shots; i++)
	{
		int first, all_lasers_have_same_range;
		double range, last_range, angle;

		first = 1;
		all_lasers_have_same_range = 1;
		angle = velodyne_message_g.partial_scan[i].angle;

		for (j = 10; j < 32; j++)
		{
			range = ((double) velodyne_message_g.partial_scan[i].distance[j]) / 500.0;

			if (first)
			{
				last_range = range;
				first = 0;
			}
			else if (fabs(range - last_range) > 0.5)
			{
				all_lasers_have_same_range = 0;
				break;
			}

			last_range = range;
		}

		if (all_lasers_have_same_range)
		{
			for (j = 10; j < 32; j++)
			{
				for (k = 0; k < zoom; k++)
				{
					for (l = 0; l < zoom; l++)
					{
						glColor3f(1.0 - ((range - min_range) / (max_range - min_range)), 0.0, 1.0 - ((range - min_range) / (max_range - min_range)));
						glVertex2i((zoom * angle + l), (zoom * j + k));
					}
				}
			}
		}
	}

	glEnd();
}


void
draw_landmarks_on_map(vector<post_slam::Landmark*> *landmarks)
{
	int i;

	glPointSize(2);
	glBegin(GL_POINTS);

	for (i = 0; i < (int) landmarks->size(); i++)
	{
		glColor3f(0.0, 0.0, 1.0);

		glVertex2i(
				landmarks->at(i)->x - fused_odometry_first_pose_x + 150,
				landmarks->at(i)->y - fused_odometry_first_pose_y + 200
		);
	}

	glEnd();
}


void
draw_matched_landmarks_on_map(vector<post_slam::Landmark*> *landmarks)
{
	int i;

	glPointSize(2);
	glBegin(GL_POINTS);

	for (i = 0; i < (int) landmarks->size(); i++)
	{
		glColor3f(0.0, 1.0, 0.0);

		glVertex2i(
				landmarks->at(i)->x - fused_odometry_first_pose_x + 150,
				landmarks->at(i)->y - fused_odometry_first_pose_y + 200
		);
	}

	glEnd();
}


vector<post_slam::Landmark*>
detect_new_landmarks()
{
	vector<post_slam::Landmark*> landmarks;

	landmarks = post_slam::detect_landmarks(&velodyne_message_g);

	post_slam::update_landmarks_position_in_the_world(
			&landmarks,
			fused_odometry_message.pose.position.x,
			fused_odometry_message.pose.position.y,
			fused_odometry_message.pose.orientation.yaw
	);

	return landmarks;
}


void
draw_last_velodyne_message()
{
	glBegin(GL_POINTS);

	int i, j, k, l;
	for (i = 0; i < velodyne_message_g.number_of_32_laser_shots; i++)
	{
		int x = (int) (velodyne_message_g.partial_scan[i].angle);

		for (j = 0; j < 32; j++)
		{
			double range = (((double) velodyne_message_g.partial_scan[i].distance[j]) / 500.0);

			if ((range > max_range) || (range <= min_range))
				range = max_range;

			for (k = 0; k < zoom; k++)
			{
				for (l = 0; l < zoom; l++)
				{
					glColor3f(1.0 - ((range - min_range) / (max_range - min_range)), 0.0, 0.0);
					glVertex2i((zoom * x + l), (zoom * j + k));

				}
			}
		}
	}

	glEnd();
}


void
arrange_velodyne_vertical_angles_to_true_position()
{
	int i, j;
	unsigned short original_distances[32];

	for (i = 0; i < velodyne_message_g.number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message_g.partial_scan[i].distance, 32 * sizeof(unsigned short));

		for (j = 0; j < 32; j++)
		{
			velodyne_message_g.partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
		}
	}
}


void
velodyne_message_handler()
{
	//	free_landmarks(&landmarks);
	//
	//	arrange_velodyne_vertical_angles_to_true_position();
	//	landmarks = detect_new_landmarks();
	//	matched_landmarks = landmark_map.match(&landmarks);
	//
	//	printf("04\n");
	//	first_velodyne_message_received = 1;
}

double last_theta = 0.0;

void
fused_odometry_message_handler()
{
	double now = carmen_get_time();


	if (first_fused_odometry)
	{
		//		fused_odometry_first_pose_x = fused_odometry_message.pose.position.x;
		//		fused_odometry_first_pose_y = fused_odometry_message.pose.position.y;
		//		fused_odometry_first_pose_theta = fused_odometry_message.pose.orientation.yaw;

		localizer.initialize(fused_odometry_message.pose.position.x, fused_odometry_message.pose.position.y, fused_odometry_message.pose.orientation.yaw);

		time_last_fused_odometry = now;
		last_theta = fused_odometry_message.pose.orientation.yaw;

		first_fused_odometry = 0;
	}

	double delta_t = now - time_last_fused_odometry;
	double angular_velocity = (fused_odometry_message.pose.orientation.yaw - last_theta) / delta_t;

	localizer.integrate_movement_information(fused_odometry_message.velocity.x, angular_velocity, delta_t);

//	printf("%lf\t%lf\t%lf\n", fused_odometry_message.velocity.x, fused_odometry_message.angular_velocity.roll, delta_t);
//	printf("%lf\t%lf\t%lf\n", fused_odometry_message.pose.position.x, fused_odometry_message.pose.position.y, fused_odometry_message.pose.position.z);
//	printf("%lf\t%lf\t%lf\n", fused_odometry_message.pose.orientation.pitch, fused_odometry_message.pose.orientation.roll, fused_odometry_message.pose.orientation.yaw);
	printf("%lf\t%lf\t%lf\n", localizer.get_pose().x, localizer.get_pose().y, localizer.get_pose().theta);

	time_last_fused_odometry = now;
	last_theta = fused_odometry_message.pose.orientation.yaw;
}


void
subcribe_messages()
{
	carmen_velodyne_subscribe_partial_scan_message(
			&velodyne_message_g,
			(carmen_handler_t) velodyne_message_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_fused_odometry_subscribe_fused_odometry_message(
			&fused_odometry_message,
			(carmen_handler_t) fused_odometry_message_handler,
			CARMEN_SUBSCRIBE_ALL
	);
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	glutPostWindowRedisplay(velodyne_window_id);
	glutPostWindowRedisplay(landmark_map_window_id);
	glutTimerFunc(redraw_update_period, Timer, 1);
}


void
draw_velodyne()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (!first_velodyne_message_received)
	{
		/**
		 * Esse comando eh para colocar a
		 * cor preta na tela inicial, quando
		 * nenhuma mensagem do velodyne chegou
		 * ainda. Sem ele, a janela openGL
		 * fica transparente, como se o mundo
		 * estivesse vazio. Eu so executo elas
		 * uma vez e nao sempre porque senao
		 * o rastro do velodyne fica aparecendo
		 * e atrapalha a visualizacao
		 */
		glClear(GL_COLOR_BUFFER_BIT);
	}
	else
	{
		draw_last_velodyne_message();
		draw_landmarks(&landmarks);
	}

	glutSwapBuffers();
}


void
draw_last_pose_received()
{
	double x = fused_odometry_message.pose.position.x - fused_odometry_first_pose_x + 150;
	double y = fused_odometry_message.pose.position.y - fused_odometry_first_pose_y + 200;

	glPointSize(2);

	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);
	glVertex2i(x, y);
	glEnd();
}


void
draw_landmark_map()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (first_fused_odometry || first_landmak_draw)
	{
		/**
		 * Esse comando eh para colocar a
		 * cor preta na tela inicial, quando
		 * nenhuma mensagem chegou ainda. Sem
		 * ele a tela fica transparente como
		 * se o mundo estivesse vazio.
		 */
		glClear(GL_COLOR_BUFFER_BIT);
		first_landmak_draw = 0;
	}
	else
	{
		draw_last_pose_received();
		draw_landmarks_on_map(&landmarks);
		draw_matched_landmarks_on_map(&matched_landmarks);
	}

	glutSwapBuffers();
}


void
handle_velodyne_viewer_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
			((double) 0.0),
			((double) 360.0 * zoom),
			((double) 0.0),
			((double) 32.0 * zoom)
	);
}


void
handle_landmark_map_viewer_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
			((double) -1000),
			((double) 440),
			((double) -400),
			((double) 300)
	);
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	// VELODYNE VIEWER WINDOW
	glutInitWindowSize(view_width * zoom, view_height * zoom);
	glutInitWindowPosition(50, 50);
	velodyne_window_id = glutCreateWindow("Velodyne Viewer");
	glutReshapeFunc(handle_velodyne_viewer_resize);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glutDisplayFunc(draw_velodyne);

	// MAP VIEWER WINDOW
	glutInitWindowSize(900, 700);
	glutPositionWindow(50, 50 + view_height * zoom + 30);
	landmark_map_window_id = glutCreateWindow("Map Viewer");
	glutReshapeFunc(handle_landmark_map_viewer_resize);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glutDisplayFunc(draw_landmark_map);

	glutTimerFunc(redraw_update_period, Timer, 1);
}


static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.roll), 0, NULL},

			{(char *) "velodyne",    (char *) "x", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.x), 0, NULL},
			{(char *) "velodyne",    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.y), 0, NULL},
			{(char *) "velodyne",    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.z), 0, NULL},
			{(char *) "velodyne",    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.yaw), 0, NULL},
			{(char *) "velodyne",    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.pitch), 0, NULL},
			{(char *) "velodyne",    (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.roll), 0, NULL}

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


void
initialize_module(int argc __attribute__ ((unused)), char **argv)
{
	post_slam::initialize_transforms(
			sensor_board_pose_g,
			velodyne_pose_g,
			car_pose_g
	);

	localizer.load_landmark_map(argv[1]);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <mapa-landmarks>\n", argv[0]));

	initialize_ipc(argc, argv);
	read_parameters(argc, argv);
	initialize_module(argc, argv);
	//initialize_viewer(argc, argv);
	subcribe_messages();
	//glutMainLoop();
	carmen_ipc_dispatch();

	return 0;
}

