/*
 * post_mapping_main.c
 *
 *  Created on: Apr 8, 2013
 *      Author: filipe
 */
#include <vector>
#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <GL/glut.h>
#include <tf.h>

#include "landmark.h"
#include "landmark_map.h"

using namespace post_slam;
using namespace std;

/********************************/
/********* PARAMETROS ***********/
/********************************/

const double min_range = 0.0;
const double max_range = 255.0;

const int zoom = 3;
const int ground_map_zoom = 8;
const int view_width = 360;
const int view_height = 32;
const int redraw_update_period = 30;

const int column_correspondence[32] =
{
	0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
	24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
};

const static double sorted_vertical_angles[32] =
{
	-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
	-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
	-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
	5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
};

/********************************/
/*********** GLOBAIS ************/
/********************************/

char *output_filename;

int velodyne_window_id;
int landmark_map_window_id;
int velodyne_ground_view_id;

int first_timer_function_call = 1;
int first_velodyne_message_received = 0;
int first_localize_ackerman_globalpos = 1;

double localize_ackerman_first_globalpos_x;
double localize_ackerman_first_globalpos_y;
double localize_ackerman_first_globalpos_theta;

carmen_localize_ackerman_globalpos_message globalpos_message;
carmen_velodyne_partial_scan_message velodyne_message_g;

carmen_pose_3D_t sensor_board_pose_g;
carmen_pose_3D_t velodyne_pose_g;
carmen_pose_3D_t car_pose_g;

post_slam::LandmarkMap landmark_map;
vector<post_slam::Landmark*> landmarks;

carmen_velodyne_partial_scan_message current_velodyne, last_velodyne;

void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("velodyne viewer: disconnected.\n");
		landmark_map.save(output_filename);
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


void
draw_landmarks(vector<post_slam::Landmark*> *landmarks __attribute__ ((unused)))
{
	glBegin(GL_POINTS);

	int i, j, k, l, m;
	double x, y, radius;
	double angle, range;
	vector<Landmark*> v;
	int post_mean_radius = 5;

	for (i = post_mean_radius; i < (velodyne_message_g.number_of_32_laser_shots - post_mean_radius); i++)
	{
//		int fail = 0;
//
//		for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
//			if (!is_a_possible_post(velodyne_message.partial_scan[j].distance))
//			{
//				fail = 1;
//				break;
//			}
//
//		if (!fail)
//		{
//			for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
//				for (k = 10; k < 32; k++)
//					for (l = 0; l < zoom; l++)
//						for (m = 0; m < zoom; m++)
//						{
//							glColor3f(1.0 - ((velodyne_message.partial_scan[i].distance[15] - min_range) / (max_range - min_range)), 0.0, 1.0 - ((velodyne_message.partial_scan[i].distance[15] - min_range) / (max_range - min_range)));
//							glVertex2i((zoom * j + l), (zoom * k + m));
//						}
//
//			i += post_mean_radius;
//		}
	}

	glEnd();
}


// TODO: checar pq quando eu reprojeto a landmark o desenho fica ruim!
// Deve ter algo a ver com o jeito que eu estava calculando o angulo
void
draw_landmarks2(vector<post_slam::Landmark*> *landmarks __attribute__ ((unused)))
{
	glBegin(GL_POINTS);

	int i, j, k, l, m;
	double x, y, radius;
	double angle, range;
	vector<Landmark*> v;
	int post_mean_radius = 5;

	for (i = post_mean_radius; i < (velodyne_message_g.number_of_32_laser_shots - post_mean_radius); i++)
	{
		double mean_range = 0;
		double percentage_of_points_near_the_mean = 0.0;

		/** Computa o range medio na janela **/
		for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
			for (k = 0; k < 32; k++)
				mean_range += ((double) velodyne_message_g.partial_scan[j].distance[k]) / 500.0;

		mean_range /= (2 * post_mean_radius * 22);

		double variance = 0;

		/** Computa a variancia dos pontos  **/
		for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
			for (k = 10; k < 32; k++)
					// percentage_of_points_near_the_mean++;
					variance = fabs(((double) velodyne_message_g.partial_scan[j].distance[k]) / 500.0 - mean_range);

		variance /= (2 * post_mean_radius * 22);
		percentage_of_points_near_the_mean /= (2 * post_mean_radius * 22);

		/** se a porcentagem for alta, temos um possivel poste **/
		// if (percentage_of_points_near_the_mean > 0.9)
		if ((variance / mean_range) < 0.2)
		{
			for (j = (i - post_mean_radius); j < (i + post_mean_radius); j++)
				for (k = 10; k < 32; k++)
					for (l = 0; l < zoom; l++)
						for (m = 0; m < zoom; m++)
						{
							glColor3f(1.0 - ((mean_range - min_range) / (max_range - min_range)), 0.0, 1.0 - ((mean_range - min_range) / (max_range - min_range)));
							glVertex2i((zoom * j + l), (zoom * k + m));
						}

			i += post_mean_radius;
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
			landmarks->at(i)->x - localize_ackerman_first_globalpos_x + 150,
			landmarks->at(i)->y - localize_ackerman_first_globalpos_y + 200
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
		globalpos_message.globalpos.x,
		globalpos_message.globalpos.y,
		globalpos_message.globalpos.theta
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
			// double range = (((double) velodyne_message_g.partial_scan[i].distance[j]) / 500.0);
			double range = (((double) velodyne_message_g.partial_scan[i].intensity[j])) *  10;

			if ((range > max_range) || (range <= min_range))
				range = max_range;

			for (k = 0; k < zoom; k++)
			{
				for (l = 0; l < zoom; l++)
				{
					glColor3f(((range - min_range) / (max_range - min_range)), 0.0, 0.0);
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
	unsigned char original_intensities[32];

	for (i = 0; i < velodyne_message_g.number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message_g.partial_scan[i].distance, 32 * sizeof(unsigned short));
		memcpy(original_intensities, velodyne_message_g.partial_scan[i].intensity, 32 * sizeof(unsigned char));

		for (j = 0; j < 32; j++)
		{
			velodyne_message_g.partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
			velodyne_message_g.partial_scan[i].intensity[column_correspondence[j]] = original_intensities[j];
		}
	}
}


void
update_current_and_last_velodyne(carmen_velodyne_partial_scan_message *velodyne_message)
{
	int i;
	static int first_iteraction = 1;

	if (first_iteraction)
	{
		current_velodyne.partial_scan = (carmen_velodyne_32_laser_shot *) calloc (velodyne_message->number_of_32_laser_shots, sizeof(carmen_velodyne_32_laser_shot));
		last_velodyne.partial_scan = (carmen_velodyne_32_laser_shot *) calloc (velodyne_message->number_of_32_laser_shots, sizeof(carmen_velodyne_32_laser_shot));
		first_iteraction = 0;
	}

	last_velodyne.number_of_32_laser_shots = current_velodyne.number_of_32_laser_shots;
	current_velodyne.number_of_32_laser_shots = velodyne_message->number_of_32_laser_shots;

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		last_velodyne.partial_scan[i].angle = current_velodyne.partial_scan[i].angle;
		memcpy(last_velodyne.partial_scan[i].distance, current_velodyne.partial_scan[i].distance, 32 * sizeof(short));
		memcpy(last_velodyne.partial_scan[i].intensity, current_velodyne.partial_scan[i].intensity, 32 * sizeof(char));

		current_velodyne.partial_scan[i].angle = velodyne_message->partial_scan[i].angle;
		memcpy(current_velodyne.partial_scan[i].distance, velodyne_message->partial_scan[i].distance, 32 * sizeof(short));
		memcpy(current_velodyne.partial_scan[i].intensity, velodyne_message->partial_scan[i].intensity, 32 * sizeof(char));
	}
}


void
velodyne_message_handler()
{
	//free_landmarks(&landmarks);

	arrange_velodyne_vertical_angles_to_true_position();
	//landmarks = detect_new_landmarks();
	//landmark_map.add(&landmarks);

	update_current_and_last_velodyne(&velodyne_message_g);

	first_velodyne_message_received = 1;
}


void
localize_message_handler()
{
	return;

	if (first_localize_ackerman_globalpos)
	{
		localize_ackerman_first_globalpos_x = globalpos_message.globalpos.x;
		localize_ackerman_first_globalpos_y = globalpos_message.globalpos.y;
		localize_ackerman_first_globalpos_theta = globalpos_message.globalpos.theta;

		first_localize_ackerman_globalpos = 0;
	}
}


void
subcribe_messages()
{
	carmen_velodyne_subscribe_partial_scan_message(
			&velodyne_message_g,
			(carmen_handler_t) velodyne_message_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_localize_ackerman_subscribe_globalpos_message(
			&globalpos_message,
			(carmen_handler_t) localize_message_handler,
			CARMEN_SUBSCRIBE_ALL);
}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	glutPostWindowRedisplay(velodyne_window_id);
	glutPostWindowRedisplay(landmark_map_window_id);
	glutPostWindowRedisplay(velodyne_ground_view_id);
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
	double x = globalpos_message.globalpos.x - localize_ackerman_first_globalpos_x + 150;
	double y = globalpos_message.globalpos.y - localize_ackerman_first_globalpos_y + 200;

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

	if (first_localize_ackerman_globalpos)
	{
		/**
		 * Esse comando eh para colocar a
		 * cor preta na tela inicial, quando
		 * nenhuma mensagem chegou ainda. Sem
		 * ele a tela fica transparente como
		 * se o mundo estivesse vazio.
		 */
		glClear(GL_COLOR_BUFFER_BIT);
	}
	else
	{
		draw_last_pose_received();
		draw_landmarks_on_map(&landmarks);
	}

	glutSwapBuffers();
}

typedef struct
{
	float x, y;
}GroundPoint;

vector<GroundPoint>
draw_velodyne_message_projected_on_the_ground(carmen_velodyne_partial_scan_message velodyne_message)
{
	int i, j, k, l;
	double car_height = 1.725;
	vector<GroundPoint> v;

	for (i = 0; i < velodyne_message.number_of_32_laser_shots; i++)
	{
		int x = (int) (velodyne_message.partial_scan[i].angle);
		double hor_angle = carmen_degrees_to_radians(velodyne_message.partial_scan[i].angle);
		double cos_rot_angle = cos(hor_angle);
		double sin_rot_angle = sin(hor_angle);

		for (j = 0; j < (32 - 1); j++)
		{
			double range_0 = (((double) velodyne_message.partial_scan[i].distance[j]) / 500.0);
			double range_1 = (((double) velodyne_message.partial_scan[i].distance[j + 1]) / 500.0);

			if ((range_0 > max_range) || (range_0 <= min_range))
//				range_0 = max_range;
				continue;

			if ((range_1 > max_range) || (range_1 <= min_range))
//				range_1 = max_range;
				continue;

			double angle_0 = carmen_degrees_to_radians(sorted_vertical_angles[j]);
			double angle_1 = carmen_degrees_to_radians(sorted_vertical_angles[j + 1]);

			double cos_vert_angle0 = cos(angle_0);
			double sin_vert_angle0 = sin(angle_0);

			double cos_vert_angle1 = cos(angle_1);
			double sin_vert_angle1 = sin(angle_1);

			double xy_distance0 = range_0 * cos_vert_angle0;
			double xy_distance1 = range_1 * cos_vert_angle1;

			double x0 = (xy_distance0 * cos_rot_angle);
			double y0 = (xy_distance0 * sin_rot_angle);
			double z0 = (range_0 * sin_vert_angle0);

			double x1 = (xy_distance1 * cos_rot_angle);
			double y1 = (xy_distance1 * sin_rot_angle);
			double z1 = (range_1 * sin_vert_angle1);

			double delta_ray = xy_distance1 - xy_distance0;
			double next_ray_angle = -carmen_normalize_theta(angle_1 - angle_0) + atan(car_height / xy_distance0);
			double expected_delta_ray = (car_height - xy_distance0 * tan(next_ray_angle)) / tan(next_ray_angle);

			if (delta_ray < expected_delta_ray)
			{
				int p, q;

				p = x1 * ground_map_zoom + ground_map_zoom / 2;
				q = y1 * ground_map_zoom + ground_map_zoom / 2;

				double color = delta_ray / expected_delta_ray;

				if (color < 0.5)
				{
					GroundPoint ground_point;
					ground_point.x = x1;
					ground_point.y = y1;
					v.push_back(ground_point);

					glColor3f(color, color, color);
					glVertex2i(p + 30 * ground_map_zoom, q + 30 * ground_map_zoom);
				}
				else
				{
					glColor3f(1.0, 0.0, 0.0);
					glVertex2i(p + 30 * ground_map_zoom, q + 30 * ground_map_zoom);
				}
			}
		}
	}

	return v;
}


void
draw_landmarks_on_the_ground_map()
{
	int i;

	double pose_globalx = globalpos_message.globalpos.x;
	double pose_globaly = globalpos_message.globalpos.y;
	double pose_globaltheta = globalpos_message.globalpos.theta;

	for (i = 0; i < (int) landmarks.size(); i++)
	{
		double landmark_globalx = landmarks.at(i)->x;
		double landmark_globaly = landmarks.at(i)->y;

		double dist = sqrt(pow(landmark_globalx - pose_globalx, 2) + pow(landmark_globaly - pose_globaly, 2));

		if (dist < 30)
		{
			int j, k, m, n;

			double landmark_localx = landmark_globalx - pose_globalx;
			double landmark_localy = landmark_globaly - pose_globaly;

			double landmark_rotatedx = landmark_localx * cos(-pose_globaltheta) - landmark_localy * sin(-pose_globaltheta);
			double landmark_rotatedy = landmark_localx * sin(-pose_globaltheta) + landmark_localy * cos(-pose_globaltheta);

			for (j = 0; j < landmarks.at(i)->radius; j++)
			{
				for (k = 0; k < landmarks.at(i)->radius; k++)
				{
					for (m = 0; m < ground_map_zoom; m++)
					{
						for (n = 0; n < ground_map_zoom; n++)
						{
							int p = (landmark_rotatedx - landmarks.at(i)->radius + j) * ground_map_zoom + m;
							int q = (landmark_rotatedy - landmarks.at(i)->radius + k) * ground_map_zoom + n;

							glColor3f(1.0, 0.0, 0.0);
							glVertex2i(
								p + 30 * ground_map_zoom,
								q + 30 * ground_map_zoom
							);
						}
					}
				}
			}
		}
	}
}

typedef struct
{
	double angle;
	double dist;
	vector<GroundPoint> points;
}Line;

void
calculate_line_parameters(Line *l)
{
	/*******************************/
	/** Least square line fitting **/
	/*******************************/

	int i;
	double sum_x, sum_y, sum_sq_x, sum_xy, mean_x, mean_y;

	if (l->points.size() < 2)
		return;

	sum_x = sum_y = 0;
	mean_x = mean_y = 0;
	sum_xy = sum_sq_x = 0;

	for (i = 0; i < l->points.size(); i++)
	{
		sum_x += l->points[i].x;
		sum_y += l->points[i].y;

		mean_x += l->points[i].x;
		mean_y += l->points[i].y;

		sum_sq_x += (l->points[i].x * l->points[i].x);
		sum_xy += (l->points[i].x * l->points[i].y);
	}

	mean_x /= (double) l->points.size();
	mean_y /= (double) l->points.size();

	l->angle = (sum_xy - (sum_x * sum_y) / l->points.size()) / (sum_sq_x - (sum_x * sum_x) / l->points.size());
	l->dist = mean_y - l->angle * mean_x;
}


double
mean_line_residual(Line l)
{
	int i;
	double diff, r = 0;

	if (l.points.size() < 2)
		return 0;

	for (i = 0; i < l.points.size(); i++)
	{
		diff = l.points[i].y - (l.points[i].x * l.angle + l.dist);
		r += fabs(diff);
	}

	return r / (double) l.points.size();
}


void
draw_line(Line l, int line_index, int num_lines)
{
	int i;
	int smaller_x, bigger_x, x, y, first, m, n;

	first = 1;

	for (i = 0; i < l.points.size(); i++)
	{
		if (first)
		{
			smaller_x = l.points[i].x;
			bigger_x = l.points[i].x;
			first = 0;
		}

		if (l.points[i].x < smaller_x)
			smaller_x = l.points[i].x;

		if (l.points[i].x > bigger_x)
			bigger_x = l.points[i].x;
	}

//	float color_r = ((float) rand()) / ((float) RAND_MAX);
//	float color_g = ((float) rand()) / ((float) RAND_MAX);
//	float color_b = ((float) rand()) / ((float) RAND_MAX);

	float color_r = ((float) line_index) / ((float) num_lines);
	float color_g = ((float) line_index) / ((float) num_lines);
	float color_b = ((float) 0.2) / ((float) num_lines);


	for (x = smaller_x; x < bigger_x; x++)
	{
		y = (int) (l.angle * (double) x + l.dist);

		for (m = 0; m < zoom; m++)
		{
			for (n = 0; n < zoom; n++)
			{
				// glColor3f(color_r, color_g, color_b);
				glColor3f(1.0, 0.0, 0.0);
				glVertex2i((x + 30) * ground_map_zoom + m, (y + 30) * ground_map_zoom + n);
			}
		}
	}

//		x = points[i].x;
//		y = points[i].y;
//
//		for (m = 0; m < ground_map_zoom; m++)
//		{
//			for (n = 0; n < ground_map_zoom; n++)
//			{
//				int p = x * ground_map_zoom + m;
//				int q = y * ground_map_zoom + n;
//
//				glColor3f(1.0, 0.0, 0.0);
//				glVertex2i(p + 30 * ground_map_zoom, q + 30 * ground_map_zoom);
//			}
//		}
//	}

}

void
detect_lines_and_draw(vector<GroundPoint> v)
{
	int i;
	Line l;
	GroundPoint p;
	vector<Line> lines;

	double threashold = 0.02;
	int min_num_points_per_line = 50;

	if (v.size() < 2)
		return;

	for (i = 0; i < v.size(); i++)
	{
		printf("x:  %f\ty:  %f\n", v[i].x, v[i].y);

		calculate_line_parameters(&l);

		if (mean_line_residual(l) > threashold)
		{
			lines.push_back(l);
			l.points.clear();
		}
		else
			l.points.push_back(v[i]);
	}

	exit(0);

	int num_valid_lines = 0;

	for (i = 0; i < lines.size(); i++)
	{
		if (lines[i].points.size() > min_num_points_per_line)
		{
			draw_line(lines[i], i, lines.size());
			num_valid_lines++;
		}
	}
}


void
draw_velodyne_projected_on_the_ground()
{
	glBegin(GL_POINTS);

	int i, j;

	for (i = 0; i < 60 * ground_map_zoom; i++)
	{
		for (j = 0; j < 60 * ground_map_zoom; j++)
		{
			glColor3f(1.0, 1.0, 1.0);
			glVertex2i(i, j);
		}
	}

	vector<GroundPoint> v = draw_velodyne_message_projected_on_the_ground(last_velodyne);
	draw_velodyne_message_projected_on_the_ground(current_velodyne);
	draw_landmarks_on_the_ground_map();
	// detect_lines_and_draw(v);

	glEnd();
}


void
draw_velodyne_ground_viewer()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (first_localize_ackerman_globalpos)
	{
		/**
		 * Esse comando eh para colocar a
		 * cor branca na tela inicial, quando
		 * nenhuma mensagem chegou ainda. Sem
		 * ele a tela fica transparente como
		 * se o mundo estivesse vazio.
		 */
		glClear(GL_COLOR_BUFFER_BIT);
	}
	else
	{
		draw_velodyne_projected_on_the_ground();
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
		((double) 430),
		((double) -300),
		((double) 300)
	);
}


void
handle_velodyne_ground_view_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(
		((double) 0),
		((double) 60 * ground_map_zoom),
		((double) 0),
		((double) 60 * ground_map_zoom)
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

	// GROUND VIEWER WINDOW
	glutInitWindowSize(60 * ground_map_zoom, 60 * ground_map_zoom);
	glutPositionWindow(30 * ground_map_zoom, 30 * ground_map_zoom);
	velodyne_ground_view_id = glutCreateWindow("Velodyne Ground Viewer");
	glutReshapeFunc(handle_velodyne_ground_view_resize);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glutDisplayFunc(draw_velodyne_ground_viewer);

	glutTimerFunc(redraw_update_period, Timer, 1);

	srand(time(NULL));
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

	output_filename = argv[1];
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <arquivo-saida-landmarks>\n", argv[0]));

	initialize_ipc(argc, argv);
	read_parameters(argc, argv);
	initialize_module(argc, argv);
	initialize_viewer(argc, argv);
	subcribe_messages();
	glutMainLoop();

	return 0;
}

