#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/rrt_node.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/freeglut_ext.h>	// glutStrokeString, como no viewer do fork

#include "trajectory_drawer.h"


// o contorno do goal fica um centimetro abaixo do plano do caminho, como no fork
#define GOAL_Z_OFFSET	-0.01


trajectory_drawer *
create_trajectory_drawer(double r, double g, double b, carmen_vector_3D_t robot_size, double distance_between_rear_car_and_rear_wheels,
		carmen_semi_trailers_config_t semi_trailer_config, double path_point_size, double persistence_time)
{
	trajectory_drawer *t_drawer = (trajectory_drawer *) malloc(sizeof(trajectory_drawer));
		
	t_drawer->path = NULL;
	t_drawer->path_segment_color = NULL;
	t_drawer->path_size = 0;
	t_drawer->path_point_size = path_point_size;

	t_drawer->goals = NULL;
	t_drawer->goals_size = 0;
	t_drawer->first_goal_velocity = 0.0;
	t_drawer->force_draw = 0;
	
	t_drawer->r = r;
	t_drawer->g = g;
	t_drawer->b = b;

	t_drawer->robot_size = robot_size;
	t_drawer->distance_between_rear_car_and_rear_wheels = distance_between_rear_car_and_rear_wheels;

	t_drawer->semi_trailer_config = semi_trailer_config;

	t_drawer->persistence_time = persistence_time;

	return t_drawer;
}


void
destroy_trajectory_drawer(trajectory_drawer *t_drawer)
{
	free(t_drawer->path);
	free(t_drawer->goals);
	free(t_drawer->path_segment_color);
	free(t_drawer);
}


void
set_color(trajectory_drawer *t_drawer, int i, double v_i)
{
	if (((i == 0) && (v_i < 0.0)) || ((i == t_drawer->path_size - 1) && (v_i < 0.0)))
	{
		t_drawer->path_segment_color[i].x = 1.0;
		t_drawer->path_segment_color[i].y = 0.0;
		t_drawer->path_segment_color[i].z = 0.0;
	}
	else
	{
		t_drawer->path_segment_color[i].x = t_drawer->r;
		t_drawer->path_segment_color[i].y = t_drawer->g;
		t_drawer->path_segment_color[i].z = t_drawer->b;
	}
}


void
add_trajectory_message(trajectory_drawer *t_drawer, carmen_navigator_ackerman_plan_message *message)
{
	t_drawer->path = (carmen_robot_and_trailers_pose_t *) realloc(t_drawer->path, message->path_length * sizeof(carmen_robot_and_trailers_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->path_length * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->path_length;

	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].x;
		t_drawer->path[i].y = message->path[i].y;
		t_drawer->path[i].theta = message->path[i].theta;
		t_drawer->path[i].num_trailers = message->path[i].num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			t_drawer->path[i].trailer_theta[z] = message->path[i].trailer_theta[z];
		set_color(t_drawer, i, message->path[i].v);
	}

	t_drawer->availability_timestamp = carmen_get_time();
}


void
add_base_ackerman_trajectory_message(trajectory_drawer *t_drawer, carmen_base_ackerman_motion_command_message *message)
{
	t_drawer->path = (carmen_robot_and_trailers_pose_t *) realloc(t_drawer->path, message->num_motion_commands * sizeof(carmen_robot_and_trailers_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->num_motion_commands * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->num_motion_commands;

//	FILE *arq = fopen("cacod.txt", "w");
	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->motion_command[i].x;
		t_drawer->path[i].y = message->motion_command[i].y;
		t_drawer->path[i].theta = message->motion_command[i].theta;
		t_drawer->path[i].num_trailers = message->motion_command[i].num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			t_drawer->path[i].trailer_theta[z] = message->motion_command[i].trailer_theta[z];
		set_color(t_drawer, i, message->motion_command[i].v);
//		fprintf(arq, "%lf %lf %lf %lf %lf\n",
//				message->motion_command[i].x, message->motion_command[i].y, message->motion_command[i].theta,
//				message->motion_command[i].phi, message->motion_command[i].v);
	}
//	fclose(arq);

	t_drawer->availability_timestamp = carmen_get_time();
}


void
add_rrt_trajectory_message(trajectory_drawer *t_drawer, rrt_path_message *message)
{
	t_drawer->path = (carmen_robot_and_trailers_pose_t *) realloc(t_drawer->path, message->size * sizeof(carmen_robot_and_trailers_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->size * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->size;

	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].p1.x;
		t_drawer->path[i].y = message->path[i].p1.y;
		t_drawer->path[i].theta = message->path[i].p1.theta;
		t_drawer->path[i].num_trailers = message->path[i].p1.num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			t_drawer->path[i].trailer_theta[z] = message->path[i].p1.trailer_theta[z];
		set_color(t_drawer, i, message->path[i].v);
	}

	t_drawer->availability_timestamp = carmen_get_time();
}


void
add_path_goals_and_annotations_message(trajectory_drawer *t_drawer, carmen_behavior_selector_path_goals_and_annotations_message *message, carmen_vector_3D_t robot_size, double distance_between_rear_car_and_rear_wheels)
{
	t_drawer->path = (carmen_robot_and_trailers_pose_t *) realloc(t_drawer->path, message->number_of_poses * sizeof(carmen_robot_and_trailers_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->number_of_poses * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->number_of_poses;

	t_drawer->robot_size = robot_size;
	t_drawer->distance_between_rear_car_and_rear_wheels = distance_between_rear_car_and_rear_wheels;

//	FILE *arq = fopen("cacod.txt", "w");
	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->poses[i].x;
		t_drawer->path[i].y = message->poses[i].y;
		t_drawer->path[i].theta = message->poses[i].theta;
		t_drawer->path[i].num_trailers = message->poses[i].num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			t_drawer->path[i].trailer_theta[z] = message->poses[i].trailer_theta[z];
		set_color(t_drawer, i, message->poses[i].v);
//		fprintf(arq, "%lf %lf %lf %lf %lf\n",
//				message->poses[i].x, message->poses[i].y, message->poses[i].theta,
//				message->poses[i].phi, message->poses[i].v);
	}
//	fclose(arq);

	t_drawer->goals = (carmen_robot_and_trailers_pose_t *) realloc(t_drawer->goals, message->goal_list_size * sizeof(carmen_robot_and_trailers_pose_t));
	t_drawer->goals_size = message->goal_list_size;

	for (int i = 0; i < t_drawer->goals_size; i++)
	{
		t_drawer->goals[i].x = message->goal_list[i].x;
		t_drawer->goals[i].y = message->goal_list[i].y;
		t_drawer->goals[i].theta = message->goal_list[i].theta;
		t_drawer->goals[i].num_trailers = message->goal_list[i].num_trailers;

		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			t_drawer->goals[i].trailer_theta[z] = message->goal_list[i].trailer_theta[z];
	}

	t_drawer->first_goal_velocity = (t_drawer->goals_size > 0) ? message->goal_list[0].v : 0.0;

	t_drawer->availability_timestamp = carmen_get_time();
}


// Como no viewer do fork (astro/src/viewer_3D/trajectory_drawer.cpp:279): desenha SO O GOAL
// CORRENTE, e como contorno. O original percorria todos os goals e pintava cada um como um
// GL_POLYGON amarelo opaco do tamanho do carro -- numa rota com 20 goals sao 20 caixas macicas,
// cada uma cobrindo os quatro metros seguintes de rota.
void
draw_goals_outline(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int semi_trailer_engaged)
{
	if (!t_drawer->goals || (t_drawer->goals_size == 0))
		return;

	if (!t_drawer->force_draw && (carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		return;		// soh desenha novamente se chegar nova mensagem

	double length_x = t_drawer->robot_size.x;
	double length_y = t_drawer->robot_size.y;
	double car_middle_to_rear_wheels = length_x / 2.0 - t_drawer->distance_between_rear_car_and_rear_wheels;

	for (int i = 0; (i < 1) && (i < t_drawer->goals_size); i++)	// (i < 1): so o goal corrente
	{
		glDisable(GL_LIGHTING);
		glPushMatrix();

			glTranslatef(t_drawer->goals[i].x - offset.x, t_drawer->goals[i].y - offset.y, GOAL_Z_OFFSET);
			glRotatef(carmen_radians_to_degrees(t_drawer->goals[i].theta), 0.0, 0.0, 1.0);

			glColor3f(1.0f, 1.0f, 0.0f);
			glBegin(GL_LINE_STRIP);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, -length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels + length_x/2, -length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels + length_x/2, length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, -length_y/2, 0);
			glEnd();

			// os semi-reboques do goal, encadeados como no draw_path() logo abaixo (o fork usa
			// convert_theta1_to_beta(), que nao existe aqui; a formulacao com d e M e' a que o
			// proprio carmen ja usa para o caminho, entao o goal fica igual ao resto do desenho)
			if (semi_trailer_engaged)
			{
				for (int semi_trailer_id = 1; semi_trailer_id <= t_drawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
				{
					glPushMatrix();
						glRotatef(-carmen_radians_to_degrees(t_drawer->goals[i].trailer_theta[semi_trailer_id-1]), 0.0, 0.0, 1.0);
						glTranslatef(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].d - t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M * cos(t_drawer->goals[i].trailer_theta[semi_trailer_id-1]),
									 -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M * sin(t_drawer->goals[i].trailer_theta[semi_trailer_id-1]),
									 0.0);

						glBegin(GL_LINE_STRIP);
							glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
							glVertex3f(t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
							glVertex3f(t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
							glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
							glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
						glEnd();
					glPopMatrix();
				}
			}

			// a velocidade pedida no goal, escrita dentro do contorno (fork:369)
			{
				double text_height = 0.65;
				double text_pos_x = car_middle_to_rear_wheels - length_x / 2;
				double text_pos_y = (length_y / 2) - text_height;
				char text[64];

				glPushMatrix();
					glColor3f(0.0f, 0.0f, 0.0f);
					glTranslatef(text_pos_x, text_pos_y, 0.0);
					glScalef(0.0045f, 0.0045f, 1.0f);
					sprintf(text, "%5.1fkm/h", 3.6 * t_drawer->first_goal_velocity);
					glutStrokeString(GLUT_STROKE_MONO_ROMAN, (const unsigned char *) text);
				glPopMatrix();
			}

		glPopMatrix();
		glEnable(GL_LIGHTING);
	}
}


void
draw_goals(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int semi_trailer_engaged)
{
	if (!t_drawer->force_draw && (carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		return;		// soh desenha novamente se chegar nova mensagem

	glPushMatrix();

		glColor3f(1.0f, 1.0f, 0.0f);

		glBegin(GL_LINES);

			for (int i = 0; (i < 1) && (i < t_drawer->goals_size); i++)	// (i < 1): so o goal corrente
			{
				double sinTheta = sin(t_drawer->goals[i].theta);
				double cosTheta = cos(t_drawer->goals[i].theta);

				glVertex3d(t_drawer->goals[i].x - offset.x, t_drawer->goals[i].y - offset.y, 0.0);
				glVertex3d(t_drawer->goals[i].x - offset.x + 2.0 * cosTheta, t_drawer->goals[i].y - offset.y + 2.0 * sinTheta, 0.0);
			}

		glEnd();

	glPopMatrix();

	draw_goals_outline(t_drawer, offset, semi_trailer_engaged);
}


static void
draw_path(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int draw_waypoints_flag, int draw_robot_waypoints_flag, int semi_trailer_engaged)
{
	if (!t_drawer->path || (t_drawer->path_size == 0))
		return;

	// O teste vem AQUI, e so' impede o desenho -- como no fork. No original ele ficava no fim da
	// funcao e fazia t_drawer->path_size = 0, ou seja, APAGAVA o caminho: um unico quadro
	// atrasado alem do persistence_time e a rota so' voltava com a proxima mensagem.
	if (!t_drawer->force_draw && (carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		return;

	if (draw_robot_waypoints_flag)
	{
		for (int i = 0; i < t_drawer->path_size; i++)
		{
			glPushMatrix();
				glTranslatef(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, 0.0);
				glRotatef(carmen_radians_to_degrees(t_drawer->path[i].theta), 0.0, 0.0, 1.0);

				glColor3f(t_drawer->path_segment_color[i].x, t_drawer->path_segment_color[i].y, t_drawer->path_segment_color[i].z);
				glBegin(GL_LINE_STRIP);
					glVertex3f(-t_drawer->distance_between_rear_car_and_rear_wheels, -t_drawer->robot_size.y / 2, 0);
					glVertex3f(t_drawer->robot_size.x - t_drawer->distance_between_rear_car_and_rear_wheels, -t_drawer->robot_size.y / 2, 0);
					glVertex3f(t_drawer->robot_size.x - t_drawer->distance_between_rear_car_and_rear_wheels, t_drawer->robot_size.y / 2, 0);
					glVertex3f(-t_drawer->distance_between_rear_car_and_rear_wheels, t_drawer->robot_size.y / 2, 0);
					glVertex3f(-t_drawer->distance_between_rear_car_and_rear_wheels, -t_drawer->robot_size.y / 2, 0);
				glEnd();

				if (semi_trailer_engaged)
				{
					for (int semi_trailer_id=1; semi_trailer_id <= t_drawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
					{
						glPushMatrix();
							glRotatef(-carmen_radians_to_degrees(t_drawer->path[i].trailer_theta[semi_trailer_id-1]), 0.0, 0.0, 1.0);

							glTranslatef(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].d - t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M * cos(t_drawer->path[i].trailer_theta[semi_trailer_id-1]),
										 -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M * sin(t_drawer->path[i].trailer_theta[semi_trailer_id-1]),
										 0.0);

							glBegin(GL_LINE_STRIP);
								glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
								glVertex3f(t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
								glVertex3f(t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
								glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
								glVertex3f(-t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -t_drawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
							glEnd();

						glPopMatrix();
					}
				}
			glPopMatrix();
		}
	}
	else
	{
		glPushMatrix();

			glBegin(GL_LINE_STRIP);

//				glColor3f(t_drawer->r, t_drawer->g, t_drawer->b);

				for (int i = 0; i < t_drawer->path_size; i++)
				{
					glColor3f(t_drawer->path_segment_color[i].x, t_drawer->path_segment_color[i].y, t_drawer->path_segment_color[i].z);
					glVertex3d(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, 0.0);
				}

			glEnd();

			if (draw_waypoints_flag)
			{
				glPointSize (t_drawer->path_point_size);

				glBegin (GL_POINTS);
					for (int i = 0; i < t_drawer->path_size; i++)
					{
						glColor3f(t_drawer->path_segment_color[i].x, t_drawer->path_segment_color[i].y, t_drawer->path_segment_color[i].z);
						glVertex3d(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, 0.0);
					}
				glEnd ();
				glPointSize (1.0);
			}

		glPopMatrix();
	}
}


void
draw_trajectory(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int draw_waypoints_flag, int draw_robot_waypoints_flag, int semi_trailer_engaged, int force_draw)
{
	if (t_drawer == NULL)
		return;

	t_drawer->force_draw = force_draw;

	draw_path(t_drawer, offset, draw_waypoints_flag, draw_robot_waypoints_flag, semi_trailer_engaged);
	draw_goals(t_drawer, offset, semi_trailer_engaged);
}
