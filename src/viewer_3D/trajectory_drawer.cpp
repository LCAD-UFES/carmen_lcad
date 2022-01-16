#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/rrt_node.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "trajectory_drawer.h"


trajectory_drawer *
create_trajectory_drawer(double r, double g, double b, carmen_vector_3D_t robot_size, double distance_between_rear_car_and_rear_wheels,
		carmen_semi_trailer_config_t semi_trailer_config, double path_point_size, double persistence_time)
{
	trajectory_drawer *t_drawer = (trajectory_drawer *) malloc(sizeof(trajectory_drawer));
		
	t_drawer->path = NULL;
	t_drawer->path_segment_color = NULL;
	t_drawer->path_size = 0;
	t_drawer->path_point_size = path_point_size;

	t_drawer->goals = NULL;
	t_drawer->goals_size = 0;
	
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
	t_drawer->path = (carmen_robot_and_trailer_pose_t *) realloc(t_drawer->path, message->path_length * sizeof(carmen_robot_and_trailer_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->path_length * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->path_length;

	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].x;
		t_drawer->path[i].y = message->path[i].y;
		t_drawer->path[i].theta = message->path[i].theta;
		t_drawer->path[i].beta = message->path[i].beta;
		set_color(t_drawer, i, message->path[i].v);
	}

	t_drawer->availability_timestamp = carmen_get_time();
}


void
add_base_ackerman_trajectory_message(trajectory_drawer *t_drawer, carmen_base_ackerman_motion_command_message *message)
{
	t_drawer->path = (carmen_robot_and_trailer_pose_t *) realloc(t_drawer->path, message->num_motion_commands * sizeof(carmen_robot_and_trailer_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->num_motion_commands * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->num_motion_commands;

//	FILE *arq = fopen("cacod.txt", "w");
	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->motion_command[i].x;
		t_drawer->path[i].y = message->motion_command[i].y;
		t_drawer->path[i].theta = message->motion_command[i].theta;
		t_drawer->path[i].beta = message->motion_command[i].beta;
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
	t_drawer->path = (carmen_robot_and_trailer_pose_t *) realloc(t_drawer->path, message->size * sizeof(carmen_robot_and_trailer_pose_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->size * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->size;

	for (int i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].p1.x;
		t_drawer->path[i].y = message->path[i].p1.y;
		t_drawer->path[i].theta = message->path[i].p1.theta;
		t_drawer->path[i].beta = message->path[i].p1.beta;
		set_color(t_drawer, i, message->path[i].v);
	}

	t_drawer->availability_timestamp = carmen_get_time();
}


void
add_path_goals_and_annotations_message(trajectory_drawer *t_drawer, carmen_behavior_selector_path_goals_and_annotations_message *message, carmen_vector_3D_t robot_size, double distance_between_rear_car_and_rear_wheels)
{
	t_drawer->path = (carmen_robot_and_trailer_pose_t *) realloc(t_drawer->path, message->number_of_poses * sizeof(carmen_robot_and_trailer_pose_t));
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
		t_drawer->path[i].beta = message->poses[i].beta;
		set_color(t_drawer, i, message->poses[i].v);
//		fprintf(arq, "%lf %lf %lf %lf %lf\n",
//				message->poses[i].x, message->poses[i].y, message->poses[i].theta,
//				message->poses[i].phi, message->poses[i].v);
	}
//	fclose(arq);

	t_drawer->goals = (carmen_pose_3D_t *) realloc(t_drawer->goals, message->goal_list_size * sizeof(carmen_pose_3D_t));
	t_drawer->goals_size = message->goal_list_size;

	for (int i = 0; i < t_drawer->goals_size; i++)
	{
		t_drawer->goals[i].position.x = message->goal_list[i].x;
		t_drawer->goals[i].position.y = message->goal_list[i].y;
		t_drawer->goals[i].position.z = 0.0;

		t_drawer->goals[i].orientation.roll = 0.0;
		t_drawer->goals[i].orientation.pitch = 0.0;
		t_drawer->goals[i].orientation.yaw = message->goal_list[i].theta;
	}

	t_drawer->availability_timestamp = carmen_get_time();
}


void
draw_goals_outline(trajectory_drawer *t_drawer, carmen_vector_3D_t offset)
{
	if (!t_drawer->goals || (t_drawer->goals_size == 0))
		return;

	double length_x = t_drawer->robot_size.x;
	double length_y = t_drawer->robot_size.y;
	double car_middle_to_rear_wheels = length_x / 2.0 - t_drawer->distance_between_rear_car_and_rear_wheels;

	for(int i = 0; i < t_drawer->goals_size; i++)
	{
		glPushMatrix();

			glColor3f(1.0f, 1.0f, 0.0f);
			glTranslated(t_drawer->goals[i].position.x - offset.x, t_drawer->goals[i].position.y - offset.y, t_drawer->goals[i].position.z - offset.z);
			glRotated(carmen_radians_to_degrees(t_drawer->goals[i].orientation.yaw), 0.0f, 0.0f, 1.0f);
			glRotated(carmen_radians_to_degrees(t_drawer->goals[i].orientation.pitch), 0.0f, 1.0f, 0.0f);
			glRotated(carmen_radians_to_degrees(t_drawer->goals[i].orientation.roll), 1.0f, 0.0f, 0.0f);

			glBegin(GL_LINE_STRIP);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, -length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels + length_x/2, -length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels + length_x/2, length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, length_y/2, 0);
				glVertex3d(car_middle_to_rear_wheels - length_x/2, -length_y/2, 0);
			glEnd();

		glPopMatrix();
	}

	if ((carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		t_drawer->goals_size = 0;	// Depois daqui, soh desenha novamente se chegar nova mensagem
}


void
draw_goals(trajectory_drawer *t_drawer, carmen_vector_3D_t offset)
{
	glPushMatrix();

		glColor3f(1.0f, 1.0f, 0.0f);
	
		glBegin(GL_LINES);	

			for (int i = 0; i < t_drawer->goals_size; i++)
			{
				double sinTheta = sin(t_drawer->goals[i].orientation.yaw);
				double cosTheta = cos(t_drawer->goals[i].orientation.yaw);

				glVertex3d(t_drawer->goals[i].position.x - offset.x, t_drawer->goals[i].position.y - offset.y, t_drawer->goals[i].position.z - offset.z);
				glVertex3d(t_drawer->goals[i].position.x - offset.x + 2.0*cosTheta, t_drawer->goals[i].position.y - offset.y + 2.0*sinTheta, t_drawer->goals[i].position.z - offset.z);														
			}

		glEnd();

	glPopMatrix();

	if ((carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		t_drawer->goals_size = 0;	// Depois daqui, soh desenha novamente se chegar nova mensagem
}


static void
draw_path(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int draw_waypoints_flag, int draw_robot_waypoints_flag, int semi_trailer_engaged)
{
	if (!t_drawer->path || (t_drawer->path_size == 0))
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
					glPushMatrix();
						glRotatef(-carmen_radians_to_degrees(t_drawer->path[i].beta), 0.0, 0.0, 1.0);

						glTranslatef(-t_drawer->semi_trailer_config.d - t_drawer->semi_trailer_config.M * cos(t_drawer->path[i].beta),
									 -t_drawer->semi_trailer_config.M * sin(t_drawer->path[i].beta),
									 0.0);

						glBegin(GL_LINE_STRIP);
							glVertex3f(-t_drawer->semi_trailer_config.distance_between_axle_and_back, -t_drawer->semi_trailer_config.width / 2, 0);
							glVertex3f(t_drawer->semi_trailer_config.distance_between_axle_and_front, -t_drawer->semi_trailer_config.width / 2, 0);
							glVertex3f(t_drawer->semi_trailer_config.distance_between_axle_and_front, t_drawer->semi_trailer_config.width / 2, 0);
							glVertex3f(-t_drawer->semi_trailer_config.distance_between_axle_and_back, t_drawer->semi_trailer_config.width / 2, 0);
							glVertex3f(-t_drawer->semi_trailer_config.distance_between_axle_and_back, -t_drawer->semi_trailer_config.width / 2, 0);
						glEnd();

					glPopMatrix();
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

	if ((carmen_get_time() - t_drawer->availability_timestamp) > t_drawer->persistence_time)
		t_drawer->path_size = 0;	// Depois daqui, soh desenha novamente se chegar nova mensagem
}


void
draw_trajectory(trajectory_drawer *t_drawer, carmen_vector_3D_t offset, int draw_waypoints_flag, int draw_robot_waypoints_flag, int semi_trailer_engaged)
{
	draw_path(t_drawer, offset, draw_waypoints_flag, draw_robot_waypoints_flag, semi_trailer_engaged);
//	draw_goals(t_drawer, offset);
	draw_goals_outline(t_drawer, offset);
}
