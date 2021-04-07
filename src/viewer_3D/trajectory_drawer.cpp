#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "trajectory_drawer.h"


double car_middle_to_rear_wheels = 4.437 / 2 - 0.96;

trajectory_drawer*
create_trajectory_drawer(double r, double g, double b)
{

	trajectory_drawer* t_drawer = (trajectory_drawer*)malloc(sizeof(trajectory_drawer));

//	int num_items;
//
//	carmen_param_t param_list[] = {
//	{"carmodel", "size_x", CARMEN_PARAM_DOUBLE, &(t_drawer->car_size.x), 0, NULL},
//	{"carmodel", "size_y", CARMEN_PARAM_DOUBLE, &(t_drawer->car_size.y), 0, NULL},
//	{"carmodel", "size_z", CARMEN_PARAM_DOUBLE, &(t_drawer->car_size.z), 0, NULL}
//	};
//
//	num_items = sizeof(param_list)/sizeof(param_list[0]);
//	carmen_param_install_params(1, NULL, param_list, num_items);

	t_drawer->car_size.x = 4.437;
	t_drawer->car_size.y = 1.806;
	t_drawer->car_size.y = 1.725;
		
	t_drawer->path = NULL;
	t_drawer->path_segment_color = NULL;
	t_drawer->path_size = 0;

	t_drawer->goals = NULL;
	t_drawer->goals_size = 0;
	
	t_drawer->r = r;
	t_drawer->g = g;
	t_drawer->b = b;

	return t_drawer;
}


void
destroy_trajectory_drawer(trajectory_drawer* t_drawer)
{
	free(t_drawer->path);
	free(t_drawer->goals);
	free(t_drawer);
}

void
add_trajectory_message(trajectory_drawer *t_drawer, carmen_navigator_ackerman_plan_message *message)
{
	t_drawer->path = (carmen_vector_3D_t *) realloc(t_drawer->path, message->path_length * sizeof(carmen_vector_3D_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->path_length * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->path_length;

	int i;
	for(i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].x;
		t_drawer->path[i].y = message->path[i].y;
		t_drawer->path[i].z = 0.0;

		if ((i == t_drawer->path_size - 1) || (message->path[i + 1].v >= 0.0))
		{
			t_drawer->path_segment_color[i].x = t_drawer->r;
			t_drawer->path_segment_color[i].y = t_drawer->g;
			t_drawer->path_segment_color[i].z = t_drawer->b;
		}
		else
		{
			t_drawer->path_segment_color[i].x = 1.0;
			t_drawer->path_segment_color[i].y = 0.0;
			t_drawer->path_segment_color[i].z = 0.0;
		}
	}
}

void
add_path_goals_and_annotations_message(trajectory_drawer *t_drawer, carmen_behavior_selector_path_goals_and_annotations_message *message)
{
	t_drawer->path = (carmen_vector_3D_t *) realloc(t_drawer->path, message->number_of_poses * sizeof(carmen_vector_3D_t));
	t_drawer->path_segment_color = (carmen_vector_3D_t *) realloc(t_drawer->path_segment_color, message->number_of_poses * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->number_of_poses;

	int i;
	for(i = 0; i < t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->poses[i].x;
		t_drawer->path[i].y = message->poses[i].y;
		t_drawer->path[i].z = 0.0;

		if ((i == t_drawer->path_size - 1) || (message->poses[i + 1].v >= 0.0))
		{
			t_drawer->path_segment_color[i].x = t_drawer->r;
			t_drawer->path_segment_color[i].y = t_drawer->g;
			t_drawer->path_segment_color[i].z = t_drawer->b;
		}
		else
		{
			t_drawer->path_segment_color[i].x = 1.0;
			t_drawer->path_segment_color[i].y = 0.0;
			t_drawer->path_segment_color[i].z = 0.0;
		}
	}

	t_drawer->goals = (carmen_pose_3D_t *) realloc(t_drawer->goals, message->goal_list_size * sizeof(carmen_pose_3D_t));
	t_drawer->goals_size = message->goal_list_size;

	for(i = 0; i < t_drawer->goals_size; i++)
	{
		t_drawer->goals[i].position.x = message->goal_list[i].x;
		t_drawer->goals[i].position.y = message->goal_list[i].y;
		t_drawer->goals[i].position.z = 0.0;

		t_drawer->goals[i].orientation.roll = 0.0;
		t_drawer->goals[i].orientation.pitch = 0.0;
		t_drawer->goals[i].orientation.yaw = message->goal_list[i].theta;
	}
}


static void
draw_goals_outline(trajectory_drawer* t_drawer, carmen_vector_3D_t offset)
{
	double length_x = t_drawer->car_size.x;
	double length_y = t_drawer->car_size.y;

	int i;
	for(i=0; i<t_drawer->goals_size; i++)
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
}


static void
draw_goals(trajectory_drawer* t_drawer, carmen_vector_3D_t offset)
{

	glPushMatrix();

		glColor3f(1.0f, 1.0f, 0.0f);
	
		glBegin(GL_LINES);	

			int i;
			for(i=0; i<t_drawer->goals_size; i++)
			{
				double sinTheta = sin(t_drawer->goals[i].orientation.yaw);
				double cosTheta = cos(t_drawer->goals[i].orientation.yaw);

				glVertex3d(t_drawer->goals[i].position.x - offset.x, t_drawer->goals[i].position.y - offset.y, t_drawer->goals[i].position.z - offset.z);
				glVertex3d(t_drawer->goals[i].position.x - offset.x + 2.0*cosTheta, t_drawer->goals[i].position.y - offset.y + 2.0*sinTheta, t_drawer->goals[i].position.z - offset.z);														
			}

		glEnd();

	glPopMatrix();
}

static void
draw_path(trajectory_drawer* t_drawer, carmen_vector_3D_t offset)
{
	glPushMatrix();

		glBegin(GL_LINE_STRIP);

//			glColor3f(t_drawer->r, t_drawer->g, t_drawer->b);

			int i;
			for (i = 0; i < t_drawer->path_size; i++)
			{
				glColor3f(t_drawer->path_segment_color[i].x, t_drawer->path_segment_color[i].y, t_drawer->path_segment_color[i].z);
				glVertex3d(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, t_drawer->path[i].z);
			}

		glEnd();

		glPointSize (5.0);
		glBegin (GL_POINTS);
			for (i = 0; i < t_drawer->path_size; i++)
			{
				glColor3f(t_drawer->path_segment_color[i].x, t_drawer->path_segment_color[i].y, t_drawer->path_segment_color[i].z);
				glVertex3d(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, t_drawer->path[i].z);
			}
		glEnd ();
		glPointSize (1.0);

	glPopMatrix();



}

void
draw_trajectory(trajectory_drawer* t_drawer, carmen_vector_3D_t offset)
{
	draw_path(t_drawer, offset);
//	draw_goals(t_drawer, offset);
	draw_goals_outline(t_drawer, offset);
}
