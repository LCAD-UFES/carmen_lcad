#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "trajectory_drawer.h"


struct trajectory_drawer
{
	double r;
	double g;
	double b;

	carmen_vector_3D_t* path;
	int path_size;
	
	carmen_pose_3D_t* goals;
	int goals_size;
};


symotha_parameters symotha_params;



trajectory_drawer*
create_trajectory_drawer(double r, double g, double b)
{

	trajectory_drawer* t_drawer = (trajectory_drawer*)malloc(sizeof(trajectory_drawer));
		
	t_drawer->path = NULL;
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
add_trajectory_message(trajectory_drawer* t_drawer, carmen_navigator_ackerman_plan_message *message)
{
	t_drawer->path = (carmen_vector_3D_t*)realloc(t_drawer->path, message->path_length * sizeof(carmen_vector_3D_t));
	t_drawer->path_size = message->path_length;

	int i;
	for(i=0; i<t_drawer->path_size; i++)
	{
		t_drawer->path[i].x = message->path[i].x;
		t_drawer->path[i].y = message->path[i].y;
		t_drawer->path[i].z = 0.0;
	}
}

void
add_goal_list_message(trajectory_drawer* t_drawer, carmen_behavior_selector_goal_list_message *message)
{
	t_drawer->goals = (carmen_pose_3D_t*)realloc(t_drawer->goals, message->size * sizeof(carmen_pose_3D_t));
	t_drawer->goals_size = message->size;

	int i;
	for(i=0; i<t_drawer->goals_size; i++)
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
			
			glColor3f(t_drawer->r, t_drawer->g, t_drawer->b);

			//printf("\nOffset: x:% lf y:% lf z:% lf\n", offset.x, offset.y, offset.z);

			int i;
			for(i=0; i<t_drawer->path_size; i++)
			{				
				//printf("x:% lf y:% lf z:% lf\n", t_drawer->path[i].x, t_drawer->path[i].y, t_drawer->path[i].z);
				glVertex3d(t_drawer->path[i].x - offset.x, t_drawer->path[i].y - offset.y, t_drawer->path[i].z);
			}
		
		glEnd();
		

	glPopMatrix();
}

void drawHollowCircle(GLfloat x, GLfloat y, GLfloat radius){
	int i;
	int lineAmount = 100; //# of triangles used to draw circle

	//GLfloat radius = 0.8f; //radius
	GLfloat twicePi = 2.0f * M_PI;

	glPushMatrix();

		glBegin(GL_LINE_LOOP);
		for(i = 0; i <= lineAmount;i++) {
			glVertex2f(
					x + (radius * cos(i *  twicePi / lineAmount)),
					y + (radius* sin(i * twicePi / lineAmount))
			);
		}
		glEnd();

	glPopMatrix();
}

void
draw_trajectory(trajectory_drawer* t_drawer, carmen_vector_3D_t offset)
{

	drawHollowCircle (car_fused_pose.position.x, car_fused_pose.position.y, symotha_params.central_lane);
	draw_path(t_drawer, offset);
	draw_goals(t_drawer, offset);
}


