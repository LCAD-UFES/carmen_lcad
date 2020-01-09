#include <carmen/carmen.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>


struct symotha_parameters
{
	double main_central_lane;
	double central_lane;
	double lane_safe_dist;
	double obstacles_safe_dist;
};


carmen_param_t param_list[] = {
			{"behavior_selector", "central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.central_lane), 0, NULL},
			{"behavior_selector", "main_central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.main_central_lane), 0, NULL},
			{"behavior_selector", "lateral_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.lane_safe_dist), 0, NULL},
			{"model_predictive_planner", "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &(symotha_params.obstacles_safe_dist), 0, NULL},
	};
	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);


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

	drawHollowCircle (car_fused_pose.position.x, car_fused_pose.position.y, symotha_params.central_lane);
