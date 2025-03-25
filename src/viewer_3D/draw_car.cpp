#include "draw_car.h"
#include <locale.h>


static void
drawBox(double length_x, double length_y, double length_z)
{
	glPushMatrix();

		glBegin(GL_QUADS);


			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);


			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);
			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);
			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);


			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);
			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);


			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);
			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);


			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);
			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);
			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);


			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);


		glEnd();

	glPopMatrix();
}


void
draw_cylinder(GLfloat radius, GLfloat height)
{
	GLfloat angle_stepsize = 2.0 * M_PI / 36.0;

	/** Draw the tube */
	glBegin(GL_QUAD_STRIP);
		GLfloat angle = 0.0;
		while (angle < 2.0 * M_PI)
		{
			GLfloat x = radius * cos(angle);
			GLfloat y = radius * sin(angle);
			glVertex3f(x, y, height / 2.0);
			glVertex3f(x, y, -height / 2.0);
			angle = angle + angle_stepsize;
		}
		glVertex3f(radius, 0.0, height);
		glVertex3f(radius, 0.0, 0.0);
	glEnd();

	/** Draw the circle on top of cylinder */
	glBegin(GL_POLYGON);
		angle = 0.0;
		while (angle < 2.0 * M_PI)
		{
			GLfloat x = radius * cos(angle);
			GLfloat y = radius * sin(angle);
			glVertex3f(x, y, height / 2.0);
			angle = angle + angle_stepsize;
		}
		glVertex3f(radius, 0.0, height);
	glEnd();

	/** Draw the circle on botton of cylinder */
	glBegin(GL_POLYGON);
		angle = 0.0;
		while (angle < 2.0 * M_PI)
		{
			GLfloat x = radius * cos(angle);
			GLfloat y = radius * sin(angle);
			glVertex3f(x, y, -height / 2.0);
			angle = angle + angle_stepsize;
		}
		glVertex3f(radius, 0.0, height);
	glEnd();
}


CarDrawer *
createCarDrawer(int argc, char** argv)
{	
	CarDrawer *carDrawer = (CarDrawer *) malloc(sizeof(CarDrawer));

	int num_items;

	char *carmodel_file = NULL;
	char *semi_trailer_model_file = NULL;

	char *robot_collision_file = NULL;

	carmen_param_t param_list[] =
	{
		{"carmodel", "file_name", CARMEN_PARAM_STRING, &carmodel_file, 0, NULL},
		{"carmodel", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.x), 1, NULL},
		{"carmodel", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.y), 1, NULL},
		{"carmodel", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.z), 1, NULL},
		{"carmodel", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.x), 1, NULL},
		{"carmodel", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.y), 1, NULL},
		{"carmodel", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.z), 1, NULL},
		{"carmodel", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.roll), 1, NULL},
		{"carmodel", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.pitch), 1, NULL},
		{"carmodel", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.yaw), 1, NULL},
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(carDrawer->car_axis_distance), 1, NULL},
		{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &(carDrawer->distance_between_rear_car_and_rear_wheels), 1, NULL},
		{"robot", "wheel_radius", CARMEN_PARAM_DOUBLE, &(carDrawer->car_wheel_radius), 1, NULL},
		{"robot", "length", CARMEN_PARAM_DOUBLE, &(carDrawer->robot_size.x), 1, NULL},
		{"robot", "width", CARMEN_PARAM_DOUBLE, &(carDrawer->robot_size.y), 1, NULL},
		{"robot", "collision_file", CARMEN_PARAM_STRING, &robot_collision_file, 1, NULL},
		{"semi_trailer", "initial_type", CARMEN_PARAM_INT, &(carDrawer->semi_trailer_config.num_semi_trailers), 0, NULL},
		{"sensor", "board_1_x", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.x), 1, NULL},
		{"sensor", "board_1_y", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.y), 1, NULL},
		{"sensor", "board_1_z", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.z), 1, NULL},
		{"sensor", "board_1_roll", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.roll), 1, NULL},
		{"sensor", "board_1_pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.pitch), 1, NULL},
		{"sensor", "board_1_yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.yaw), 1, NULL},

		{"xsens", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.x), 1, NULL},
		{"xsens", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.y), 1, NULL},
		{"xsens", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.z), 1, NULL},
		{"xsens", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.x), 1, NULL},
		{"xsens", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.y), 1, NULL},
		{"xsens", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.z), 1, NULL},
		{"xsens", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.roll), 1, NULL},
		{"xsens", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.pitch), 1, NULL},
		{"xsens", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.yaw), 1, NULL},

		{"laser", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.x), 1, NULL},
		{"laser", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.y), 1, NULL},
		{"laser", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.z), 1, NULL},
		{"velodyne", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.x), 1, NULL},
		{"velodyne", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.y), 1, NULL},
		{"velodyne", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.z), 1, NULL},
		{"velodyne", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.roll), 1, NULL},
		{"velodyne", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.pitch), 1, NULL},
		{"velodyne", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.yaw), 1, NULL}
	};
	
	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carDrawer->robot_collision_config = carmen_collision_detection_get_global_collision_config();

	carDrawer->sensor_box_size = carDrawer->xsens_size;

	printf("FILE: %s\n", carmodel_file);

	if (carmodel_file == NULL)
		carDrawer->carModel = glmReadOBJ("ford_escape_model.obj");
	else
		carDrawer->carModel = glmReadOBJ(carmodel_file);
	glmUnitize(carDrawer->carModel);

//	glmScale(carDrawer->carModel, carDrawer->car_size.x / 2.0);
	glmScaleXYZ(carDrawer->carModel, carDrawer->car_size.x / 2.0, carDrawer->car_size.y / 2.0, carDrawer->car_size.z / 2.0);

	for (int semi_trailer_id=1; semi_trailer_id <= carDrawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
	{
		char semi_trailer_string[256];
		char semi_trailer_model_string[256];

		sprintf(semi_trailer_string, "%s%d", "semi_trailer", semi_trailer_id);
		sprintf(semi_trailer_model_string, "%s%d", "semi_trailer_model", semi_trailer_id);

		carmen_param_t param_list2[] =
		{
			{semi_trailer_string, "d", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].d), 0, NULL},
			{semi_trailer_string, "M", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M), 0, NULL},
			{semi_trailer_string, "width", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width), 0, NULL},
			{semi_trailer_string, "distance_between_axle_and_front", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front), 0, NULL},
			{semi_trailer_string, "distance_between_axle_and_back", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back), 0, NULL},
			{semi_trailer_model_string, "file_name", CARMEN_PARAM_STRING, &semi_trailer_model_file, 0, NULL},
			{semi_trailer_model_string, "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size[semi_trailer_id-1].x), 1, NULL},
			{semi_trailer_model_string, "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size[semi_trailer_id-1].y), 1, NULL},
			{semi_trailer_model_string, "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size[semi_trailer_id-1].z), 1, NULL},
			{semi_trailer_model_string, "x", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].position.x), 1, NULL},
			{semi_trailer_model_string, "y", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].position.y), 1, NULL},
			{semi_trailer_model_string, "z", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].position.z), 1, NULL},
			{semi_trailer_model_string, "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].orientation.roll), 1, NULL},
			{semi_trailer_model_string, "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].orientation.pitch), 1, NULL},
			{semi_trailer_model_string, "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose[semi_trailer_id-1].orientation.yaw), 1, NULL}
		};

		num_items = sizeof(param_list2)/sizeof(param_list2[0]);
		carmen_param_install_params(argc, argv, param_list2, num_items);

		carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].max_beta = carmen_degrees_to_radians(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].max_beta);
//		FIXME Avelino
//		carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].d = carDrawer->robot_collision_config->semi_trailer_d;
//		carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].M = carDrawer->robot_collision_config->semi_trailer_M;

		if (semi_trailer_model_file == NULL)
			carDrawer->semiTrailerModel[semi_trailer_id - 1] = glmReadOBJ("ford_escape_model.obj");
		else
			carDrawer->semiTrailerModel[semi_trailer_id - 1] = glmReadOBJ(semi_trailer_model_file);
		glmUnitize(carDrawer->semiTrailerModel[semi_trailer_id - 1]);

//		glmScale(carDrawer->semiTrailerModel[semi_trailer_id - 1], carDrawer->semi_trailer_size[semi_trailer_id - 1].x / 2.0);
		glmScaleXYZ(carDrawer->semiTrailerModel[semi_trailer_id - 1],
				carDrawer->semi_trailer_size[semi_trailer_id - 1].x / 2.0,
				carDrawer->semi_trailer_size[semi_trailer_id - 1].y / 2.0,
				carDrawer->semi_trailer_size[semi_trailer_id - 1].z / 2.0);
	}

	return (carDrawer);
}

void
calculate_trailer_positions(CarDrawer *carDrawer, double beta[MAX_NUM_TRAILERS], int num_semi_trailers, double positions[MAX_NUM_TRAILERS][3], int drawing_model = 1)
{
//	static double cc = 0.0;
    // Car position
    double car_x = carDrawer->semi_trailer_pose[0].position.x;
    double car_y = carDrawer->semi_trailer_pose[0].position.y;
    double car_z = carDrawer->semi_trailer_pose[0].position.z;

    // Calculate trailer positions
    for (int i = 0; i < num_semi_trailers; i++)
    {
		double semi_trailer_d = carDrawer->semi_trailer_config.semi_trailers[i].d;
		double semi_trailer_M = carDrawer->semi_trailer_config.semi_trailers[i].M;

//	   double prev_semi_trailer_x = (i == 0) ? car_x : -car_x*cos(beta[i]); // Esse funcionou por conta do car_y ser 0.0, mas não tenho certeza quanto se o uso da matriz de rotação abaixo funcionaria para uma situação diferente
//	   double prev_semi_trailer_y = (i == 0) ? car_y : car_x*sin(beta[i]);
		double prev_semi_trailer_x = 0.0;
		double prev_semi_trailer_y = 0.0;
		if (drawing_model)
		{
			prev_semi_trailer_x = (i == 0) ? car_x : (-car_x * cos(beta[i])) + (car_y * sin(beta[i])) + car_x;
			prev_semi_trailer_y = (i == 0) ? car_y : (car_x * sin(beta[i])) - (car_y * cos(beta[i]));
//			cc+=0.01;
//			printf("%lf\n", cc);
		}

		double prev_semi_trailer_z =  car_z ;

		double semi_trailer_x = prev_semi_trailer_x  - (semi_trailer_d + semi_trailer_M * cos(beta[i])) ;
		double semi_trailer_y = prev_semi_trailer_y + semi_trailer_M * sin(beta[i]);
		double semi_trailer_z = prev_semi_trailer_z ;

		positions[i][0] = semi_trailer_x;
		positions[i][1] = semi_trailer_z;
		positions[i][2] = semi_trailer_y;

//        printf("prev_semi_trailer_x: %f, prev_semi_trailer_y: %f, semi_trailer_d: %f, beta[i]: %f, semi_trailer_M: %f, semi_trailer_x: %f, semi_trailer_y: %f\n", prev_semi_trailer_x, prev_semi_trailer_y, semi_trailer_d, beta[i], semi_trailer_M, semi_trailer_x, semi_trailer_y);
    }


}



void
draw_wheel_axis(double wheel_diameter, double wheel_distance)
{	
	glPushMatrix();
		
		double wheel_thickness = 0.2;
						
		drawBox(0.05, wheel_distance, 0.05);

		glPushMatrix();
			glTranslatef(0.0, -wheel_distance/2 + wheel_thickness/2, 0.0);
			drawBox( wheel_diameter, wheel_thickness, wheel_diameter);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.0, wheel_distance/2 - wheel_thickness/2, 0.0);
			drawBox( wheel_diameter, wheel_thickness, wheel_diameter);
		glPopMatrix();

	glPopMatrix();
}


void
draw_collision_range(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta[MAX_NUM_TRAILERS], int semi_trailer_engaged)
{
	glPushMatrix();
		glColor3f (1.0, 0.0, 0.0);
		glTranslatef(pose.position.x, pose.position.y, pose.position.z);
		glRotatef(carmen_radians_to_degrees(pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.roll), 1.0f, 0.0f, 0.0f);
		for (int i = 0; i < carDrawer->robot_collision_config->n_markers; i++)
		{
			glPushMatrix();
				glTranslatef(carDrawer->robot_collision_config->markers[i].x, carDrawer->robot_collision_config->markers[i].y, 0.0);
				draw_cylinder(carDrawer->robot_collision_config->markers[i].radius, 0.0);
			glPopMatrix();
		}

		if (semi_trailer_engaged)
		{
			for (int semi_trailer_id=1; semi_trailer_id <= carDrawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
			{
				double cont_m_and_d_values = 0.0;
				if (semi_trailer_id > 1)
				{
					cont_m_and_d_values += (carDrawer->robot_collision_config->semi_trailer_d[semi_trailer_id-1] - sin(beta[semi_trailer_id-1]) * carDrawer->robot_collision_config->semi_trailer_M[semi_trailer_id-1]);
				}
				glTranslatef(-carDrawer->robot_collision_config->semi_trailer_M[semi_trailer_id-1] - cont_m_and_d_values, 0.0, 0.0);

//				glTranslatef(-carDrawer->robot_collision_config->semi_trailer_M[semi_trailer_id-1], 0.0, 0.0);
				glRotatef(-carmen_radians_to_degrees(beta[semi_trailer_id-1]), 0.0f, 0.0f, 1.0f);
				for (int i = 0; i < carDrawer->robot_collision_config->n_semi_trailer_markers[semi_trailer_id-1]; i++)
				{
					glPushMatrix();
						glTranslatef(carDrawer->robot_collision_config->semi_trailer_markers[semi_trailer_id-1][i].x - carDrawer->robot_collision_config->semi_trailer_d[semi_trailer_id-1],
								carDrawer->robot_collision_config->semi_trailer_markers[semi_trailer_id-1][i].y, 0.0);
						draw_cylinder(carDrawer->robot_collision_config->semi_trailer_markers[semi_trailer_id-1][i].radius, 0.0);
					glPopMatrix();
				}
			}
		}
	glPopMatrix();
}


//static void drawCylinder(double length_x, double length_y, double length_z)
//{
//	glPushMatrix();
//
//		//https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluCylinder.xml
//		GLUquadricObj *quadratic;
//		quadratic = gluNewQuadric();
//		gluCylinder(quadratic,0.0445f,0.0445f,0.14,5,5);
//
//	glPopMatrix();
//
//}


static void
drawOutline(double length_x, double length_y)
{
	glPushMatrix();

		glBegin(GL_LINE_STRIP);

			glVertex3f(-length_x/2, -length_y/2, 0);
			glVertex3f(length_x/2, -length_y/2, 0);
			glVertex3f(length_x/2, length_y/2, 0);
			glVertex3f(-length_x/2, length_y/2, 0);
			glVertex3f(-length_x/2, -length_y/2, 0);

		glEnd();

	glPopMatrix();
}


void
draw_axis(double length)
{
	length = 2*length;

	glPushMatrix();
		
		glColor3f(0.4, 1.0, 0.4);
		drawBox(length, 0.05, 0.05);
		drawBox(0.05, length, 0.05);
		drawBox(0.05, 0.05, length);

		glColor3f(0.0, 0.0, 0.0);
		double l;
		for(l=0; l<length; l+=1.0)
		{	
			glPushMatrix();
				glTranslatef(0.0,0.0,l);			
				drawBox(0.06, 0.06, 0.02);
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0,0.0,-l);			
				drawBox(0.06, 0.06, 0.02);
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0,l,0.0);			
				drawBox(0.06, 0.02, 0.06);
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0,-l,0.0);			
				drawBox(0.06, 0.02, 0.06);
			glPopMatrix();

			glPushMatrix();
				glTranslatef(l,0.0,0.0);			
				drawBox(0.02, 0.06, 0.06);
			glPopMatrix();

			glPushMatrix();
				glTranslatef(-l,0.0,0.0);			
				drawBox(0.02, 0.06, 0.06);
			glPopMatrix();
		}

	glPopMatrix();
}


void
draw_car_outline(CarDrawer *carDrawer, double beta[MAX_NUM_TRAILERS], int semi_trailer_engaged)
{
	// Car
	glPushMatrix();

//		glTranslatef(carDrawer->car_pose.position.x, carDrawer->car_pose.position.y, 0.0);
		glTranslatef(carDrawer->robot_size.x / 2.0 - carDrawer->distance_between_rear_car_and_rear_wheels, 0.0, 0.0);

		glColor3f(0.3,0.3,0.3);

		drawOutline(carDrawer->robot_size.x, carDrawer->robot_size.y);

	glPopMatrix();

	glBegin(GL_LINES);
		glVertex3d(0.0, -carDrawer->robot_size.y / 2, 0.0);
		glVertex3d(0.0, carDrawer->robot_size.y / 2, 0.0);
		glVertex3d(carDrawer->car_axis_distance, -carDrawer->robot_size.y / 2, 0.0);
		glVertex3d(carDrawer->car_axis_distance, carDrawer->robot_size.y / 2, 0.0);
		glVertex3d(carDrawer->car_axis_distance, 0.0, 0.0);
		glVertex3d(carDrawer->robot_size.x, 0.0, 0.0);
	glEnd();

	if (semi_trailer_engaged)
	{
		double trailer_positions[MAX_NUM_TRAILERS][3];
		calculate_trailer_positions(carDrawer, beta, carDrawer->semi_trailer_config.num_semi_trailers, trailer_positions, 0);
		for (int semi_trailer_id=1; semi_trailer_id <= carDrawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
		{
			glPushMatrix();
			for (int i = 0; i < semi_trailer_id; i++)
			{
				glRotatef(-carmen_radians_to_degrees(beta[i]), 0.0, 0.0, 1.0);
				glTranslatef(trailer_positions[i][0], -trailer_positions[i][2], trailer_positions[i][1]);
			}
//				glRotatef(-carmen_radians_to_degrees(beta[semi_trailer_id-1]), 0.0, 0.0, 1.0);
//
//				glTranslatef(-carDrawer->robot_collision_config->semi_trailer_d[semi_trailer_id-1] - carDrawer->robot_collision_config->semi_trailer_M[semi_trailer_id-1] * cos(beta[semi_trailer_id-1]),
//							 -carDrawer->robot_collision_config->semi_trailer_M[semi_trailer_id-1] * sin(beta[semi_trailer_id-1]),
//							 0.0);

				glBegin(GL_LINE_STRIP);
					glVertex3f(-carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
					glVertex3f(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, -carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
					glVertex3f(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front, carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
					glVertex3f(-carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
					glVertex3f(-carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back, -carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0);
				glEnd();

				glBegin(GL_LINES);
					glVertex3d(0.0, -carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0.0);
					glVertex3d(0.0, carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].width / 2, 0.0);
				glEnd();

				glBegin(GL_POINTS);
					glVertex3d(carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id-1].d, 0.0, 0.0);
				glEnd();

			glPopMatrix();
		}
	}
}


void
draw_car(CarDrawer *carDrawer, double beta[MAX_NUM_TRAILERS], int semi_trailer_engaged)
{

	static double previous_size_x = 0.0;
	static double previous_size_y = 0.0;
	static double previous_size_z = 0.0;

//	if (carDrawer->car_size.x != previous_size)
	if ((carDrawer->car_size.x != previous_size_x) || (carDrawer->car_size.y != previous_size_y) || (carDrawer->car_size.z != previous_size_z))
	{
		glmUnitize(carDrawer->carModel);
//		glmScale(carDrawer->carModel, carDrawer->car_size.x / 2.0);
		glmScaleXYZ(carDrawer->carModel, carDrawer->car_size.x / 2.0, carDrawer->car_size.y / 2.0, carDrawer->car_size.z / 2.0);
		previous_size_x = carDrawer->car_size.x;
		previous_size_y = carDrawer->car_size.y;
		previous_size_z = carDrawer->car_size.z;
	}

	//draw_axis(500.0);

	// Car
	glPushMatrix();

		glTranslatef(carDrawer->car_pose.position.x, carDrawer->car_pose.position.y, carDrawer->car_pose.position.z + carDrawer->car_wheel_radius);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glRotatef(-carmen_radians_to_degrees(carDrawer->car_pose.orientation.yaw), 0.0, 1.0, 0.0);
		glRotatef(0.0, 0.0, 0.0, 1.0);

		glColor3f(0.3, 0.3, 0.3);
		glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);
		
	glPopMatrix();
	
//	if (semi_trailer_engaged)
//	{
//
//		for (int semi_trailer_id = 1; semi_trailer_id <= carDrawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
//		{
//			// Semi-trailer
//			glPushMatrix();
//
//				glRotatef(90.0, 1.0, 0.0, 0.0);
//				glRotatef(-carmen_radians_to_degrees(beta[semi_trailer_id-1]), 0.0, 1.0, 0.0);
//				glRotatef(0.0, 0.0, 0.0, 1.0);
//				glTranslatef(carDrawer->semi_trailer_pose[semi_trailer_id-1].position.x - carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id - 1].d - carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id - 1].M * cos(beta[semi_trailer_id-1]),
//							 carDrawer->semi_trailer_pose[semi_trailer_id-1].position.z,
//							 carDrawer->semi_trailer_pose[semi_trailer_id-1].position.y + carDrawer->semi_trailer_config.semi_trailers[semi_trailer_id - 1].M * sin(beta[semi_trailer_id-1]));
//
//				glColor3f(0.3, 0.3, 0.3);
//				glmDraw(carDrawer->semiTrailerModel[semi_trailer_id-1], GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);
//
//			glPopMatrix();
//		}
//	}

	if (semi_trailer_engaged)
	{
		double trailer_positions[MAX_NUM_TRAILERS][3];
		calculate_trailer_positions(carDrawer, beta, carDrawer->semi_trailer_config.num_semi_trailers, trailer_positions);
	    for (int semi_trailer_id = 1; semi_trailer_id <= carDrawer->semi_trailer_config.num_semi_trailers; semi_trailer_id++)
	    {
	        // Semi-trailer
	        glPushMatrix();

	        glRotatef(0.0, 0.0, 0.0, 1.0);
			glRotatef(90.0, 1.0, 0.0, 0.0);
			for (int i = 0; i < semi_trailer_id; i++)
			{
				glRotatef(-carmen_radians_to_degrees(beta[i]), 0.0, 1.0, 0.0);
				glTranslatef(trailer_positions[i][0], trailer_positions[i][1], trailer_positions[i][2]);
			}

			glColor3f(0.3, 0.3, 0.3);
			glmDraw(carDrawer->semiTrailerModel[semi_trailer_id-1], GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);

	        glPopMatrix();
	    }
	}

	// Sensor Board
	glPushMatrix();

		glTranslatef(carDrawer->sensor_board_1_pose.position.x,carDrawer->sensor_board_1_pose.position.y,carDrawer->sensor_board_1_pose.position.z);
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		// SensorBox
		glPushMatrix();

			glColor3f(0.0, 0.0, 8.0);
			glTranslatef(0.15, 0.0, 0.0);
			drawBox(2.0 * 0.15, 2.0 * 0.15, 0.03);

		glPopMatrix();

		// Xsens
		glPushMatrix();

			glTranslatef(carDrawer->xsens_pose.position.x,carDrawer->xsens_pose.position.y,carDrawer->xsens_pose.position.z);
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.roll), 1.0f, 0.0f, 0.0f);

			glColor3f(1.0,0.6,0.0);
			drawBox(carDrawer->xsens_size.x, carDrawer->xsens_size.y, carDrawer->xsens_size.z);

		glPopMatrix();

		// Velodyne
		glPushMatrix();

			glTranslatef(carDrawer->laser_pose.position.x, carDrawer->laser_pose.position.y, carDrawer->laser_pose.position.z);
			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);
			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.roll), 1.0f, 0.0f, 0.0f);

			glColor3f(0.0,0.0,1.0);
			draw_cylinder(sqrt(pow(carDrawer->laser_size.x, 2.0) + pow(carDrawer->laser_size.y, 2.0)), carDrawer->laser_size.z);
//			drawBox(carDrawer->laser_size.x, carDrawer->laser_size.y, carDrawer->laser_size.z);

		glPopMatrix();

	glPopMatrix();

}


void
draw_car_at_pose(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta[MAX_NUM_TRAILERS], int semi_trailer_engaged)
{
	glPushMatrix();
		glTranslatef(pose.position.x, pose.position.y, pose.position.z);
		glRotatef(carmen_radians_to_degrees(pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		draw_car(carDrawer, beta, semi_trailer_engaged);
	glPopMatrix();
}


void
draw_car_outline_at_pose(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta[MAX_NUM_TRAILERS], int semi_trailer_engaged)
{
//	printf("car x %lf, car y %lf\n", pose.position.x, pose.position.y);
	glPushMatrix();
		glTranslatef(pose.position.x, pose.position.y, pose.position.z);
		glRotatef(carmen_radians_to_degrees(pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		draw_car_outline(carDrawer, beta, semi_trailer_engaged);
	glPopMatrix();
}


void
destroyCarDrawer(CarDrawer *carDrawer)
{
	glmDelete(carDrawer->carModel);
	free(carDrawer->robot_collision_config->markers);
	free(carDrawer->robot_collision_config->semi_trailer_markers);
	free(carDrawer);
}
