
#include "draw_car.h"

struct CarDrawer
{
	carmen_vector_3D_t car_size;
	carmen_pose_3D_t car_pose;
	double car_axis_distance;
	double car_wheel_diameter;

	carmen_pose_3D_t sensor_board_1_pose;

	carmen_vector_3D_t laser_size;
	carmen_pose_3D_t laser_pose;
	
	carmen_vector_3D_t sensor_box_size;
	carmen_pose_3D_t sensor_box_pose;

	carmen_vector_3D_t xsens_size;
	carmen_pose_3D_t xsens_pose;	

	GLMmodel* carModel;
};

static void drawBox(double length_x, double length_y, double length_z);


CarDrawer* createCarDrawer(int argc, char** argv)
{	
	CarDrawer* carDrawer = (CarDrawer*)malloc(sizeof(CarDrawer));

	int num_items;

	char* model_file = NULL;

	carmen_param_t param_list[] = {
	{"carmodel", "file_name", CARMEN_PARAM_STRING, &model_file, 0, NULL},
	{"carmodel", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.x), 0, NULL},
	{"carmodel", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.y), 0, NULL},
	{"carmodel", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.z), 0, NULL},
	{"carmodel", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.x), 0, NULL},
	{"carmodel", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.y), 0, NULL},
	{"carmodel", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.z), 0, NULL},
	{"carmodel", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.roll), 0, NULL},
	{"carmodel", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.pitch), 0, NULL},
	{"carmodel", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.yaw), 0, NULL},
	{"car", "axis_distance", CARMEN_PARAM_DOUBLE, &(carDrawer->car_axis_distance), 0, NULL},
	{"car", "wheel_diameter", CARMEN_PARAM_DOUBLE, &(carDrawer->car_wheel_diameter), 0, NULL},

	{"sensor_board_1", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.x), 0, NULL},
	{"sensor_board_1", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.y), 0, NULL},
	{"sensor_board_1", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.position.z), 0, NULL},
	{"sensor_board_1", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.roll), 0, NULL},
	{"sensor_board_1", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.pitch), 0, NULL},
	{"sensor_board_1", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_board_1_pose.orientation.yaw), 0, NULL},

//	{"sensor_box", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_size.x), 0, NULL},
//	{"sensor_box", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_size.y), 0, NULL},
//	{"sensor_box", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_size.z), 0, NULL},
//	{"sensor_box", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.position.x), 0, NULL},
//	{"sensor_box", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.position.y), 0, NULL},
//	{"sensor_box", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.position.z), 0, NULL},
//	{"sensor_box", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.orientation.roll), 0, NULL},
//	{"sensor_box", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.orientation.pitch), 0, NULL},
//	{"sensor_box", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->sensor_box_pose.orientation.yaw), 0, NULL},

	{"xsens", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.x), 0, NULL},
	{"xsens", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.y), 0, NULL},
	{"xsens", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_size.z), 0, NULL},
	{"xsens", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.x), 0, NULL},
	{"xsens", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.y), 0, NULL},
	{"xsens", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.position.z), 0, NULL},
	{"xsens", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.roll), 0, NULL},
	{"xsens", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.pitch), 0, NULL},
	{"xsens", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->xsens_pose.orientation.yaw), 0, NULL},	

	{"laser", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.x), 0, NULL},
	{"laser", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.y), 0, NULL},
	{"laser", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_size.z), 0, NULL},
	{"velodyne", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.x), 0, NULL},
	{"velodyne", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.y), 0, NULL},
	{"velodyne", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.position.z), 0, NULL},
	{"velodyne", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.roll), 0, NULL},
	{"velodyne", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.pitch), 0, NULL},
	{"velodyne", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->laser_pose.orientation.yaw), 0, NULL}
	};
	
	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	printf("FILE: %s\n", model_file);

	if (model_file == NULL)
		carDrawer->carModel = glmReadOBJ("ford_escape_model.obj");
	else
		carDrawer->carModel = glmReadOBJ(model_file);
	glmUnitize(carDrawer->carModel);

	glmScale(carDrawer->carModel, carDrawer->car_size.x/2.0);

	return carDrawer;
}


void draw_wheel_axis(double wheel_diameter, double wheel_distance)
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


static void draw_cylinder(GLfloat radius, GLfloat height)
{
    GLfloat x              = 0.0;
    GLfloat y              = 0.0;
    GLfloat angle          = 0.0;
    GLfloat angle_stepsize = 0.1;

    /** Draw the tube */
    glPushMatrix();

		glBegin(GL_QUAD_STRIP);
		angle = 0.0;
			while( angle < 2*M_PI ) {
				x = radius * cos(angle);
				y = radius * sin(angle);
				glVertex3f(x, y , height);
				glVertex3f(x, y , 0.0);
				angle = angle + angle_stepsize;
			}
			glVertex3f(radius, 0.0, height);
			glVertex3f(radius, 0.0, 0.0);
		glEnd();

		/** Draw the circle on top of cylinder */
		glBegin(GL_POLYGON);
		angle = 0.0;
			while( angle < 2*M_PI ) {
				x = radius * cos(angle);
				y = radius * sin(angle);
				glVertex3f(x, y , height);
				angle = angle + angle_stepsize;
			}
			glVertex3f(radius, 0.0, height);
		glEnd();

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

static void drawBox(double length_x, double length_y, double length_z)
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

void draw_axis(double length)
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


void draw_car(CarDrawer* carDrawer)
{
	//draw_axis(500.0);

	// Car
	glPushMatrix();

		glTranslatef(carDrawer->car_pose.position.x,carDrawer->car_pose.position.y,carDrawer->car_pose.position.z+carDrawer->car_wheel_diameter/2);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glRotatef(0.0, 0.0, 1.0, 0.0);

		glColor3f(0.3,0.3,0.3);
		//glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR);
		glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);
		
		
	glPopMatrix();
	
	
	// Sensor Board
	glPushMatrix();

		glTranslatef(carDrawer->sensor_board_1_pose.position.x,carDrawer->sensor_board_1_pose.position.y,carDrawer->sensor_board_1_pose.position.z);
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);		
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(carDrawer->sensor_board_1_pose.orientation.roll), 1.0f, 0.0f, 0.0f);		


		// Xsens
		glPushMatrix();

			glTranslatef(carDrawer->xsens_pose.position.x,carDrawer->xsens_pose.position.y,carDrawer->xsens_pose.position.z);
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);		
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
			glRotatef(carmen_radians_to_degrees(carDrawer->xsens_pose.orientation.roll), 1.0f, 0.0f, 0.0f);		

			glColor3f(1.0,0.6,0.0);
			drawBox(carDrawer->xsens_size.x, carDrawer->xsens_size.y, carDrawer->xsens_size.z);		

		glPopMatrix();

//		// Laser
//		glPushMatrix();
//
//			glTranslatef(carDrawer->laser_pose.position.x, carDrawer->laser_pose.position.y, carDrawer->laser_pose.position.z);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.roll), 1.0f, 0.0f, 0.0f);
//
//			glColor3f(0.0,0.0,1.0);
//			drawBox(carDrawer->laser_size.x, carDrawer->laser_size.y, carDrawer->laser_size.z);
//
//		glPopMatrix();

//		//Laser
//		glPushMatrix();
//
//			glTranslatef(0, 0, 0.12);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.yaw),  0.0f, 0.0f, 1.0f);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
//			glRotatef(carmen_radians_to_degrees(carDrawer->laser_pose.orientation.roll), 1.0f, 0.0f, 0.0f);
//
//			glColor3f(1.0,0.0,1.0);
//			draw_cylinder(0.0445, 0.14);
//			//drawCylinder(carDrawer->laser_size.x, carDrawer->laser_size.y, carDrawer->laser_size.z);
//
//		glPopMatrix();
//
//		// SensorBox
//		glPushMatrix();
//
//			glColor3f(0.0,1.0,1.0);
//			drawBox(carDrawer->sensor_box_size.x, carDrawer->sensor_box_size.y, carDrawer->sensor_box_size.z);
//
//		glPopMatrix();
	
	glPopMatrix();

	/*
	glPushMatrix();
		
		glPushMatrix();
			
			glColor3f(0.3,0.3,0.3);
			draw_wheel_axis(carDrawer->car_wheel_diameter,carDrawer->car_size.y);

			glPushMatrix();
				glTranslatef(carDrawer->car_axis_distance, 0.0, 0.0);
				draw_wheel_axis(carDrawer->car_wheel_diameter,carDrawer->car_size.y);
			glPopMatrix();

			glColor3f(1.0,0.0,0.0);
			glPushMatrix();
				glTranslatef(carDrawer->car_pose.position.x,carDrawer->car_pose.position.y,carDrawer->car_pose.position.z+carDrawer->car_wheel_diameter/2);
				glRotatef(carDrawer->car_pose.orientation.roll, 1.0f, 0.0f, 0.0f);
				glRotatef(carDrawer->car_pose.orientation.pitch, 0.0f, 1.0f, 0.0f);
				glRotatef(carDrawer->car_pose.orientation.yaw,  0.0f, 0.0f, 1.0f);
				drawBox(carDrawer->car_size.x, carDrawer->car_size.y, carDrawer->car_size.z-carDrawer->car_wheel_diameter);
			glPopMatrix();

		glPopMatrix();

	glPopMatrix();

	*/
	
	
}

void draw_car_at_pose(CarDrawer* carDrawer, carmen_pose_3D_t pose)
{
	glPushMatrix();
		glTranslatef(pose.position.x, pose.position.y, pose.position.z);
		glRotatef(carmen_radians_to_degrees(pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.roll), 1.0f, 0.0f, 0.0f);

		draw_car(carDrawer);
	glPopMatrix();
}


void destroyCarDrawer(CarDrawer* carDrawer)
{
	glmDelete(carDrawer->carModel);
	free(carDrawer);
}
