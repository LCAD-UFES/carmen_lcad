#include "draw_car.h"
#include <locale.h>


static void drawBox(double length_x, double length_y, double length_z);


CarDrawer *
createCarDrawer(int argc, char** argv)
{	
	CarDrawer* carDrawer = (CarDrawer*)malloc(sizeof(CarDrawer));

	int num_items;

	char *carmodel_file = NULL;
	char *semi_trailer_model_file = NULL;

	char *robot_collision_file = NULL;
	char *semi_trailer_collision_file = NULL;

	carmen_param_t param_list[] = {
	{"carmodel", "file_name", CARMEN_PARAM_STRING, &carmodel_file, 0, NULL},
	{"carmodel", "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.x), 0, NULL},
	{"carmodel", "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.y), 0, NULL},
	{"carmodel", "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_size.z), 0, NULL},
	{"carmodel", "x", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.x), 0, NULL},
	{"carmodel", "y", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.y), 0, NULL},
	{"carmodel", "z", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.position.z), 0, NULL},
	{"carmodel", "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.roll), 0, NULL},
	{"carmodel", "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.pitch), 0, NULL},
	{"carmodel", "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->car_pose.orientation.yaw), 0, NULL},
	{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(carDrawer->car_axis_distance), 0, NULL},
	{"robot", "wheel_radius", CARMEN_PARAM_DOUBLE, &(carDrawer->car_wheel_radius), 0, NULL},
	{"robot", "length", CARMEN_PARAM_DOUBLE, &(carDrawer->robot_size.x), 0, NULL},
	{"robot", "width", CARMEN_PARAM_DOUBLE, &(carDrawer->robot_size.y), 0, NULL},
	{"robot", "collision_file", CARMEN_PARAM_STRING, &robot_collision_file, 1, NULL},
	{"semi_trailer", "initial_type", CARMEN_PARAM_INT, &(carDrawer->semi_trailer_config.type), 0, NULL},
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

	printf("FILE: %s\n", carmodel_file);

	if (carmodel_file == NULL)
		carDrawer->carModel = glmReadOBJ("ford_escape_model.obj");
	else
		carDrawer->carModel = glmReadOBJ(carmodel_file);
	glmUnitize(carDrawer->carModel);

	glmScale(carDrawer->carModel, carDrawer->car_size.x/2.0);

	char *carmen_home = getenv("CARMEN_HOME");

	if (carmen_home == NULL)
		exit(printf("Could not get environment variable $CARMEN_HOME in createCarDrawer()\n"));

	char collision_file_[2048];
	strcpy(collision_file_, carmen_home);
	strcat(collision_file_, "/bin/");
	strcat(collision_file_, robot_collision_file);

	FILE *collision_file_pointer = fopen(collision_file_, "r");
	setlocale(LC_NUMERIC, "C");
	fscanf(collision_file_pointer, "%d", &(carDrawer->robot_collision_config.n_markers));
	int max_h_level;
	fscanf(collision_file_pointer, "%d", &max_h_level);
	carDrawer->robot_collision_config.markers = (carmen_collision_marker_t *) malloc(carDrawer->robot_collision_config.n_markers * sizeof(carmen_collision_marker_t));

	for (int i = 0; i < carDrawer->robot_collision_config.n_markers; i++)
		fscanf(collision_file_pointer,"%lf %lf %lf %d", &(carDrawer->robot_collision_config.markers[i].x) , &(carDrawer->robot_collision_config.markers[i].y),
				&(carDrawer->robot_collision_config.markers[i].radius), &(carDrawer->robot_collision_config.markers[i].height_level));

	fclose(collision_file_pointer);

	if (carDrawer->semi_trailer_config.type > 0)
	{
		char semi_trailer_string[256];
		char semi_trailer_model_string[256];

		sprintf(semi_trailer_string, "%s%d", "semi_trailer", carDrawer->semi_trailer_config.type);
		sprintf(semi_trailer_model_string, "%s%d", "semi_trailer_model", carDrawer->semi_trailer_config.type);

		carmen_param_t param_list2[] = {
		{semi_trailer_string, "d", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.d), 0, NULL},
		{semi_trailer_string, "M", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.M), 0, NULL},
		{semi_trailer_string, "width", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.width), 0, NULL},
		{semi_trailer_string, "distance_between_axle_and_front", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.distance_between_axle_and_front), 0, NULL},
		{semi_trailer_string, "distance_between_axle_and_back", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_config.distance_between_axle_and_back), 0, NULL},
		{semi_trailer_string, "collision_file", CARMEN_PARAM_STRING, &semi_trailer_collision_file, 1, NULL},
		{semi_trailer_model_string, "file_name", CARMEN_PARAM_STRING, &semi_trailer_model_file, 0, NULL},
		{semi_trailer_model_string, "size_x", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size.x), 0, NULL},
		{semi_trailer_model_string, "size_y", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size.y), 0, NULL},
		{semi_trailer_model_string, "size_z", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_size.z), 0, NULL},
		{semi_trailer_model_string, "x", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.position.x), 0, NULL},
		{semi_trailer_model_string, "y", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.position.y), 0, NULL},
		{semi_trailer_model_string, "z", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.position.z), 0, NULL},
		{semi_trailer_model_string, "roll", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.orientation.roll), 0, NULL},
		{semi_trailer_model_string, "pitch", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.orientation.pitch), 0, NULL},
		{semi_trailer_model_string, "yaw", CARMEN_PARAM_DOUBLE, &(carDrawer->semi_trailer_pose.orientation.yaw), 0, NULL}
		};

		num_items = sizeof(param_list2)/sizeof(param_list2[0]);
		carmen_param_install_params(argc, argv, param_list2, num_items);

		carDrawer->semi_trailer_config.max_beta = carmen_degrees_to_radians(carDrawer->semi_trailer_config.max_beta);

		if (semi_trailer_model_file == NULL)
			carDrawer->semiTrailerModel = glmReadOBJ("ford_escape_model.obj");
		else
			carDrawer->semiTrailerModel = glmReadOBJ(semi_trailer_model_file);
		glmUnitize(carDrawer->semiTrailerModel);

		glmScale(carDrawer->semiTrailerModel, carDrawer->semi_trailer_size.x/2.0);

		strcpy(collision_file_, carmen_home);
		strcat(collision_file_, "/bin/");
		strcat(collision_file_, semi_trailer_collision_file);

		collision_file_pointer = fopen(collision_file_, "r");
		setlocale(LC_NUMERIC, "C");
		fscanf(collision_file_pointer, "%d", &(carDrawer->semi_trailer_collision_config.n_markers));
		fscanf(collision_file_pointer, "%d", &max_h_level);
		carDrawer->semi_trailer_collision_config.markers = (carmen_collision_marker_t *) malloc(carDrawer->semi_trailer_collision_config.n_markers * sizeof(carmen_collision_marker_t));

		for (int i = 0; i < carDrawer->semi_trailer_collision_config.n_markers; i++)
			fscanf(collision_file_pointer,"%lf %lf %lf %d", &(carDrawer->semi_trailer_collision_config.markers[i].x) , &(carDrawer->semi_trailer_collision_config.markers[i].y),
					&(carDrawer->semi_trailer_collision_config.markers[i].radius), &(carDrawer->semi_trailer_collision_config.markers[i].height_level));

		fclose(collision_file_pointer);
	}

	return carDrawer;
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
draw_cylinder(GLfloat radius, GLfloat height)
{
    GLfloat x              = 0.0;
    GLfloat y              = 0.0;
    GLfloat angle          = 0.0;
    GLfloat angle_stepsize = 0.1;

    /** Draw the tube */
    glPushMatrix();

//		glBegin(GL_QUAD_STRIP);
//		angle = 0.0;
//			while( angle < 2*M_PI ) {
//				x = radius * cos(angle);
//				y = radius * sin(angle);
//				glVertex3f(x, y , height);
//				glVertex3f(x, y , 0.0);
//				angle = angle + angle_stepsize;
//			}
//			glVertex3f(radius, 0.0, height);
//			glVertex3f(radius, 0.0, 0.0);
//		glEnd();

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


void
draw_collision_range(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta, int semi_trailer_engaged)
{
	glPushMatrix();
		glColor3f (1.0, 0.0, 0.0);
		glTranslatef(pose.position.x, pose.position.y, pose.position.z);
		glRotatef(carmen_radians_to_degrees(pose.orientation.yaw), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.pitch), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(pose.orientation.roll), 1.0f, 0.0f, 0.0f);
		for (int i = 0; i < carDrawer->robot_collision_config.n_markers; i++)
		{
			glPushMatrix();
				glTranslatef(carDrawer->robot_collision_config.markers[i].x, carDrawer->robot_collision_config.markers[i].y, 0.0);
				draw_cylinder(carDrawer->robot_collision_config.markers[i].radius, 0.0);
			glPopMatrix();
		}

		if (semi_trailer_engaged)
		{
			glTranslatef(-carDrawer->semi_trailer_config.M, 0.0, 0.0);
			glRotatef(-carmen_radians_to_degrees(beta), 0.0f, 0.0f, 1.0f);
			for (int i = 0; i < carDrawer->semi_trailer_collision_config.n_markers; i++)
			{
				glPushMatrix();
					glTranslatef(carDrawer->semi_trailer_collision_config.markers[i].x - carDrawer->semi_trailer_config.d, carDrawer->semi_trailer_collision_config.markers[i].y, 0.0);
					draw_cylinder(carDrawer->semi_trailer_collision_config.markers[i].radius, 0.0);
				glPopMatrix();
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
draw_car_outline(CarDrawer *carDrawer, double beta, int semi_trailer_engaged)
{
	// Car
	glPushMatrix();

		glTranslatef(carDrawer->car_pose.position.x,carDrawer->car_pose.position.y,0.0);

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
		glPushMatrix();
			glRotatef(-carmen_radians_to_degrees(beta), 0.0, 0.0, 1.0);

			glTranslatef(-carDrawer->semi_trailer_config.d - carDrawer->semi_trailer_config.M * cos(beta),
						 -carDrawer->semi_trailer_config.M * sin(beta),
						 0.0);

			glBegin(GL_LINE_STRIP);
				glVertex3f(-carDrawer->semi_trailer_config.distance_between_axle_and_back, -carDrawer->semi_trailer_config.width / 2, 0);
				glVertex3f(carDrawer->semi_trailer_config.distance_between_axle_and_front, -carDrawer->semi_trailer_config.width / 2, 0);
				glVertex3f(carDrawer->semi_trailer_config.distance_between_axle_and_front, carDrawer->semi_trailer_config.width / 2, 0);
				glVertex3f(-carDrawer->semi_trailer_config.distance_between_axle_and_back, carDrawer->semi_trailer_config.width / 2, 0);
				glVertex3f(-carDrawer->semi_trailer_config.distance_between_axle_and_back, -carDrawer->semi_trailer_config.width / 2, 0);
			glEnd();

		glPopMatrix();
	}
}


void
draw_car(CarDrawer *carDrawer, double beta, int semi_trailer_engaged)
{
	//draw_axis(500.0);

	// Car
	glPushMatrix();

		glTranslatef(carDrawer->car_pose.position.x,carDrawer->car_pose.position.y,carDrawer->car_pose.position.z+carDrawer->car_wheel_radius);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glRotatef(0.0, 0.0, 1.0, 0.0);

		glColor3f(0.3,0.3,0.3);
		//glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR);
		glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);
		
		
	glPopMatrix();
	
	if (semi_trailer_engaged)
	{
		// Semi-trailer
		glPushMatrix();

			glRotatef(90.0, 1.0, 0.0, 0.0);
			glRotatef(0.0, 0.0, 1.0, 0.0);
			glRotatef(-carmen_radians_to_degrees(beta), 0.0, 1.0, 0.0);

			glTranslatef(carDrawer->semi_trailer_pose.position.x - carDrawer->semi_trailer_config.d - carDrawer->semi_trailer_config.M * cos(beta),
						 carDrawer->semi_trailer_pose.position.z,
						 carDrawer->semi_trailer_pose.position.y + carDrawer->semi_trailer_config.M * sin(beta));

			glColor3f(0.3,0.3,0.3);
			//glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR);
			glmDraw(carDrawer->semiTrailerModel, GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);

		glPopMatrix();
	}

	
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
			draw_wheel_axis(carDrawer->car_wheel_radius * 2.0,carDrawer->car_size.y);

			glPushMatrix();
				glTranslatef(carDrawer->car_axis_distance, 0.0, 0.0);
				draw_wheel_axis(carDrawer->car_wheel_radius * 2.0,carDrawer->car_size.y);
			glPopMatrix();

			glColor3f(1.0,0.0,0.0);
			glPushMatrix();
				glTranslatef(carDrawer->car_pose.position.x,carDrawer->car_pose.position.y,carDrawer->car_pose.position.z+carDrawer->car_wheel_radius);
				glRotatef(carDrawer->car_pose.orientation.roll, 1.0f, 0.0f, 0.0f);
				glRotatef(carDrawer->car_pose.orientation.pitch, 0.0f, 1.0f, 0.0f);
				glRotatef(carDrawer->car_pose.orientation.yaw,  0.0f, 0.0f, 1.0f);
				drawBox(carDrawer->car_size.x, carDrawer->car_size.y, carDrawer->car_size.z-carDrawer->car_wheel_radius * 2.0);
			glPopMatrix();

		glPopMatrix();

	glPopMatrix();

	*/
}


void
draw_car_at_pose(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta, int semi_trailer_engaged)
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
draw_car_outline_at_pose(CarDrawer *carDrawer, carmen_pose_3D_t pose, double beta, int semi_trailer_engaged)
{
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
	free(carDrawer->robot_collision_config.markers);
	free(carDrawer->semi_trailer_collision_config.markers);
	free(carDrawer);
}
