#include <carmen/carmen.h>
#include <carmen/parking_assistant_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>

const int redraw_update_period = 30;

int parking_assistant_window_id;

static carmen_robot_ackerman_config_t car_config;
carmen_parking_assistant_goal_message goal;
carmen_parking_assistant_parking_space_message parking_space;
carmen_localize_ackerman_globalpos_message pose;
carmen_navigator_ackerman_plan_message plan;
carmen_ultrasonic_sonar_sensor_message ultrasonic;
static int planned = 0;
static int parking_space_mapped = 0;
static int step = 2;
static int driver_following_right_path = 0;
static int goal_reached = 0;

int num_messages_per_second = 0;
double time_last_report = 0;

void
shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("parking_assistant_viewer: disconnected.\n");
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
carmen_parking_assistant_message_handler()
{
	num_messages_per_second++;

	if (time(NULL) - time_last_report > 1)
	{
		time_last_report = time(NULL);
		printf("parking_assistant_VIEWER Num messages per second: %d\n", num_messages_per_second);
		num_messages_per_second = 0;
	}
}

void
carmen_goal_message_handler(carmen_parking_assistant_goal_message* msg)
{
	printf("goal message\n");
	goal.pose = msg->pose;
	goal.host = carmen_get_host();
	goal.timestamp = msg->timestamp;

	parking_space_mapped = 1;
}

void
carmen_plan_message_handler(carmen_navigator_ackerman_plan_message* msg)
{
	if (!planned && msg->path_length) {

		plan.host = carmen_get_host();
		plan.path = (carmen_robot_and_trailer_traj_point_t *)malloc(sizeof(carmen_robot_and_trailer_traj_point_t)*(msg->path_length));
		for (int i=0 ; i<msg->path_length ; i++) {
			(plan.path+i)->phi = (msg->path+i)->phi;
			(plan.path+i)->theta = (msg->path+i)->theta;
			(plan.path+i)->beta = (msg->path+i)->beta;
			(plan.path+i)->v = (msg->path+i)->v;
			(plan.path+i)->x = (msg->path+i)->x;
			(plan.path+i)->y = (msg->path+i)->y;
		}
		plan.path_length = msg->path_length;
		plan.timestamp = msg->timestamp;

		planned = 1;

		printf("path_length: %d\n",msg->path_length);
	}
}


void
subcribe_messages()
{

	carmen_parking_assistant_subscribe_goal(NULL,
		(carmen_handler_t) carmen_goal_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_parking_assistant_subscribe_parking_space(&parking_space,
		(carmen_handler_t) carmen_parking_assistant_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_ultrasonic_sonar_sensor_subscribe(&ultrasonic,
		(carmen_handler_t) carmen_parking_assistant_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(&pose,
			(carmen_handler_t) carmen_parking_assistant_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_plan_message(NULL,
			(carmen_handler_t) carmen_plan_message_handler,
			CARMEN_SUBSCRIBE_LATEST);



}


void
Timer(int value __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	glutPostWindowRedisplay(parking_assistant_window_id);
	glutTimerFunc(redraw_update_period, Timer, 1);
}

double* rotacao(carmen_world_point_t p,double theta)
{
	double* resultado = (double *)malloc(sizeof(double));
	resultado[0] = p.pose.x*cos(theta) - p.pose.y*sin(theta);
	resultado[1] = p.pose.x*sin(theta) + p.pose.y*cos(theta);
	return resultado;
}

void
draw_car(double x,double y,double theta,int use_ultrasonic) {


	double* resultado[4] = {NULL, NULL, NULL, NULL};

	//cars vertices
	carmen_world_point_t p[4];

	/*

	p3---p2
	|	  |
	|	  |
	|	  |
	p0---p1

	*/

	//enters the car at the origin
	p[0].pose.x = -car_config.width/2.0; p[0].pose.y = -(car_config.distance_between_rear_car_and_rear_wheels);
	p[1].pose.x = car_config.width/2.0; p[1].pose.y = -(car_config.distance_between_rear_car_and_rear_wheels);
	p[2].pose.x = car_config.width/2.0; p[2].pose.y = (car_config.length - car_config.distance_between_rear_car_and_rear_wheels);
	p[3].pose.x = -car_config.width/2.0; p[3].pose.y = (car_config.length - car_config.distance_between_rear_car_and_rear_wheels);

	//rotates the car using theta
	resultado[0] = rotacao(p[0],theta);
	resultado[1] = rotacao(p[1],theta);
	resultado[2] = rotacao(p[2],theta);
	resultado[3] = rotacao(p[3],theta);

	//moves the car to the given position (x,y)
	resultado[0][0] = resultado[0][0] + x;
	resultado[0][1] = resultado[0][1] + y;
	resultado[1][0] = resultado[1][0] + x;
	resultado[1][1] = resultado[1][1] + y;
	resultado[2][0] = resultado[2][0] + x;
	resultado[2][1] = resultado[2][1] + y;
	resultado[3][0] = resultado[3][0] + x;
	resultado[3][1] = resultado[3][1] + y;

	glBegin(GL_LINE_LOOP);
	glVertex2f(resultado[0][0], resultado[0][1]);
	glVertex2f(resultado[1][0], resultado[1][1]);
	glVertex2f(resultado[2][0], resultado[2][1]);
	glVertex2f(resultado[3][0], resultado[3][1]);
	glEnd();

	if (use_ultrasonic) {

		float front_sensor;
		float back_sensor;

		front_sensor = ultrasonic.sensor[3]/3.10;
		back_sensor = ultrasonic.sensor[0]/3.10;

		glLineWidth(4.0);
		glBegin(GL_LINES);

		if (back_sensor > 0.5) { //GREEN if distance is greater than 1.55 meters
			glColor3f(0.0, 1.0, 0.0);
		}else if (back_sensor >= 0.2258) { //GREEN TO YELLOW if distance between 0.70 and 1.55 meters
			glColor3f((0.5-back_sensor)*3.6470, 1.0, 0.0);
		}else { //YELLOW TO RED if distance smaller than 0.70 meters
			glColor3f(1.0, back_sensor*4.4286, 0.0);
		}
		glVertex2f(resultado[0][0], resultado[0][1]);
		glVertex2f(resultado[1][0], resultado[1][1]);

		if (front_sensor > 0.5) { //GREEN if distance is greater than 1.55 meters
			glColor3f(0.0, 1.0, 0.0);
		}else if (front_sensor >= 0.2258) { //GREEN TO YELLOW if distance between 0.70 and 1.55 meters
			glColor3f((0.5-front_sensor)*3.6470, 1.0, 0.0);
		}else { //YELLOW TO RED if distance smaller than 0.70 meters
			glColor3f(1.0, front_sensor*4.42869, 0.0);
		}
		glVertex2f(resultado[2][0], resultado[2][1]);
		glVertex2f(resultado[3][0], resultado[3][1]);
		glEnd();
	}

}

void
draw_drivers_car_origin() {
	glEnable(GL_POINT_SMOOTH);
	if (driver_following_right_path) {
		//GREEN IF THE DRIVER IS FOLLOWING THE PATH RIGHT
		glColor3f(0.0, 0.5, 0.0);
	}else {
		//RED IF IT IS NOT
		glColor3f(1.0, 0.0, 0.0);
	}
	glPointSize(10.0);
	glBegin(GL_POINTS);
	glVertex2f((-1)*pose.globalpos.y,pose.globalpos.x);
	glEnd();
}

double
distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((y2-y1),2) + pow((x2-x1),2));
}

void print_point(carmen_point_t p)
{
	glEnable(GL_POINT_SMOOTH);
	glColor3f(0.0, 0.5, 0.0);
	glPointSize(10.0);
	glBegin(GL_POINTS);
	glVertex2f((-1)*p.y,p.x);
	glEnd();
}

void print_circle(carmen_point_t p) {
	double x,y;
	x = (-1)*p.y;
	y = p.x;
	double i;
	//Draw Circle
	glBegin(GL_POLYGON);
	//Change the 6 to 12 to increase the steps (number of drawn points) for a smoother circle
	//Note that anything above 24 will have little affect on the circles appearance
	//Play with the numbers till you find the result you are looking for
	//Value 1.5 - Draws Triangle
	//Value 2 - Draws Square
	//Value 3 - Draws Hexagon
	//Value 4 - Draws Octagon
	//Value 5 - Draws Decagon
	//Notice the correlation between the value and the number of sides
	//The number of sides is always twice the value given this range
	double pi = 3.14159265358979323846264338327950288419716939937510;
	for(i = 0; i < 2 * pi; i += pi / 12.0) //<-- Change this Value
		glVertex3f(cos(i) * parking_space.r + x, sin(i) * parking_space.r + y, 0.0);
	glEnd();
	//Draw Circle
}


void
draw_parking_assistant_message()
{
	if (!parking_space_mapped) {
		glRasterPos2f(-8.0,10.0);
		glColor4f(0.0f,0.0f,1.0f,1.0f);
		glutBitmapString(GLUT_BITMAP_HELVETICA_18,(const unsigned char*)"Mapping parking space, please wait.");
		return;
	}else {

		//DRAW ORIGIN
		glEnable(GL_POINT_SMOOTH);
		glColor3f(1.0, 0.0, 0.0);
		glPointSize(10.0);
		glBegin(GL_POINTS);
		glVertex2f(0,0);
		glEnd();

		glColor3f(1.0, 1.0, 0.5);
		print_circle(parking_space.c1);
		print_circle(parking_space.c2);

		glColor3f(0.0, 0.0, 0.0);
		print_point(parking_space.p1);
		print_point(parking_space.p2);
		print_point(parking_space.p3);

		print_point(parking_space.c1);
		print_point(parking_space.c2);

		// DRAW CAR GLOBALPOS
		glColor3f(0.0, 0.0, 0.0);
		glLineWidth(2.0);
		draw_car((-1)*pose.globalpos.y,pose.globalpos.x,pose.globalpos.theta,0);

		//DRAW GOAL POSE
		glColor3f(0.0, 0.0, 0.0);
		draw_car((-1)*goal.pose.y,goal.pose.x,goal.pose.theta,0);

		return;
	}

	if (!planned) {
		glRasterPos2f(-8.0,10.0);
		glColor4f(0.0f,0.0f,1.0f,1.0f);
		glutBitmapString(GLUT_BITMAP_HELVETICA_18,(const unsigned char*)"Tracing the route, please wait.");
		return;
	}

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(2.0);

	double free_space = parking_space.size - car_config.length;
	int i = 0;
	int j = 0;

	// DRAW CAR GLOBALPOS
	glColor3f(0.0, 0.0, 0.0);
	draw_car((-1)*pose.globalpos.y,pose.globalpos.x,pose.globalpos.theta,1);

	//DRAW THE LIMITS OF PARKING SPACE
	glLineWidth(1.0);
	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 0.0);
	double x1,x2;
	x1 = goal.pose.x + (car_config.length - car_config.distance_between_rear_car_and_rear_wheels) + free_space/2.0;
	glVertex2f((-1)*goal.pose.y-(car_config.width/2.0), x1);
	glVertex2f((-1)*goal.pose.y+(car_config.width/2.0), x1);
	x2 = goal.pose.x - car_config.distance_between_rear_car_and_rear_wheels - free_space/2.0;
	glVertex2f((-1)*goal.pose.y-(car_config.width/2.0), x2);
	glVertex2f((-1)*goal.pose.y+(car_config.width/2.0), x2);

	glVertex2f((-1)*goal.pose.y+(car_config.width/2.0), x1);
	glVertex2f((-1)*goal.pose.y+(car_config.width/2.0), x2);
	glEnd();

	if (distance((-1)*pose.globalpos.y,pose.globalpos.x,(-1)*goal.pose.y,goal.pose.x) < 0.3 && fabs(goal.pose.theta-pose.globalpos.theta) < 0.2)
	{
		goal_reached = 1;
	}


	if (planned && !goal_reached) {

		if (step > plan.path_length-1) {
			step = plan.path_length-1;
		}

		//DRAW THE PATH THAT MUST BE FOLLOWED BY THE DRIVER AND CHECK THE DISTANCE BETWEEN PATH AND GLOBALPOS
		double delta;
		double num_parts = 8.0;
		double Xi, Yi, m;
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(1.0);
		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 0.0);
		driver_following_right_path = 0;
		for (i=0 ; i<(plan.path_length-1) ; i++) {
			delta = ((plan.path+i)->x - (plan.path+i+1)->x)/num_parts;
			for (j = 0 ; j <= num_parts ; j++) {
				Xi = (plan.path+i)->x - j*delta;
				m = ((plan.path+i+1)->y - (plan.path+i)->y) / ((plan.path+i+1)->x - (plan.path+i)->x);
				Yi = (-1)*(plan.path+i)->y + ((plan.path+i)->x)*m - Xi*m;

				if (distance((-1)*pose.globalpos.y,pose.globalpos.x,Yi,Xi) <= 0.4) {
					driver_following_right_path = 1;
				}
			}
			glVertex2f((-1)*(plan.path+i)->y, (plan.path+i)->x);
			glVertex2f((-1)*(plan.path+i+1)->y, (plan.path+i+1)->x);
		}
		glEnd();

		//DRAW THE NEXT STEP
		glColor3f(0.0, 0.0, 0.5);
		draw_car((-1)*(plan.path+step)->y,(plan.path+step)->x,(plan.path+step)->theta,0);

		//IF PASSED THE CURRENT STEP FOLLOWING THE PATH RIGHT GOES TO THE NEXT STEP
		if (pose.globalpos.x < (plan.path+step)->x) {
			step = step+2;
			if (!driver_following_right_path || fabs((plan.path+step)->theta-pose.globalpos.theta) > 0.4) { //0.4 = 22.91 degrees
				step = 2;
				planned = 0; //SET planned = 0 TO TRACE ANOTHER ROUTE
			}
		}
	}

	draw_drivers_car_origin();
}


void
draw_parking_assistant()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT);
	draw_parking_assistant_message();

	glutSwapBuffers();
}


void
handle_parking_assistant_viewer_resize(int w __attribute__ ((unused)), int h __attribute__ ((unused)))
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(
		((double) -15),
		((double) 15),
		((double) -5),
		((double) 25)
	);
}


void
initialize_viewer(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	char window_name[128];
	sprintf(window_name, "parking_assistant Viewer");

	// parking_assistant VIEWER WINDOW
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(200, 50);
	parking_assistant_window_id = glutCreateWindow(window_name);
	glutReshapeFunc(handle_parking_assistant_viewer_resize);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glutDisplayFunc(draw_parking_assistant);

	glutTimerFunc(redraw_update_period, Timer, 1);
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char*)"robot",  (char*)"distance_between_rearview",CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rearview), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_car_and_rear_wheels",CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_car_and_rear_wheels), 1, NULL},
			{(char*)"robot",  (char*)"width",CARMEN_PARAM_DOUBLE, &(car_config.width), 1, NULL},
			{(char*)"robot",  (char*)"length", CARMEN_PARAM_DOUBLE, &(car_config.length), 1, NULL}

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int
main(int argc, char **argv)
{

	goal.pose.x = 0.0;
	goal.pose.y = 0.0;

	initialize_ipc(argc, argv);
	initialize_viewer(argc, argv);

	read_parameters(argc, argv);

	subcribe_messages();
	glutMainLoop();

	return 0;
}

