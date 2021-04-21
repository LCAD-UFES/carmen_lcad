#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/rotation_geometry.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "Window.h"
#include "draw_car.h"

#define WINDOW_WIDTH	1000.0
#define WINDOW_HEIGHT	600.0

window* gl_window;
CarDrawer* car_draw;

static carmen_xsens_global_matrix_message xsens_matrix_message_global;

carmen_orientation_3D_t car_orientation;
carmen_pose_3D_t camera_pose;

void mouseFunc(int type, int button, int x, int y);
void keyPress(int code);
void keyRelease(int code);
void resizeFunc(int width, int height);

static void xsens_matrix_message_handler(void) 
{ 	
	rotation_matrix* xsens_matrix = create_rotation_matrix_from_matrix_inverse(xsens_matrix_message_global.matrix_data.m_data);
	car_orientation = get_angles_from_rotation_matrix(xsens_matrix);
	destroy_rotation_matrix(xsens_matrix);

	car_orientation.roll = carmen_radians_to_degrees(car_orientation.roll);
	car_orientation.pitch = carmen_radians_to_degrees(car_orientation.pitch);
	car_orientation.yaw = carmen_radians_to_degrees(car_orientation.yaw);	

	printf("r:% lf p:% lf y:% lf\n", car_orientation.roll, car_orientation.pitch, car_orientation.yaw);
}

void moveCamera(float x, float y, float z)
{
	camera_pose.position.x += x;
	camera_pose.position.y += y;
	camera_pose.position.z += z;
}

void rotateCamera(float roll, float pitch, float yaw)
{
	camera_pose.orientation.roll += roll;
	camera_pose.orientation.pitch += pitch; 
	camera_pose.orientation.yaw += yaw;
}

void initGl()
{
	int zero = 0;
	glutInit(&zero, NULL);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f); 
	glClearDepth(1.0);        
	glDepthFunc(GL_LESS);     
	glEnable(GL_DEPTH_TEST);  
	glShadeModel(GL_SMOOTH);     

	GLfloat light_ambient[]= { 0.5f, 0.5f, 0.5f, 1.0f }; 		
	GLfloat light_diffuse[]= { 0.4f, 0.4f, 0.3f, 1.0f };			
	GLfloat light_position[]= { 0.0f, 1.0f, 2.0f, 1.0f };	
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);			
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);		
	glLightfv(GL_LIGHT1, GL_POSITION, light_position);	
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHTING);	   

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();     

	gluPerspective(45.0f, WINDOW_WIDTH / WINDOW_HEIGHT, 0.1f, 10000.0f);  

	glMatrixMode(GL_MODELVIEW);

	camera_pose.position.x = 0.0;
	camera_pose.position.y = 0.0;
	camera_pose.position.z = 10.0;
	camera_pose.orientation.roll = 0.0;
	camera_pose.orientation.pitch = 0.0; 
	camera_pose.orientation.yaw = 0.0;
  
}

void initialize(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	
	gl_window = initWindow(WINDOW_WIDTH, WINDOW_HEIGHT);
	initGl();
	car_draw = createCarDrawer(argc,argv);

	carmen_xsens_subscribe_xsens_global_matrix_message(	&xsens_matrix_message_global,
		                                            	(carmen_handler_t)xsens_matrix_message_handler,
		                                            	CARMEN_SUBSCRIBE_LATEST);

}

void finalize()
{
	destroyCarDrawer(car_draw);
}

int main(int argc, char** argv)
{
	initialize(argc, argv);
            

	while(showWindow(gl_window))
	{
		if(!processWindow(gl_window, mouseFunc, keyPress, keyRelease, resizeFunc))
		{
			break;
		}
			
		carmen_ipc_sleep(0.01);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		
		glTranslatef(-camera_pose.position.x, -camera_pose.position.y, -camera_pose.position.z);	
		glRotatef(camera_pose.orientation.roll, -1.0f, 0.0f, 0.0f);
		glRotatef(camera_pose.orientation.pitch, 0.0f, -1.0f, 0.0f);
		glRotatef(camera_pose.orientation.yaw, 0.0f, 0.0f, -1.0f);
			
		
		glRotatef(car_orientation.yaw, 0.0f, 0.0f, 1.0f);		
		glRotatef(car_orientation.pitch, 0.0f, 1.0f, 0.0f);
		glRotatef(car_orientation.roll, 1.0f, 0.0f, 0.0f);

		draw_car(car_draw, 0.0, 0);
	}                                                                                 
	
	carmen_ipc_disconnect();  

	return 0;
}




void mouseFunc(int type, int button, int x, int y)
{
	static int lastX = 0;
	static int lastY = 0;
	static int pressed = 0;

	if(type == 4)
	{
		pressed = 1;
	}
	else if(type == 5)
	{
		pressed = 0;
	}

	if(button == 4)
	{
		moveCamera(0,0,1.0);
	}
	else if(button == 5)
	{
		moveCamera(0,0,-1.0);
	}

	int dx = x - lastX;
	int dy = y - lastY;

	lastX = x;
	lastY = y;

	if(pressed)
	{
		rotateCamera(-dy, 0, dx);
	}
}


void keyPress(int code)
{
	switch(code)
	{
		case 111: //UP
		{
			moveCamera(0.0,0.0,-1.0);
		}break;

		case 113: //LEFT
		{
			rotateCamera(0.0, -4.0, 0.0);
		}break;

		case 114: //RIGHT
		{
			rotateCamera(0.0, 4.0, 0.0);
		}break;

		case 116: //DOWN
		{
			moveCamera(0.0,0.0,1.0);
		}break;

		case 65:  // Space
		{

		}break;
	}
}

void keyRelease(int code)
{
	code = code; // just to make gcc happy
}

void
resizeFunc(int width, int height)			// compatibilidade com callback do viewer_3D
{
	width = width; // just to make gcc happy
	height = height;
}

