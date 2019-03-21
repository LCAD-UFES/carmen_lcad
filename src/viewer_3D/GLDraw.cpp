#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/moving_objects_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "viewer_3D.h"
#include "GLDraw.h"
#include "texture_loader.h"


//float cameraX, cameraY, cameraZ;
//float cameraXOffset, cameraYOffset, cameraZOffset;
//float cameraRoll, cameraPitch, cameraYaw;

static carmen_pose_3D_t camera_pose;
static carmen_pose_3D_t camera_offset;

static unsigned int map_image_texture_id;
static unsigned int localize_image_base_texture_id;
static unsigned int localize_image_curr_texture_id;

static unsigned int laser_buffer_id;
static double *laser_pos_buffer;

double background_r;
double background_g;
double background_b;

void
initGl ()
{
    int zero = 0;
    glutInit (&zero, NULL);
    glewInit ();

    glClearColor (0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth (1.0);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_FLAT);

    GLfloat light_ambient[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat light_diffuse[] = {0.4f, 0.4f, 0.3f, 1.0f};
    GLfloat light_position[] = {0.0f, 1.0f, 2.0f, 1.0f};

    glLightfv (GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv (GL_LIGHT1, GL_POSITION, light_position);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);

    glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable (GL_COLOR_MATERIAL);

    map_image_texture_id = create_texture ();

    localize_image_base_texture_id = create_texture2 ();
    localize_image_curr_texture_id = create_texture2 ();

    glGenBuffers (1, &laser_buffer_id);
    glBindBuffer (GL_ARRAY_BUFFER, laser_buffer_id);

    laser_pos_buffer = NULL;

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();

    gluPerspective (45.0f, WINDOW_WIDTH / WINDOW_HEIGHT, 0.1f, 4000.0f); // Calculate The Aspect Ratio Of The Window

    carmen_pose_3D_t zero_pose;
    zero_pose.position.x = 0.0;
    zero_pose.position.y = 0.0;
    zero_pose.position.z = 0.0;
    zero_pose.orientation.roll = 0.0;
    zero_pose.orientation.pitch = 0.0;
    zero_pose.orientation.yaw = 0.0;

    camera_pose = zero_pose;
    camera_offset = zero_pose;

    camera_pose.position.z = 30.0;
    camera_pose.orientation.pitch = carmen_degrees_to_radians (90.0);

    background_r = 0.0;
    background_g = 0.0;
    background_b = 0.0;

    glMatrixMode (GL_MODELVIEW);

}

void
set_background_color (double r, double g, double b)
{
    background_r = r;
    background_g = g;
    background_b = b;
}

void
set_camera (carmen_pose_3D_t pose)
{
    camera_pose = pose;
}

void
set_camera_offset (carmen_vector_3D_t offset)
{
    camera_offset.position = offset;
}

void
move_camera (carmen_vector_3D_t displacement)
{
    rotation_matrix* r_matrix = create_rotation_matrix (camera_pose.orientation);

    carmen_vector_3D_t displacement_world_coordinates = multiply_matrix_vector (r_matrix, displacement);

    camera_pose.position = add_vectors (camera_pose.position, displacement_world_coordinates);

    destroy_rotation_matrix (r_matrix);
}

void
rotate_camera (carmen_orientation_3D_t rotation)
{
    rotation_matrix* cam_matrix = create_rotation_matrix (camera_pose.orientation);
    rotation_matrix* r_matrix = create_rotation_matrix (rotation);

    rotation_matrix* new_cam_matrix = multiply_matrix_matrix (r_matrix, cam_matrix);

    camera_pose.orientation = get_angles_from_rotation_matrix (new_cam_matrix);

    destroy_rotation_matrix (cam_matrix);
    destroy_rotation_matrix (r_matrix);
    destroy_rotation_matrix (new_cam_matrix);
}

void
rotate_camera_offset (carmen_orientation_3D_t rotation)
{
    camera_offset.orientation.roll += rotation.roll;
    camera_offset.orientation.pitch += rotation.pitch;
    camera_offset.orientation.yaw += rotation.yaw;
}

void
reset_camera ()
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor (background_r, background_g, background_b, 1.0f);

    glClearDepth (1.0);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);

    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);

    glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable (GL_COLOR_MATERIAL);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (45.0f, WINDOW_WIDTH / WINDOW_HEIGHT, 0.1f, 4000.0f);
    ;
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    glLoadIdentity ();
    glRotated (90.0, 0.0, 0.0, 1.0);
    glRotated (90.0, 0.0, 1.0, 0.0);

    glRotated (carmen_radians_to_degrees (camera_pose.orientation.roll), -1.0, 0.0, 0.0);
    glRotated (carmen_radians_to_degrees (camera_pose.orientation.pitch), 0.0, -1.0, 0.0);
    glRotated (carmen_radians_to_degrees (camera_pose.orientation.yaw), 0.0, 0.0, -1.0);

    glTranslated (-camera_pose.position.x, -camera_pose.position.y, -camera_pose.position.z);

    glRotated (carmen_radians_to_degrees (camera_offset.orientation.roll), -1.0, 0.0, 0.0);
    glRotated (carmen_radians_to_degrees (camera_offset.orientation.pitch), 0.0, -1.0, 0.0);
    glRotated (carmen_radians_to_degrees (camera_offset.orientation.yaw), 0.0, 0.0, -1.0);

    glTranslated (-camera_offset.position.x, -camera_offset.position.y, -camera_offset.position.z);
}

void
draw_box (double length_x, double length_y, double length_z)
{
    glPushMatrix ();

    glBegin (GL_QUADS);

    glNormal3d (0.0, 0.0, -1.0);
    glVertex3d (-length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, -length_z / 2);
    glVertex3d (-length_x / 2, length_y / 2, -length_z / 2);

    glNormal3d (0.0, 0.0, 1.0);
    glVertex3d (-length_x / 2, -length_y / 2, length_z / 2);
    glVertex3d (length_x / 2, -length_y / 2, length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, length_z / 2);
    glVertex3d (-length_x / 2, length_y / 2, length_z / 2);

    glNormal3d (1.0, 0.0, 0.0);
    glVertex3d (length_x / 2, -length_y / 2, length_z / 2);
    glVertex3d (length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, length_z / 2);

    glNormal3d (-1.0, 0.0, 0.0);
    glVertex3d (-length_x / 2, -length_y / 2, length_z / 2);
    glVertex3d (-length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3d (-length_x / 2, length_y / 2, -length_z / 2);
    glVertex3d (-length_x / 2, length_y / 2, length_z / 2);

    glNormal3d (0.0, 1.0, 0.0);
    glVertex3d (-length_x / 2, length_y / 2, length_z / 2);
    glVertex3d (-length_x / 2, length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, -length_z / 2);
    glVertex3d (length_x / 2, length_y / 2, length_z / 2);

    glNormal3f (0.0f, -1.0f, 0.0f);
    glVertex3f (-length_x / 2, -length_y / 2, length_z / 2);
    glVertex3f (-length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3f (length_x / 2, -length_y / 2, -length_z / 2);
    glVertex3f (length_x / 2, -length_y / 2, length_z / 2);


    glEnd ();

    glPopMatrix ();
}

void
draw_particles (carmen_vector_3D_t *particles_pos, double *particles_weight, int num_particles, int color)
{
    int i;
    double minWeight;
    double maxWeight;

    if ((particles_weight == NULL) || (num_particles == 0))
    	return;

    maxWeight = minWeight = particles_weight[0];
    for (i = 1; i < num_particles; i++)
    {
    	int weight = particles_weight[i];
    	if (weight > maxWeight)
    		maxWeight = weight;
    	if (weight < minWeight)
    		minWeight = weight;
    }
    double diff = maxWeight - minWeight;

    glBegin (GL_POINTS);

    for (i = 0; i < num_particles; i++)
    {
    	if (color == 0)
    	{
    		if (diff != 0.0)
    			glColor3f (0.0, (maxWeight - particles_weight[i]) / diff, ((particles_weight[i] - minWeight) / diff) / 2.0 + 0.5);
    		else
    			glColor3f (0.0, 0.0, 1.0);
    	}
    	else if (color == 1)
    	{
    		if (diff != 0.0)
    			glColor3f (0.0, ((particles_weight[i] - minWeight) / diff) / 2.0 + 0.5, ((maxWeight - particles_weight[i]) / diff) / 2.0);
    		else
    			glColor3f (0.0, 1.0, 0.0);
    	}
    	else if (color == 2)
    	{
    		if (diff != 0.0)
    			glColor3f (((particles_weight[i] - minWeight) / diff) / 2.0 + 0.5, 0.0, ((maxWeight - particles_weight[i]) / diff) / 2.0);
    		else
    			glColor3f (1.0, 0.0, 0.0);
    	}
        glVertex3f (particles_pos[i].x, particles_pos[i].y, particles_pos[i].z);
    }

    glEnd ();
}

void
draw_arrow (float size)
{
    glBegin (GL_LINES);

    glVertex3d (-size / 2, 0.0f, 0.0f);
    glVertex3d (size / 2, 0.0f, 0.0f);

    glVertex3d (size / 2, 0.0f, 0.0f);
    glVertex3d (3 * size / 8, size / 8, 0.0f);

    glVertex3d (size / 2, 0.0f, 0.0f);
    glVertex3d (3 * size / 8, -size / 8, 0.0f);

    glEnd ();
}

static void
draw_axis (double length)
{
    length = 2 * length;

    glPushMatrix ();

    //glColor3f(0.4, 1.0, 0.4);

    glPushMatrix ();
    glTranslatef (length / 2.0, 0.0, 0.0);
    draw_box (2.0 * length, 0.05, 0.05);
    glPopMatrix ();

    draw_box (0.05, length, 0.05);
    draw_box (0.05, 0.05, length);

    glColor3f (0.0, 0.0, 0.0);
    double l;
    for (l = 0; l < length; l += 1.0)
    {
        glPushMatrix ();
        glTranslatef (0.0, 0.0, l);
        draw_box (0.06, 0.06, 0.02);
        glPopMatrix ();

        glPushMatrix ();
        glTranslatef (0.0, 0.0, -l);
        draw_box (0.06, 0.06, 0.02);
        glPopMatrix ();

        glPushMatrix ();
        glTranslatef (0.0, l, 0.0);
        draw_box (0.06, 0.02, 0.06);
        glPopMatrix ();

        glPushMatrix ();
        glTranslatef (0.0, -l, 0.0);
        draw_box (0.06, 0.02, 0.06);
        glPopMatrix ();

        glPushMatrix ();
        glTranslatef (l, 0.0, 0.0);
        draw_box (0.02, 0.06, 0.06);
        glPopMatrix ();

        glPushMatrix ();
        glTranslatef (-l, 0.0, 0.0);
        draw_box (0.02, 0.06, 0.06);
        glPopMatrix ();
    }

    glPopMatrix ();
}

static carmen_vector_3D_t
get_xsens_position_global_reference (carmen_pose_3D_t xsens_pose, carmen_pose_3D_t sensor_board_pose, carmen_pose_3D_t car_pose)
{
    rotation_matrix* board_to_car_matrix = create_rotation_matrix (sensor_board_pose.orientation);
    rotation_matrix* car_to_global_matrix = create_rotation_matrix (car_pose.orientation);

    carmen_vector_3D_t car_reference = multiply_matrix_vector (board_to_car_matrix, xsens_pose.position);
    car_reference = add_vectors (car_reference, sensor_board_pose.position);

    carmen_vector_3D_t global_reference = multiply_matrix_vector (car_to_global_matrix, car_reference);
    global_reference = add_vectors (global_reference, car_pose.position);

    destroy_rotation_matrix (board_to_car_matrix);
    destroy_rotation_matrix (car_to_global_matrix);

    return global_reference;
}

void
draw_xsens_orientation (carmen_orientation_3D_t xsens_orientation, double xsens_yaw_bias, carmen_pose_3D_t xsens_pose, carmen_pose_3D_t sensor_board_pose, carmen_pose_3D_t car_pose)
{
    carmen_vector_3D_t xsens_global_position = get_xsens_position_global_reference (xsens_pose, sensor_board_pose, car_pose);

    glPushMatrix ();

    glTranslatef (xsens_global_position.x, xsens_global_position.y, xsens_global_position.z);
    glRotatef (carmen_radians_to_degrees (xsens_orientation.yaw - xsens_yaw_bias), 0.0f, 0.0f, 1.0f);
    glRotatef (carmen_radians_to_degrees (xsens_orientation.pitch), 0.0f, 1.0f, 0.0f);
    glRotatef (carmen_radians_to_degrees (xsens_orientation.roll), 1.0f, 0.0f, 0.0f);

    draw_axis (1.0);

    glPopMatrix ();
}

void
draw_gps_orientation (double gps_orientation, int gps_heading_valid, carmen_orientation_3D_t xsens_orientation, carmen_pose_3D_t xsens_pose, carmen_pose_3D_t sensor_board_pose, carmen_pose_3D_t car_pose)
{
    carmen_vector_3D_t xsens_global_position = get_xsens_position_global_reference (xsens_pose, sensor_board_pose, car_pose);

    glPushMatrix ();

    glTranslatef (xsens_global_position.x, xsens_global_position.y, xsens_global_position.z);
    glRotatef (carmen_radians_to_degrees (gps_orientation), 0.0f, 0.0f, 1.0f);
    glRotatef (carmen_radians_to_degrees (xsens_orientation.pitch), 0.0f, 1.0f, 0.0f);
    glRotatef (carmen_radians_to_degrees (xsens_orientation.roll), 1.0f, 0.0f, 0.0f);

    if (gps_heading_valid)
    	draw_axis (1.0);
    else
    	draw_axis (0.5);

    glPopMatrix ();
}

void
draw_orientation_instruments (carmen_orientation_3D_t orientation, double r, double g, double b)
{
    glPushMatrix ();

    glColor3d (r, g, b);

    glLoadIdentity ();

    glTranslated (-0.16, -0.16, -0.5);

    glPushMatrix ();
    glRotated (carmen_radians_to_degrees (orientation.yaw), 0.0, 0.0, 1.0);
    draw_arrow (0.05);
    glPopMatrix ();

    glTranslated (0.1, 0.0, 0.0);

    glPushMatrix ();
    glRotated (carmen_radians_to_degrees (orientation.pitch), 0.0, 0.0, -1.0);
    draw_arrow (0.05);
    glPopMatrix ();

    glTranslated (0.1, 0.0, 0.0);

    glPushMatrix ();
    glRotated (carmen_radians_to_degrees (orientation.roll), 0.0, 0.0, 1.0);
    draw_arrow (0.05);
    glPopMatrix ();

    glPopMatrix ();
}

static void
draw_gps_symbol (void)
{
    glPushMatrix ();

    glColor3d (0.0, 1.0, 0.0);

    glTranslated (-5.0, 5.0, 0.0);

    glBegin (GL_LINE_LOOP);
    glVertex2d (5.0, 5.0);
    glVertex2d (5.0, 10.0);
    glVertex2d (15.0, 20.0);
    glVertex2d (20.0, 15.0);
    glVertex2d (10.0, 5.0);
    glEnd ();

    glBegin (GL_LINE_LOOP);
    glVertex2d (-5.0, -5.0);
    glVertex2d (-5.0, -10.0);
    glVertex2d (-15.0, -20.0);
    glVertex2d (-20.0, -15.0);
    glVertex2d (-10.0, -5.0);
    glEnd ();

    glBegin (GL_QUADS);
    glVertex2d (-7.0, 2.0);
    glVertex2d (-2.0, 7.0);
    glVertex2d (8.0, -3.0);
    glVertex2d (3.0, -8.0);
    glEnd ();

    glBegin (GL_LINES);
    glVertex2d (10.0, -5.0);
    glVertex2d (5.0, -10.0);

    glVertex2d (15.0, -4.0);
    glVertex2d (4.0, -15.0);

    glVertex2d (20.0, -3.0);
    glVertex2d (3.0, -20.0);

    glEnd ();

    glPopMatrix ();
}

static void
draw_fault_symbol (void)
{
    glPushMatrix ();

    glColor3d (1.0, 0.0, 0.0);
    glLineWidth (4.0);

    double pi = 3.14159265359;

    glBegin (GL_LINE_LOOP);

    double i;
    for (i = 0.0; i < 2 * pi; i += 2 * pi / 25.0)
    {
        glVertex2d (40 * sin (i), 40 * cos (i));
    }

    glEnd ();

    glLineWidth (3.0);

    glBegin (GL_LINES);

    glVertex2d (40 * sin (3 * pi / 4.0), 40 * cos (3 * pi / 4.0));
    glVertex2d (40 * sin (-pi / 4.0), 40 * cos (-pi / 4.0));

    glVertex2d (40 * sin (-3 * pi / 4.0), 40 * cos (-3 * pi / 4.0));
    glVertex2d (40 * sin (pi / 4.0), 40 * cos (pi / 4.0));

    glEnd ();

    glLineWidth (1.0);

    glPopMatrix ();
}

void
draw_gps_fault_signal (void)
{
    glPushMatrix ();

    glDisable (GL_DEPTH_TEST);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluOrtho2D (0, 1000.0, 0, 600.0);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();

    glTranslated (50, 550, 0);

    draw_gps_symbol ();
    draw_fault_symbol ();

    glPopMatrix ();
}

void
draw_laser_rays (point_cloud current_reading, carmen_vector_3D_t laser_position)
{

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin (GL_LINES);

    glColor4d (0.2, 0.2, 0.2, 0.2);

    int j;
    for (j = 0; j < current_reading.num_points; j++)
    {
        glVertex3d (laser_position.x, laser_position.y, laser_position.z);
        glVertex3d (current_reading.points[j].x, current_reading.points[j].y, current_reading.points[j].z);
    }

    glEnd ();
}


void
draw_number_associated(double x, double y, int associated, char *model_type)
{
	/*** MOVING OBJECTS MODULE ***/
	size_t len;    // String length
	int i;      // Iterator
	char *text; // Text

	len = 15;
    //  Allocate memory for a string of the specified size
    text = (char*)malloc(len * sizeof(char));

    snprintf(text, len, "%d %s", associated, model_type);

	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	glColor3f(0.0, 1.0, 0.0);
	glRasterPos3f(x, y, 2.5);

	glPushMatrix();
	for (i = 0; text[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
	glPopAttrib();

    //  Free the allocated memory for the string
    free(text);
}

void
draw_linear_velocity(double x, double y, double linear_velocity, double height)
{
	/*** MOVING OBJECTS MODULE ***/
	int len;       // String length
	int i;         //  Iterator
	char *text;    // Text

	len = 10;
    //  Allocate memory for a string of the specified size
    text = (char*)malloc(len * sizeof(char));

    snprintf(text, 10, "%.2f", linear_velocity);

	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	glColor3f(1.0, 1.0, 1.0);
	glRasterPos3f(x+0.5, y+0.5, height);

	glPushMatrix();
	for (i = 0; text[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
	glPopAttrib();

    //  Free the allocated memory for the string
    free(text);
}

void
draw_ldmrs_objects(carmen_laser_ldmrs_object *ldmrs_objects_tracking, int num_ldmrs_objects, double min_velocity)
{

	int i;
	rotation_matrix *rotate = NULL;

	for (i = 0; i < num_ldmrs_objects; i++)
	{
		if (ldmrs_objects_tracking[i].velocity < min_velocity)
			continue;
		carmen_pose_3D_t pos;
		carmen_vector_3D_t p1, p2, p3 , p4, p5, p6, p7, p8, t;
		carmen_vector_3D_t s1, s2, s3, s4; /* moving object direction arrow */
		double correction_wheel_height = 0.28;
		double W, L, H;

		pos.position.x = ldmrs_objects_tracking[i].x;
		pos.position.y = ldmrs_objects_tracking[i].y;
		pos.position.z = 0.0;

		pos.orientation.yaw =  ldmrs_objects_tracking[i].orientation;
		pos.orientation.roll = 0.0;
		pos.orientation.pitch = 0.0;

		W = ldmrs_objects_tracking[i].width;
		L = ldmrs_objects_tracking[i].lenght;
		H = 1.0;

//		W = moving_objects_tracking[i].width;
//		L = moving_objects_tracking[i].length;
//		H = moving_objects_tracking[i].height;

		rotate = compute_rotation_matrix(NULL, pos.orientation);

		glColor3d(0.0,1.0,1.0);

		p1.x = - L/2.0;
		p1.y = - W/2.0;
		p1.z = 0.0 - correction_wheel_height;

		p2.x = L/2.0;
		p2.y = - W/2.0;
		p2.z = 0.0 - correction_wheel_height;

		p3.x = L/2.0;
		p3.y = W/2.0;
		p3.z = 0.0 - correction_wheel_height;

		p4.x = - L/2.0;
		p4.y = W/2.0;
		p4.z = 0.0 - correction_wheel_height;

		p5.x = - L/2.0;
		p5.y = - W/2.0;
		p5.z = H - correction_wheel_height;

		p6.x = L/2.0;
		p6.y = - W/2.0;
		p6.z = H - correction_wheel_height;

		p7.x = L/2.0;
		p7.y = W/2.0;
		p7.z = H - correction_wheel_height;

		p8.x = - L/2.0;
		p8.y = W/2.0;
		p8.z = H - correction_wheel_height;

		t = multiply_matrix_vector(rotate, p1);
		p1 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p2);
		p2 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p3);
		p3 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p4);
		p4 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p5);
		p5 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p6);
		p6 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p7);
		p7 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p8);
		p8 = add_vectors(t, pos.position);

		glBegin (GL_LINES);

		glVertex3d (p1.x, p1.y, p1.z);
		glVertex3d (p2.x, p2.y, p2.z);

		glVertex3d (p2.x, p2.y, p2.z);
		glVertex3d (p3.x, p3.y, p3.z);

		glVertex3d (p3.x, p3.y, p3.z);
		glVertex3d (p4.x, p4.y, p4.z);

		glVertex3d (p4.x, p4.y, p4.z);
		glVertex3d (p1.x, p1.y, p1.z);
		//////////////////////////////

		glVertex3d (p5.x, p5.y, p5.z);
		glVertex3d (p6.x, p6.y, p6.z);

		glVertex3d (p6.x, p6.y, p6.z);
		glVertex3d (p7.x, p7.y, p7.z);

		glVertex3d (p7.x, p7.y, p7.z);
		glVertex3d (p8.x, p8.y, p8.z);

		glVertex3d (p8.x, p8.y, p8.z);
		glVertex3d (p5.x, p5.y, p5.z);
		//////////////////////////////

		glVertex3d (p1.x, p1.y, p1.z);
		glVertex3d (p5.x, p5.y, p5.z);

		glVertex3d (p2.x, p2.y, p2.z);
		glVertex3d (p6.x, p6.y, p6.z);

		glVertex3d (p3.x, p3.y, p3.z);
		glVertex3d (p7.x, p7.y, p7.z);

		glVertex3d (p4.x, p4.y, p4.z);
		glVertex3d (p8.x, p8.y, p8.z);

		glEnd ();

		/* Moving Object direction arrow */
		s1.x = 0.0;
		s1.y = 0.0;
		s1.z = H - correction_wheel_height;

		s2.x = L/2.0;
		s2.y = 0.0;
		s2.z = H - correction_wheel_height;

		s3.x = (L/2.0) - 0.3;
		s3.y = 0.2;
		s3.z = H - correction_wheel_height;

		s4.x = (L/2.0) - 0.3;
		s4.y = -0.2;
		s4.z = H - correction_wheel_height;

		t = multiply_matrix_vector(rotate, s1);
		s1 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s2);
		s2 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s3);
		s3 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s4);
		s4 = add_vectors(t, pos.position);

		glBegin(GL_LINES);
		/* Part of arrow: | */
		glVertex3d(s1.x, s1.y, s1.z);
		glVertex3d(s2.x, s2.y, s2.z);

		/* Part of arrow: / */
		glVertex3d(s3.x, s3.y, s3.z);
		glVertex3d(s2.x, s2.y, s2.z);

		/* Part of arrow: \ */
		glVertex3d(s4.x, s4.y, s4.z);
		glVertex3d(s2.x, s2.y, s2.z);

		glEnd();

		//center of object
		glPushAttrib(GL_POINT_BIT);
		glPointSize(5);
		glBegin(GL_POINTS);
		glVertex3d(s1.x, s1.y, s1.z);
		glEnd();
		glPopAttrib();

//		char class_name[50];
//
//		switch (ldmrs_objects_tracking[i].classId)
//		{
//			case 0:
//				strcpy(class_name,"Unclassified");
//				break;
//			case 1:
//				strcpy(class_name,"Small");
//				break;
//			case 2:
//				strcpy(class_name,"Big");
//				break;
//			case 3:
//				strcpy(class_name,"Pedestrian");
//				break;
//			case 4:
//				strcpy(class_name,"Bike");
//				break;
//			case 5:
//				strcpy(class_name,"Car");
//				break;
//			case 6:
//				strcpy(class_name,"Truck");
//				break;
//			default:
//				strcpy(class_name,"Unknown");
//				break;
//		}

		/* has to drawn after stuff above, so that it appears on top */
		//draw_number_associated(pos.position.x, pos.position.y, ldmrs_objects_tracking[i].id,"");
		draw_linear_velocity(pos.position.x, pos.position.y, ldmrs_objects_tracking[i].velocity,
				1.0);

		destroy_rotation_matrix(rotate);
	}
}


void
draw_tracking_moving_objects(moving_objects_tracking_t *moving_objects_tracking, int current_num_point_clouds,
		carmen_vector_3D_t offset, int draw_particles_flag)
{
	/*** MOVING OBJECTS MODULE ***/
	int i;
	rotation_matrix *rotate = NULL;

	for (i = 0; i < current_num_point_clouds; i++)
	{
		if (moving_objects_tracking[i].geometric_model == -1)
			continue;
		carmen_pose_3D_t pos;
		pos = moving_objects_tracking[i].moving_objects_pose;
		carmen_vector_3D_t p1, p2, p3 , p4, p5, p6, p7, p8, t;
		carmen_vector_3D_t s1, s2, s3, s4; /* moving object direction arrow */
		double correction_wheel_height = 0.28;
		double W, L, H;

		pos.position.x = pos.position.x - offset.x;
		pos.position.y = pos.position.y - offset.y;
		pos.position.z = 0.0;

		W = moving_objects_tracking[i].model_features.geometry.width;
		L = moving_objects_tracking[i].model_features.geometry.length;
		H = moving_objects_tracking[i].model_features.geometry.height;

//		W = moving_objects_tracking[i].width;
//		L = moving_objects_tracking[i].length;
//		H = moving_objects_tracking[i].height;

		rotate = compute_rotation_matrix(NULL, pos.orientation);

		glColor3d(moving_objects_tracking[i].model_features.red,
				moving_objects_tracking[i].model_features.green,
				moving_objects_tracking[i].model_features.blue);

		p1.x = - L/2.0;
		p1.y = - W/2.0;
		p1.z = 0.0 - correction_wheel_height;

		p2.x = L/2.0;
		p2.y = - W/2.0;
		p2.z = 0.0 - correction_wheel_height;

		p3.x = L/2.0;
		p3.y = W/2.0;
		p3.z = 0.0 - correction_wheel_height;

		p4.x = - L/2.0;
		p4.y = W/2.0;
		p4.z = 0.0 - correction_wheel_height;

		p5.x = - L/2.0;
		p5.y = - W/2.0;
		p5.z = H - correction_wheel_height;

		p6.x = L/2.0;
		p6.y = - W/2.0;
		p6.z = H - correction_wheel_height;

		p7.x = L/2.0;
		p7.y = W/2.0;
		p7.z = H - correction_wheel_height;

		p8.x = - L/2.0;
		p8.y = W/2.0;
		p8.z = H - correction_wheel_height;

		t = multiply_matrix_vector(rotate, p1);
		p1 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p2);
		p2 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p3);
		p3 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p4);
		p4 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p5);
		p5 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p6);
		p6 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p7);
		p7 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, p8);
		p8 = add_vectors(t, pos.position);

		glBegin (GL_LINES);

		glVertex3d (p1.x, p1.y, p1.z);
		glVertex3d (p2.x, p2.y, p2.z);

		glVertex3d (p2.x, p2.y, p2.z);
		glVertex3d (p3.x, p3.y, p3.z);

		glVertex3d (p3.x, p3.y, p3.z);
		glVertex3d (p4.x, p4.y, p4.z);

		glVertex3d (p4.x, p4.y, p4.z);
		glVertex3d (p1.x, p1.y, p1.z);
		//////////////////////////////

		glVertex3d (p5.x, p5.y, p5.z);
		glVertex3d (p6.x, p6.y, p6.z);

		glVertex3d (p6.x, p6.y, p6.z);
		glVertex3d (p7.x, p7.y, p7.z);

		glVertex3d (p7.x, p7.y, p7.z);
		glVertex3d (p8.x, p8.y, p8.z);

		glVertex3d (p8.x, p8.y, p8.z);
		glVertex3d (p5.x, p5.y, p5.z);
		//////////////////////////////

		glVertex3d (p1.x, p1.y, p1.z);
		glVertex3d (p5.x, p5.y, p5.z);

		glVertex3d (p2.x, p2.y, p2.z);
		glVertex3d (p6.x, p6.y, p6.z);

		glVertex3d (p3.x, p3.y, p3.z);
		glVertex3d (p7.x, p7.y, p7.z);

		glVertex3d (p4.x, p4.y, p4.z);
		glVertex3d (p8.x, p8.y, p8.z);

		glEnd ();

		/* Moving Object direction arrow */
		s1.x = 0.0;
		s1.y = 0.0;
		s1.z = H - correction_wheel_height;

		s2.x = L/2.0;
		s2.y = 0.0;
		s2.z = H - correction_wheel_height;

		s3.x = (L/2.0) - 0.3;
		s3.y = 0.2;
		s3.z = H - correction_wheel_height;

		s4.x = (L/2.0) - 0.3;
		s4.y = -0.2;
		s4.z = H - correction_wheel_height;

		t = multiply_matrix_vector(rotate, s1);
		s1 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s2);
		s2 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s3);
		s3 = add_vectors(t, pos.position);

		t = multiply_matrix_vector(rotate, s4);
		s4 = add_vectors(t, pos.position);

		glBegin(GL_LINES);
		/* Part of arrow: | */
		glVertex3d(s1.x, s1.y, s1.z);
		glVertex3d(s2.x, s2.y, s2.z);

		/* Part of arrow: / */
		glVertex3d(s3.x, s3.y, s3.z);
		glVertex3d(s2.x, s2.y, s2.z);

		/* Part of arrow: \ */
		glVertex3d(s4.x, s4.y, s4.z);
		glVertex3d(s2.x, s2.y, s2.z);

		glEnd();

		//center of object
		glPushAttrib(GL_POINT_BIT);
		glPointSize(5);
		glBegin(GL_POINTS);
		glVertex3d(s1.x, s1.y, s1.z);
		glEnd();
		glPopAttrib();

		/* has to drawn after stuff above, so that it appears on top */
		draw_number_associated(pos.position.x, pos.position.y, moving_objects_tracking[i].num_associated,
				moving_objects_tracking[i].model_features.model_name);
		draw_linear_velocity(pos.position.x, pos.position.y, moving_objects_tracking[i].linear_velocity,
				moving_objects_tracking[i].model_features.geometry.height);

		destroy_rotation_matrix(rotate);

		if (draw_particles_flag == 1) {
//			//todo para visualizar as particulas apenas
//			for(j = 0; j < 10; j++){
//				carmen_pose_3D_t pos;
//				pos = moving_objects_tracking[i].moving_objects_pose;
//				carmen_vector_3D_t p1, p2, p3 , p4, p5, p6, p7, p8, t;
//				carmen_vector_3D_t s1, s2, s3, s4; /* moving object direction arrow */
//				double correction_wheel_height = 0.28;
//				double W, L, H;
//
//				pos.position.x = moving_objects_tracking[i].particulas[j].pose.x - offset.x;
//				pos.position.y = moving_objects_tracking[i].particulas[j].pose.y - offset.y;
//				pos.position.z = 0.0;
//				pos.orientation.yaw = moving_objects_tracking[i].particulas[j].pose.theta;
//
//				W = moving_objects_tracking[i].particulas[j].geometry.width;
//				L = moving_objects_tracking[i].particulas[j].geometry.length;
//				H = moving_objects_tracking[i].particulas[j].geometry.height;
//
//		//		W = moving_objects_tracking[i].width;
//		//		L = moving_objects_tracking[i].length;
//		//		H = moving_objects_tracking[i].height;
//
//				rotate = compute_rotation_matrix(NULL, pos.orientation);
//
//				glColor3d(moving_objects_tracking[i].model_features.red -0.5,
//						moving_objects_tracking[i].model_features.green -0.5,
//						moving_objects_tracking[i].model_features.blue -0.5);
//
//				p1.x = - L/2.0;
//				p1.y = - W/2.0;
//				p1.z = 0.0 - correction_wheel_height;
//
//				p2.x = L/2.0;
//				p2.y = - W/2.0;
//				p2.z = 0.0 - correction_wheel_height;
//
//				p3.x = L/2.0;
//				p3.y = W/2.0;
//				p3.z = 0.0 - correction_wheel_height;
//
//				p4.x = - L/2.0;
//				p4.y = W/2.0;
//				p4.z = 0.0 - correction_wheel_height;
//
//				p5.x = - L/2.0;
//				p5.y = - W/2.0;
//				p5.z = H - correction_wheel_height;
//
//				p6.x = L/2.0;
//				p6.y = - W/2.0;
//				p6.z = H - correction_wheel_height;
//
//				p7.x = L/2.0;
//				p7.y = W/2.0;
//				p7.z = H - correction_wheel_height;
//
//				p8.x = - L/2.0;
//				p8.y = W/2.0;
//				p8.z = H - correction_wheel_height;
//
//				t = multiply_matrix_vector(rotate, p1);
//				p1 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p2);
//				p2 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p3);
//				p3 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p4);
//				p4 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p5);
//				p5 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p6);
//				p6 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p7);
//				p7 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, p8);
//				p8 = add_vectors(t, pos.position);
//
//				glBegin (GL_LINES);
//
//				glVertex3d (p1.x, p1.y, p1.z);
//				glVertex3d (p2.x, p2.y, p2.z);
//
//				glVertex3d (p2.x, p2.y, p2.z);
//				glVertex3d (p3.x, p3.y, p3.z);
//
//				glVertex3d (p3.x, p3.y, p3.z);
//				glVertex3d (p4.x, p4.y, p4.z);
//
//				glVertex3d (p4.x, p4.y, p4.z);
//				glVertex3d (p1.x, p1.y, p1.z);
//				//////////////////////////////
//
//				glVertex3d (p5.x, p5.y, p5.z);
//				glVertex3d (p6.x, p6.y, p6.z);
//
//				glVertex3d (p6.x, p6.y, p6.z);
//				glVertex3d (p7.x, p7.y, p7.z);
//
//				glVertex3d (p7.x, p7.y, p7.z);
//				glVertex3d (p8.x, p8.y, p8.z);
//
//				glVertex3d (p8.x, p8.y, p8.z);
//				glVertex3d (p5.x, p5.y, p5.z);
//				//////////////////////////////
//
//				glVertex3d (p1.x, p1.y, p1.z);
//				glVertex3d (p5.x, p5.y, p5.z);
//
//				glVertex3d (p2.x, p2.y, p2.z);
//				glVertex3d (p6.x, p6.y, p6.z);
//
//				glVertex3d (p3.x, p3.y, p3.z);
//				glVertex3d (p7.x, p7.y, p7.z);
//
//				glVertex3d (p4.x, p4.y, p4.z);
//				glVertex3d (p8.x, p8.y, p8.z);
//
//				glEnd ();
//
//				/* Moving Object direction arrow */
//				s1.x = 0.0;
//				s1.y = 0.0;
//				s1.z = H - correction_wheel_height;
//
//				s2.x = L/2.0;
//				s2.y = 0.0;
//				s2.z = H - correction_wheel_height;
//
//				s3.x = (L/2.0) - 0.3;
//				s3.y = 0.2;
//				s3.z = H - correction_wheel_height;
//
//				s4.x = (L/2.0) - 0.3;
//				s4.y = -0.2;
//				s4.z = H - correction_wheel_height;
//
//				t = multiply_matrix_vector(rotate, s1);
//				s1 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, s2);
//				s2 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, s3);
//				s3 = add_vectors(t, pos.position);
//
//				t = multiply_matrix_vector(rotate, s4);
//				s4 = add_vectors(t, pos.position);
//
//				glBegin(GL_LINES);
//				/* Part of arrow: | */
//				glVertex3d(s1.x, s1.y, s1.z);
//				glVertex3d(s2.x, s2.y, s2.z);
//
//				/* Part of arrow: / */
//				glVertex3d(s3.x, s3.y, s3.z);
//				glVertex3d(s2.x, s2.y, s2.z);
//
//				/* Part of arrow: \ */
//				glVertex3d(s4.x, s4.y, s4.z);
//				glVertex3d(s2.x, s2.y, s2.z);
//
//				glEnd();
//
//				//center of object
//				glPushAttrib(GL_POINT_BIT);
//				glPointSize(5);
//				glBegin(GL_POINTS);
//				glVertex3d(s1.x, s1.y, s1.z);
//				glEnd();
//				glPopAttrib();
//
//				/* has to drawn after stuff above, so that it appears on top */
//	//			draw_number_associated(pos.position.x, pos.position.y, moving_objects_tracking[i].num_associated,
//	//					moving_objects_tracking[i].model_features.model_name);
////				draw_linear_velocity(pos.position.x, pos.position.y, moving_objects_tracking[i].particulas[j].velocity,
////						moving_objects_tracking[i].particulas[j].geometry.height);
//
//				destroy_rotation_matrix(rotate);
//			}
		}
	}
}


void
draw_moving_objects_point_clouds(point_cloud *moving_objects_point_clouds, int cloud_size,
		carmen_vector_3D_t offset)
{
   // glPointSize (5);
    glBegin (GL_POINTS);


//    glColor3d(0.0,0.0,1.0);

    int i;
    for (i = 0; i < cloud_size; i++)
    {
        int j;
        for (j = 0; j < moving_objects_point_clouds[i].num_points; j++)
        {
            glColor3d (moving_objects_point_clouds[i].point_color[j].x, moving_objects_point_clouds[i].point_color[j].y, moving_objects_point_clouds[i].point_color[j].z);
            glVertex3d (moving_objects_point_clouds[i].points[j].x - offset.x, moving_objects_point_clouds[i].points[j].y - offset.y, moving_objects_point_clouds[i].points[j].z - moving_objects_point_clouds[i].car_position.z - 0.28);
        }
    }

    glEnd ();

}

static void
set_laser_point_color (double x, double y, double z)
{
    x = x;
    y = y;

    if (z < -5.0)
    {
        glColor3d (0.0, 0.0, 0.0);
    }
    else
    {
        glColor3d (0.0 - z, 0.1 + z / 10.0, (z + 3.0) / 6.0);
    }
    /*
    if(z < -2.0)
    {
            glColor3d(0.2, 0.2, 0.2);
    }
    else if(z < 0.0)
    {
            glColor3d( 0.0-z, 0.4, z-1.2);
    }
    else
    {
            glColor3d(z-1.2, z-1.2, 1.0);
    }*/
}

void
draw_stereo_point_cloud (point_cloud *stereo_point_cloud, int stereo_point_cloud_size)
{
    glBegin (GL_POINTS);

    int i;
    for (i = 0; i < stereo_point_cloud_size; i++)
    {
        int j;
        for (j = 0; j < stereo_point_cloud[i].num_points; j++)
        {
            glColor3d (stereo_point_cloud[i].point_color[j].x, stereo_point_cloud[i].point_color[j].y, stereo_point_cloud[i].point_color[j].z);
            glVertex3d (stereo_point_cloud[i].points[j].x, stereo_point_cloud[i].points[j].y, stereo_point_cloud[i].points[j].z);
        }
    }

    glEnd ();

}

void
draw_laser_points(point_cloud *laser_points, int laser_size)
{
    glBegin (GL_POINTS);

    //glColor3d(0.0,0.0,1.0);

    int i;
    for (i = 0; i < laser_size; i++)
    {
        int j;
        for (j = 0; j < laser_points[i].num_points; j++)
        {
            set_laser_point_color(laser_points[i].points[j].x, laser_points[i].points[j].y, laser_points[i].points[j].z);
            glVertex3d(laser_points[i].points[j].x, laser_points[i].points[j].y, laser_points[i].points[j].z);
        }
    }

    glEnd();
}

void
draw_laser_mesh (point_cloud *laser_points, int laser_size)
{
    glColor3d (0.0, 0.0, 1.0);

    int i;
    for (i = 1; i < laser_size; i++)
    {
        glBegin (GL_TRIANGLE_STRIP);

        int j;
        for (j = 0; j < laser_points[i].num_points && j < laser_points[i - 1].num_points; j++)
        {
            float ni = laser_points[i].points[j].y * laser_points[i - 1].points[j].z - laser_points[i].points[j].z * laser_points[i - 1].points[j].y;
            float nj = laser_points[i].points[j].x * laser_points[i - 1].points[j].z - laser_points[i].points[j].z * laser_points[i - 1].points[j].x;
            float nk = laser_points[i].points[j].x * laser_points[i - 1].points[j].y - laser_points[i].points[j].y * laser_points[i - 1].points[j].x;

            float sum = sqrt (ni * ni + nj * nj + nk * nk);


            ni = ni / sum;
            nj = nj / sum;
            nk = nk / sum;

            glNormal3d (ni, nj, nk);
            glVertex3d (laser_points[i].points[j].x, laser_points[i].points[j].y, laser_points[i].points[j].z);
            glVertex3d (laser_points[i - 1].points[j].x, laser_points[i - 1].points[j].y, laser_points[i - 1].points[j].z);

        }

        glEnd ();

    }

}

void
draw_velodyne_points (point_cloud *velodyne_points, int cloud_size)
{
   // glPointSize (5);
    glBegin (GL_POINTS);

    //glColor3d(0.0,0.0,1.0);

    int i;
    for (i = 0; i < cloud_size; i++)
    {
        int j;
        for (j = 0; j < velodyne_points[i].num_points; j++)
        {
            set_laser_point_color (velodyne_points[i].points[j].x, velodyne_points[i].points[j].y, velodyne_points[i].points[j].z);
            glVertex3d (velodyne_points[i].points[j].x, velodyne_points[i].points[j].y, velodyne_points[i].points[j].z);
        }
    }

    glEnd ();

}

void
draw_gps (carmen_vector_3D_t *gps_trail, int *gps_nr, int size)
{
    glPointSize (3.0);

    glBegin (GL_POINTS);

    for (int i = 0; i < size; i++)
    {
        if (gps_nr[i] == 1)
        	glColor3d (0.3, 1.0, 0.3);
        else if (gps_nr[i] == 2)
        	glColor3d (0.9, 0.0, 0.0);
        else if (gps_nr[i] == 3)
        	glColor3d (0.0, 0.9, 0.0);
        else // == 0
        	glColor3d (1.0, 1.0, 1.0);

        glVertex3d (gps_trail[i].x, gps_trail[i].y, gps_trail[i].z);
    }

    glEnd ();

    glPointSize (1.0);
}

void
draw_gps_xsens_xyz (carmen_vector_3D_t *gps_trail, int size)
{
    glPointSize (2.0);

    glBegin (GL_POINTS);

    glColor3d (0.1, 0.5, 0.2);

    int i;
    for (i = 0; i < size; i++)
    {
        glVertex3d (gps_trail[i].x, gps_trail[i].y, gps_trail[i].z);
    }

    glEnd ();

    glPointSize (1.0);
}

void
draw_odometry (carmen_vector_3D_t* odometry_trail, int size)
{
    glPointSize (3.0);
    glBegin (GL_POINTS);

    glColor3d (1.0, 1.0, 0.0);

    int i;
    for (i = 0; i < size; i++)
    {
        glVertex3d (odometry_trail[i].x, odometry_trail[i].y, odometry_trail[i].z);
    }

    glEnd ();
    glPointSize (1.0);
}

void
draw_localize_ackerman (carmen_vector_3D_t* localize_ackerman_trail, int size)
{
    glPointSize (3.0);
    glBegin (GL_POINTS);

    glColor3d (0.9, 0.6, 0.4);

    int i;
    for (i = 0; i < size; i++)
    {
        glVertex3d (localize_ackerman_trail[i].x, localize_ackerman_trail[i].y, localize_ackerman_trail[i].z);
    }

    glEnd ();
    glPointSize (1.0);
}

void
draw_map_image (carmen_vector_3D_t gps_position_at_turn_on, carmen_vector_3D_t map_center, double square_size, IplImage *img)
{
    carmen_vector_2D_t tex_coord_min;
    carmen_vector_2D_t tex_coord_max;
    char *map_image = NULL;

    if (img != NULL)
        map_image = update_map_image_texture2 (map_center, square_size, img);

    glTranslated (map_center.x - gps_position_at_turn_on.x, map_center.y - gps_position_at_turn_on.y, -0.56 / 2.0); // @@@ Alberto: 0.56 ee o diametro da roda do carro. Tem que fazer este valor chegar aqui vindo do carmen.ini
    glColor3d (1.0, 1.0, 1.0);
    glEnable (GL_TEXTURE_2D);
    glPushMatrix ();
    glTexImage2D (GL_TEXTURE_2D, 0, 3, TEXTURE_SIZE, TEXTURE_SIZE, 0, GL_BGR, GL_UNSIGNED_BYTE, map_image);
    glBindTexture (GL_TEXTURE_2D, map_image_texture_id);
    glBegin (GL_QUADS);
    tex_coord_min = get_map_image_tex_coord_min ();
    tex_coord_max = get_map_image_tex_coord_max ();
    glTexCoord2f (tex_coord_max.x, tex_coord_min.y);
    glVertex3d (-square_size / 2.0, -square_size / 2.0, 0.0f);
    glTexCoord2f (tex_coord_max.x, tex_coord_max.y);
    glVertex3d (square_size / 2.0, -square_size / 2.0, 0.0f);
    glTexCoord2f (tex_coord_min.x, tex_coord_max.y);
    glVertex3d (square_size / 2.0, square_size / 2.0, 0.0f);
    glTexCoord2f (tex_coord_min.x, tex_coord_min.y);
    glVertex3d (-square_size / 2.0, square_size / 2.0, 0.0f);
    glEnd ();
    glPopMatrix ();
    glDisable (GL_TEXTURE_2D);
}

void
draw_localize_image (bool draw_image_base, carmen_vector_3D_t gps_position_at_turn_on, carmen_pose_3D_t pose, char *image, int width, int height, int square_size)
{
    double z = pose.position.z;
    double dx = pose.position.x - gps_position_at_turn_on.x;
    double dy = pose.position.y - gps_position_at_turn_on.y;

    glPushMatrix ();
    glTranslated (dx, dy, z);
    glRotated (carmen_radians_to_degrees(pose.orientation.yaw-M_PI/2.0), 0.0, 0.0, 1.0);
    glColor3d (1.0, 1.0, 1.0);
    glEnable (GL_TEXTURE_2D);
    glPushMatrix ();
    if (draw_image_base)
    	glBindTexture(GL_TEXTURE_2D, localize_image_base_texture_id);
    else
    	glBindTexture(GL_TEXTURE_2D, localize_image_curr_texture_id);
    glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    glBegin (GL_QUADS);
    glNormal3d(1, 0, 0);
    glTexCoord2f (0.0f, 1.0f); glVertex3d (-square_size / 2.0, 0.0f, 0.0f);
    glTexCoord2f (1.0f, 1.0f); glVertex3d (square_size / 2.0, 0.0f, 0.0f);
    glTexCoord2f (1.0f, 0.0f); glVertex3d (square_size / 2.0, 0.0f, square_size / 2.0);
    glTexCoord2f (0.0f, 0.0f); glVertex3d (-square_size / 2.0, 0.0f, square_size / 2.0);
    glEnd ();
    glPopMatrix ();
    glDisable (GL_TEXTURE_2D);
    glPopMatrix ();
}
