#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/laser_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/stereo_point_cloud_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/velodyne_interface.h>
#include <carmen/download_map_interface.h>
#include <carmen/stereo_velodyne_interface.h>
#include <prob_map.h>
#include <carmen/mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/laser_ldmrs_utils.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/rddf_interface.h>
#include <GL/glew.h>
#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "viewer_3D.h"

#include "GLDraw.h"
#include "point_cloud_drawer.h"
#include "draw_car.h"
#include "velodyne_360_drawer.h"
#include "variable_velodyne_drawer.h"
#include "interface_drawer.h"
#include "Window.h"
#include "map_drawer.h"
#include "trajectory_drawer.h"
#include "velodyne_intensity_drawer.h"
#include "annotation_drawer.h"

// #define TEST_LANE_ANALYSIS
#ifdef TEST_LANE_ANALYSIS
#include <carmen/lane_analysis_interface.h>
#include "lane_analysis_drawer.h"
static int draw_lane_analysis_flag;
static lane_analysis_drawer *lane_drawer;
#endif


static int num_laser_devices;
static int moving_objects_point_clouds_size = 1;
static int stereo_point_cloud_size;
static int ldmrs_size;
static double ldmrs_min_velocity;
static int laser_size;
static int velodyne_size;
static int odometry_size;
static int gps_size;
static int localize_ackerman_size;
static int camera_square_size;

static point_cloud *stereo_point_cloud;
static int last_stereo_point_cloud;
static int stereo_initialized;

static point_cloud *ldmrs_points;
static int last_ldmrs_position;
static int ldmrs_initialized;

static point_cloud *front_bullbar_middle_laser_points;
static int front_bullbar_middle_laser_points_idx;

static point_cloud *laser_points;
static int last_laser_position;
static int laser_initialized;

static point_cloud *front_bullbar_left_corner_laser_points;
static int front_bullbar_left_corner_laser_points_idx;
static point_cloud *front_bullbar_right_corner_laser_points;
static int front_bullbar_right_corner_laser_points_idx;
static point_cloud *rear_bullbar_left_corner_laser_points;
static int rear_bullbar_left_corner_laser_points_idx;
static point_cloud *rear_bullbar_right_corner_laser_points;
static int rear_bullbar_right_corner_laser_points_idx;

static point_cloud *moving_objects_point_clouds;
static int last_moving_objects_point_clouds;
static int moving_objects_point_clouds_initialized;
moving_objects_tracking_t  *moving_objects_tracking;
int current_num_point_clouds;
int previous_num_point_clouds = 0;

int num_ldmrs_objects = 0;
carmen_laser_ldmrs_object *ldmrs_objects_tracking;
/************************************************************************
 * TODO: A variavel abaixo esta hard code, colocar para ser lida de algum lugar
 * **********************************************************************/
static int camera = 3;

static point_cloud *velodyne_points;
static int last_velodyne_position;
static int velodyne_initialized;

static carmen_vector_3D_t *odometry_trail;
static int last_odometry_trail;
static int odometry_initialized;

static carmen_vector_3D_t *localize_ackerman_trail;
static int last_localize_ackerman_trail;

static carmen_vector_3D_t gps_initial_pos;
static carmen_vector_3D_t *gps_trail;
static int *gps_nr;
static int last_gps_trail;
static int gps_initialized;

static double gps_heading = 0.0;
static int gps_heading_valid = 0;

static carmen_vector_3D_t xsens_xyz_initial_pos;
static carmen_vector_3D_t *xsens_xyz_trail;
static int next_xsens_xyz_trail;
static int xsens_xyz_initialized;
static int gps_fix_flag;

static carmen_vector_3D_t *fused_odometry_particles_pos;
static double *fused_odometry_particles_weight;
static int num_fused_odometry_particles;

static carmen_vector_3D_t *localizer_prediction_particles_pos;
static double *localizer_prediction_particles_weight;
static int num_localizer_prediction_particles;

static carmen_vector_3D_t *localizer_correction_particles_pos;
static double *localizer_correction_particles_weight;
static int num_localizer_correction_particles;

static carmen_pose_3D_t xsens_pose;
static carmen_pose_3D_t laser_pose;
static carmen_pose_3D_t car_pose;
static carmen_pose_3D_t camera_pose;
static carmen_pose_3D_t velodyne_pose;
static carmen_pose_3D_t laser_ldmrs_pose;
static carmen_pose_3D_t sensor_board_1_pose;
static carmen_pose_3D_t front_bullbar_pose;
static carmen_pose_3D_t front_bullbar_middle_pose;
static carmen_pose_3D_t front_bullbar_left_corner_pose;
static carmen_pose_3D_t front_bullbar_right_corner_pose;
static carmen_pose_3D_t rear_bullbar_pose;
static carmen_pose_3D_t rear_bullbar_left_corner_pose;
static carmen_pose_3D_t rear_bullbar_right_corner_pose;

static int sensor_board_1_laser_id;
static int front_bullbar_left_corner_laser_id;
static int front_bullbar_right_corner_laser_id;
static int rear_bullbar_left_corner_laser_id;
static int rear_bullbar_right_corner_laser_id;

static carmen_pose_3D_t car_fused_pose;

#define BOARD_1_LASER_HIERARCHY_SIZE 4

carmen_pose_3D_t* board_1_laser_hierarchy[BOARD_1_LASER_HIERARCHY_SIZE] = {&laser_pose, &sensor_board_1_pose, &car_pose, &car_fused_pose};

#define FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE 4

carmen_pose_3D_t* front_bullbar_middle_hierarchy[FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE]  = {&front_bullbar_middle_pose, &front_bullbar_pose, &car_pose, &car_fused_pose};

#define FRONT_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE 4
#define FRONT_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE 4

carmen_pose_3D_t* front_bullbar_left_corner_hierarchy[FRONT_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE]  = {&front_bullbar_left_corner_pose, &front_bullbar_pose, &car_pose, &car_fused_pose};
carmen_pose_3D_t* front_bullbar_right_corner_hierarchy[FRONT_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE] = {&front_bullbar_right_corner_pose, &front_bullbar_pose, &car_pose, &car_fused_pose};

#define REAR_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE 4
#define REAR_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE 4

carmen_pose_3D_t* rear_bullbar_left_corner_hierarchy[REAR_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE]  = {&rear_bullbar_left_corner_pose, &rear_bullbar_pose, &car_pose, &car_fused_pose};
carmen_pose_3D_t* rear_bullbar_right_corner_hierarchy[REAR_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE] = {&rear_bullbar_right_corner_pose, &rear_bullbar_pose, &car_pose, &car_fused_pose};

#define VELODYNE_HIERARCHY_SIZE 4
carmen_pose_3D_t* velodyne_hierarchy[VELODYNE_HIERARCHY_SIZE]  = {&velodyne_pose, &sensor_board_1_pose, &car_pose, &car_fused_pose};


//static double car_v = 0.0;
static double car_phi = 0.0;

//static carmen_pose_3D_t car_global_pos_pose;
static carmen_vector_3D_t car_fused_velocity;
static double car_fused_time;
static carmen_vector_3D_t gps_position_at_turn_on;

static int first_map_received = 0;
static carmen_vector_3D_t first_map_origin;

static carmen_orientation_3D_t xsens_orientation;
static double xsens_yaw_bias;

double orientation;
static carmen_vector_3D_t annotation_point;
static carmen_vector_3D_t annotation_point_world;
static std::vector<carmen_annotation_t> annotations;

static int point_size = 1; //size of line in OpenGL, it's read of ini

static int draw_particles_flag;
static int draw_points_flag;
static int draw_velodyne_flag;
static int draw_stereo_cloud_flag;
static int draw_car_flag;
static int draw_rays_flag;
static int draw_map_image_flag;
static int draw_localize_image_flag;
static int weight_type_flag;
static int draw_gps_flag;
static int draw_odometry_flag;
static int draw_xsens_gps_flag;
static int draw_map_flag;
static int draw_trajectory_flag1;
static int draw_trajectory_flag2;
static int draw_trajectory_flag3;
static int draw_xsens_orientation_flag;
static int draw_localize_ackerman_flag;
static int draw_annotation_flag;
static int draw_moving_objects_flag;
static int draw_gps_axis_flag;

static int follow_car_flag;
static int zero_z_flag;

static CarDrawer* car_drawer;
static point_cloud_drawer* ldmrs_drawer;
static point_cloud_drawer* laser_drawer;
static point_cloud_drawer* velodyne_drawer;
static velodyne_360_drawer* v_360_drawer;
static variable_velodyne_drawer* var_v_drawer;
static interface_drawer* i_drawer;
static map_drawer* m_drawer;
static trajectory_drawer* t_drawer1;
static trajectory_drawer* t_drawer2;
static trajectory_drawer* t_drawer3;
static std::vector<trajectory_drawer*> t_drawerTree;
static velodyne_intensity_drawer* v_int_drawer;
static AnnotationDrawer *annotation_drawer;


window *w = NULL;

void mouseFunc(int type, int button, int x, int y);
void keyPress(int code);
void keyRelease(int code);

static int argc_g;
static char** argv_g;

void init_stuff(int argc, char** argv);
void destroy_stuff();

static carmen_download_map_message download_map_message;
static int first_download_map_have_been_aquired = 0;
static int new_map_has_been_received = 0;

static carmen_localize_neural_imagepos_message localize_imagepos_base_message;
static carmen_localize_neural_imagepos_message localize_imagepos_curr_message;
static int localize_imagepos_base_initialized = 0;
static int localize_imagepos_curr_initialized = 0;

static int stereo_velodyne_vertical_resolution;
static int stereo_velodyne_flipped;
static int stereo_velodyne_num_points_cloud;

static int stereo_velodyne_vertical_roi_ini;
static int stereo_velodyne_vertical_roi_end;

static int stereo_velodyne_horizontal_roi_ini;
static int stereo_velodyne_horizontal_roi_end;

static double lastDisplayTime;

static double time_spent_by_each_scan;
static double distance_between_front_and_rear_axles;

static int force_velodyne_flag = 0;

static carmen_vector_3D_t
get_position_offset(void)
{
    if (odometry_initialized)
    {
        return gps_position_at_turn_on;
    }
    else if (xsens_xyz_initialized)
    {
        return xsens_xyz_initial_pos;
    }
    else if (gps_initialized)
    {
        return gps_initial_pos;
    }
    else if (first_map_received)
    {
        return first_map_origin;
    }

    carmen_vector_3D_t zero;
    zero.x = 0;
    zero.y = 0;
    zero.z = 0;

    return zero;
}


static void
rddf_annotation_handler(carmen_rddf_annotation_message *msg)
{
	for (unsigned int i = 0; i < annotations.size(); i++)
		if (annotations[i].annotation_description)
			free(annotations[i].annotation_description);

	annotations.clear();

	for (int i = 0; i < msg->num_annotations; i++)
	{
		carmen_annotation_t annotation;

		memcpy(&annotation, &(msg->annotations[i]), sizeof(carmen_annotation_t));
		if (msg->annotations[i].annotation_description)
		{
			annotation.annotation_description = (char *) calloc (strlen(msg->annotations[i].annotation_description) + 1, sizeof(char));
			strcpy(annotation.annotation_description, msg->annotations[i].annotation_description);
		}
		else
		{
			annotation.annotation_description = (char *) calloc (strlen((char *) " ") + 1, sizeof(char));
			strcpy(annotation.annotation_description, (char *) " ");
		}

		annotations.push_back(annotation);
	}

//	if (!has_annotation(*msg, annotations))
//	{
//		//annotation_drawer = addAnnotation(*msg, annotation_drawer);
//		annotations.push_back(*msg);
//	}
}


static void
carmen_fused_odometry_message_handler(carmen_fused_odometry_particle_message *odometry_message)
{
    xsens_yaw_bias = odometry_message->xsens_yaw_bias;

    gps_position_at_turn_on = odometry_message->gps_position_at_turn_on;
    odometry_initialized = 1;

    odometry_message->pose.position.z = 0.0;
    odometry_trail[last_odometry_trail] = odometry_message->pose.position;
    odometry_trail[last_odometry_trail] = sub_vectors(odometry_trail[last_odometry_trail], get_position_offset());

    last_odometry_trail++;

    if (last_odometry_trail >= odometry_size)
        last_odometry_trail -= odometry_size;
}


static void
carmen_fused_odometry_particle_message_handler(carmen_fused_odometry_particle_message *odometry_message)
{
    if (odometry_initialized && draw_particles_flag)
    {
        if (weight_type_flag == 2 || odometry_message->weight_type == weight_type_flag)
        {
            if (odometry_message->num_particles > num_fused_odometry_particles)
            {
                fused_odometry_particles_pos = (carmen_vector_3D_t *) realloc(fused_odometry_particles_pos,
                		odometry_message->num_particles * sizeof (carmen_vector_3D_t));
                fused_odometry_particles_weight = (double *) realloc(fused_odometry_particles_weight,
                		odometry_message->num_particles * sizeof (double));
            }
            num_fused_odometry_particles = odometry_message->num_particles;

            for (int i = 0; i < num_fused_odometry_particles; i++)
            {
                fused_odometry_particles_pos[i] = sub_vectors(odometry_message->particle_pos[i], get_position_offset());
                fused_odometry_particles_weight[i] = odometry_message->weights[i];

                if (zero_z_flag)
                    fused_odometry_particles_pos[i].z = 0.0;
            }
        }
    }
}


static void
carmen_localize_ackerman_particle_prediction_handler(carmen_localize_ackerman_particle_message *message)
{
    if (odometry_initialized && draw_particles_flag)
    {
		if (message->num_particles > num_localizer_prediction_particles)
		{
			localizer_prediction_particles_pos = (carmen_vector_3D_t *) realloc(localizer_prediction_particles_pos, message->num_particles * sizeof (carmen_vector_3D_t));
			localizer_prediction_particles_weight = (double *) realloc(localizer_prediction_particles_weight, message->num_particles * sizeof (double));
		}
		num_localizer_prediction_particles = message->num_particles;

		for (int i = 0; i < num_localizer_prediction_particles; i++)
		{
			carmen_vector_3D_t particle = {message->particles[i].x, message->particles[i].y, 0.0};
			localizer_prediction_particles_pos[i] = sub_vectors(particle, get_position_offset());
			localizer_prediction_particles_weight[i] = message->particles[i].weight;

            if (zero_z_flag)
                localizer_prediction_particles_pos[i].z = 0.0;
		}
    }
}

static void
carmen_localize_ackerman_particle_correction_handler(carmen_localize_ackerman_particle_message *message)
{
    if (odometry_initialized && draw_particles_flag)
    {
		if (message->num_particles > num_localizer_correction_particles)
		{
			localizer_correction_particles_pos = (carmen_vector_3D_t *) realloc(localizer_correction_particles_pos, message->num_particles * sizeof (carmen_vector_3D_t));
			localizer_correction_particles_weight = (double *) realloc(localizer_correction_particles_weight, message->num_particles * sizeof (double));
		}
		num_localizer_correction_particles = message->num_particles;

		for (int i = 0; i < num_localizer_correction_particles; i++)
		{
			carmen_vector_3D_t particle = {message->particles[i].x, message->particles[i].y, 0.0};
			localizer_correction_particles_pos[i] = sub_vectors(particle, get_position_offset());
			localizer_correction_particles_weight[i] = message->particles[i].weight;

            if (zero_z_flag)
                localizer_correction_particles_pos[i].z = 0.0;
		}
    }
}


static void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message* localize_ackerman_message)
{
    carmen_vector_3D_t pos;

    carmen_vector_3D_t offset = get_position_offset();

    pos.x = localize_ackerman_message->globalpos.x - offset.x;
    pos.y = localize_ackerman_message->globalpos.y - offset.y;
    pos.z = 0.0 - offset.z;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!! FILIPE !!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    car_fused_pose = localize_ackerman_message->pose;
    car_fused_pose.position = sub_vectors(car_fused_pose.position, get_position_offset());
    car_fused_pose.orientation.yaw = localize_ackerman_message->globalpos.theta;
    car_fused_velocity = localize_ackerman_message->velocity;
    car_fused_time = localize_ackerman_message->timestamp;
    car_phi = localize_ackerman_message->phi;

    car_fused_pose.position.z = 0.0;
    pos.z = 0.0;

    if (zero_z_flag)
    {
        car_fused_pose.position.z = 0.0;
        car_fused_pose.orientation.pitch = 0.0;
        car_fused_pose.orientation.roll = 0.0;
        pos.z = 0.0;
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!! FILIPE !!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    localize_ackerman_trail[last_localize_ackerman_trail] = pos;

    last_localize_ackerman_trail++;

    if (last_localize_ackerman_trail >= localize_ackerman_size)
        last_localize_ackerman_trail -= localize_ackerman_size;
}


static void
xsens_matrix_message_handler(carmen_xsens_global_matrix_message* xsens_matrix_message)
{
    rotation_matrix* xsens_matrix = create_rotation_matrix_from_matrix_inverse(xsens_matrix_message->matrix_data.m_data);
    xsens_orientation = get_angles_from_rotation_matrix(xsens_matrix);
    destroy_rotation_matrix(xsens_matrix);
}


static void
xsens_xyz_message_handler(carmen_xsens_xyz_message *xsens_xyz)
{
    static double last_timestamp = 0.0;
    static int k = 0;

    // GPS is not initializing properly on the first message, so it's waiting for 10 messages to initialize
    if (k < 10)
    {
        k++;
        return;
    }

    if (!xsens_xyz_initialized)
    {
        if (xsens_xyz->gps_fix)
        {
            xsens_xyz_initial_pos = xsens_xyz->position;
            xsens_xyz_initialized = 1;
        }
        return;
    }

    if (last_timestamp != 0.0 && fabs(xsens_xyz->timestamp - last_timestamp) > 3.0)
    {
        destroy_stuff();
        init_stuff(argc_g, argv_g);
        last_timestamp = 0.0;
        return;
    }

    gps_fix_flag = xsens_xyz->gps_fix;
    carmen_vector_3D_t new_pos = sub_vectors(xsens_xyz->position, get_position_offset());
    if (zero_z_flag)
        new_pos.z = 0.0;

    xsens_xyz_trail[next_xsens_xyz_trail] = new_pos;

    rotation_matrix* xsens_matrix = create_rotation_matrix_from_quaternions(xsens_xyz->quat);
    xsens_orientation = get_angles_from_rotation_matrix(xsens_matrix);

    destroy_rotation_matrix(xsens_matrix);

    next_xsens_xyz_trail++;
    if (next_xsens_xyz_trail >= gps_size)
        next_xsens_xyz_trail -= gps_size;

    last_timestamp = xsens_xyz->timestamp;
}


static void
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_mti)
{
    carmen_quaternion_t quat = {xsens_mti->quat_data.m_data[0], xsens_mti->quat_data.m_data[1], xsens_mti->quat_data.m_data[1], xsens_mti->quat_data.m_data[3]};
    rotation_matrix *xsens_matrix = create_rotation_matrix_from_quaternions(quat);

    xsens_orientation = get_angles_from_rotation_matrix(xsens_matrix);

    destroy_rotation_matrix(xsens_matrix);
}


carmen_vector_3D_t
get_laser_position(carmen_vector_3D_t car_position)
{
    // car_reference represents the position of the laser on the car frame of reference
    carmen_vector_3D_t car_reference = laser_pose.position;

    rotation_matrix* r_matrix = create_rotation_matrix(car_fused_pose.orientation);
    carmen_vector_3D_t global_reference = multiply_matrix_vector(r_matrix, car_reference);
    carmen_vector_3D_t laser_position = add_vectors(global_reference, car_position);

    destroy_rotation_matrix(r_matrix);
    laser_position.z = 0.0;

    return laser_position;
}


static carmen_vector_3D_t
get_point_position_global_reference(carmen_vector_3D_t car_position, carmen_vector_3D_t car_reference, rotation_matrix* car_to_global_matrix)
{

    //rotation_matrix* r_matrix = create_rotation_matrix(car_fused_pose.orientation);

    carmen_vector_3D_t global_reference = multiply_matrix_vector(car_to_global_matrix, car_reference);

    carmen_vector_3D_t point = add_vectors(global_reference, car_position);

    //destroy_rotation_matrix(r_matrix);

    return point;
}


static carmen_vector_3D_t
get_velodyne_point_car_reference(double rot_angle, double vert_angle, double range,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
    double cos_rot_angle = cos(rot_angle);
    double sin_rot_angle = sin(rot_angle);

    double cos_vert_angle = cos(vert_angle);
    double sin_vert_angle = sin(vert_angle);

    double xy_distance = range * cos_vert_angle;

    carmen_vector_3D_t velodyne_reference;

    velodyne_reference.x = (xy_distance * cos_rot_angle);
    velodyne_reference.y = (xy_distance * sin_rot_angle);
    velodyne_reference.z = (range * sin_vert_angle);

    carmen_vector_3D_t board_reference = multiply_matrix_vector(velodyne_to_board_matrix, velodyne_reference);
    board_reference = add_vectors(board_reference, velodyne_pose_position);

    carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
    car_reference = add_vectors(car_reference, sensor_board_1_pose_position);

    return (car_reference);
}


static carmen_vector_3D_t
create_point_colors_height(carmen_vector_3D_t point, carmen_vector_3D_t car_position)
{
    carmen_vector_3D_t colors;

    //double x = point.x;
    //double y = point.y;
    double z = point.z - car_position.z;

    if (z < -5.0)
    {
        colors.x = 1.0;
        colors.y = 0.0;
        colors.z = 0.0;
    }
    else
    {
        colors.x = 0.0 - z;
        colors.y = 0.1 + z / 10.0;
        colors.z = (z + 3.0) / 6.0;
    }

    return colors;
}


carmen_vector_3D_t
create_point_colors_intensity(double intensity)
{
        carmen_vector_3D_t colors;

        double intensity_normalized = 2.0 * (intensity / 255.0);

        colors.x = intensity_normalized;
        colors.y = intensity_normalized;
        colors.z = intensity_normalized;

        return colors;
}


int
compute_velodyne_points(point_cloud *velodyne_points, carmen_velodyne_partial_scan_message *velodyne_message,
		double *vertical_correction, int vertical_size,
		rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
		carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
    carmen_pose_3D_t car_interpolated_position;
    rotation_matrix r_matrix_car_to_global;

	double dt = velodyne_message->timestamp - car_fused_time - velodyne_message->number_of_32_laser_shots * time_spent_by_each_scan;
	int i;
	int range_max_points = 0;
	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++, dt += time_spent_by_each_scan)
	{
		car_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(car_fused_pose, dt, car_fused_velocity.x, car_phi,
				distance_between_front_and_rear_axles);
		compute_rotation_matrix(&r_matrix_car_to_global, car_interpolated_position.orientation);
		int j;
		for (j = 0; j < vertical_size; j++)
		{
			if (velodyne_message->partial_scan[i].distance[j] == 0)
			{
				range_max_points++;
				continue;
			}
			carmen_vector_3D_t point_position = get_velodyne_point_car_reference(-carmen_degrees_to_radians(velodyne_message->partial_scan[i].angle),
					carmen_degrees_to_radians(vertical_correction[j]), (double) velodyne_message->partial_scan[i].distance[j] / 500.0,
					velodyne_to_board_matrix, board_to_car_matrix, velodyne_pose_position, sensor_board_1_pose_position);
			carmen_vector_3D_t point_global_position = get_point_position_global_reference(car_interpolated_position.position, point_position,
					&r_matrix_car_to_global);
			velodyne_points->points[i * (vertical_size) + j - range_max_points] = point_global_position;
			velodyne_points->point_color[i * (vertical_size) + j - range_max_points] = create_point_colors_height(point_global_position,
					car_interpolated_position.position);
//			velodyne_points->point_color[i * (vertical_size) + j - range_max_points] = create_point_colors_intensity(velodyne_message->partial_scan[i].intensity[j]);
		}
	}

	return (range_max_points);
}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    static double vertical_correction[32] = {-30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0,
        -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
        -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67};

    static double last_timestamp = 0.0;

    if (!force_velodyne_flag)
		if (!odometry_initialized || !draw_velodyne_flag)
			return;

    if (last_timestamp == 0.0)
    {
        last_timestamp = velodyne_message->timestamp;
        return;
    }

    velodyne_initialized = 1;

//  Para ver so uma mensagem do velodyne
//    static int nt = 0;
//    nt++;
//    if ((nt < 10) || (nt > 10))
//    	return;
	
    last_velodyne_position++;
    if (last_velodyne_position >= velodyne_size)
        last_velodyne_position = 0;

    int num_points = velodyne_message->number_of_32_laser_shots * (32);

    if (num_points > velodyne_points[last_velodyne_position].num_points)
    {
        velodyne_points[last_velodyne_position].points = (carmen_vector_3D_t *) realloc(velodyne_points[last_velodyne_position].points, num_points * sizeof (carmen_vector_3D_t));
        velodyne_points[last_velodyne_position].point_color = (carmen_vector_3D_t *) realloc(velodyne_points[last_velodyne_position].point_color, num_points * sizeof (carmen_vector_3D_t));
    }
    velodyne_points[last_velodyne_position].num_points = num_points;
    velodyne_points[last_velodyne_position].car_position = car_fused_pose.position;
    velodyne_points[last_velodyne_position].timestamp = velodyne_message->timestamp;

    rotation_matrix* velodyne_to_board_matrix = create_rotation_matrix(velodyne_pose.orientation);
    rotation_matrix* board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	int range_max_points = compute_velodyne_points(&velodyne_points[last_velodyne_position], velodyne_message, vertical_correction, 32,
			velodyne_to_board_matrix, board_to_car_matrix, velodyne_pose.position, sensor_board_1_pose.position);
	velodyne_points[last_velodyne_position].num_points -= range_max_points;

	destroy_rotation_matrix(velodyne_to_board_matrix);
    destroy_rotation_matrix(board_to_car_matrix);

    if (draw_velodyne_flag == 2)
        add_point_cloud(velodyne_drawer, velodyne_points[last_velodyne_position]);

    if (draw_velodyne_flag == 3)
        add_velodyne_message(v_360_drawer, velodyne_message);

    if (draw_velodyne_flag == 5)
        velodyne_intensity_drawer_add_velodyne_message(v_int_drawer, velodyne_message, car_fused_pose, car_fused_velocity, car_fused_time);

    last_timestamp = velodyne_message->timestamp;
}


static void
velodyne_variable_scan_message_handler(carmen_velodyne_variable_scan_message* velodyne_message)
{
	add_variable_velodyne_message(var_v_drawer, velodyne_message, car_fused_pose, sensor_board_1_pose);
}


static void
sick_variable_scan_message_handler(carmen_velodyne_variable_scan_message* velodyne_message)
{
	add_variable_velodyne_message(var_v_drawer, velodyne_message, car_fused_pose, front_bullbar_pose);
}


void
carmen_ldmrs_new_draw_dispatcher(carmen_laser_ldmrs_new_message *laser_message,
		int parentsSize __attribute__ ((unused)), carmen_pose_3D_t **parents __attribute__ ((unused)),
		point_cloud *point_cloud, int *current_ldmrs_position)
{
	static double vertical_correction[4] = {-1.2, -0.4, 0.4, 1.2};

	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_new_convert_laser_scan_to_partial_velodyne_message(laser_message, laser_message->timestamp);

	// circular list of points history
    *current_ldmrs_position = (*current_ldmrs_position + 1) % ldmrs_size;
	int last_ldmrs_position = *current_ldmrs_position;

    int num_points = partial_scan_message.number_of_32_laser_shots * 4;

    if (point_cloud[last_ldmrs_position].points == NULL || point_cloud[last_ldmrs_position].point_color == NULL)
    {
        point_cloud[last_ldmrs_position].points = (carmen_vector_3D_t *) malloc (num_points * sizeof (carmen_vector_3D_t));
        point_cloud[last_ldmrs_position].point_color = (carmen_vector_3D_t *) malloc (num_points * sizeof (carmen_vector_3D_t));
        point_cloud[last_ldmrs_position].num_points = num_points;
    }
    else
    {
    	free(point_cloud[last_ldmrs_position].points);
    	free(point_cloud[last_ldmrs_position].point_color);

    	point_cloud[last_ldmrs_position].points = (carmen_vector_3D_t *) malloc (num_points * sizeof (carmen_vector_3D_t));
		point_cloud[last_ldmrs_position].point_color = (carmen_vector_3D_t *) malloc (num_points * sizeof (carmen_vector_3D_t));
    }

    point_cloud[last_ldmrs_position].num_points = num_points;
    point_cloud[last_ldmrs_position].car_position = car_fused_pose.position;
    point_cloud[last_ldmrs_position].timestamp = laser_message->timestamp;

    rotation_matrix *ldmrs_to_front_bulbar_rotation_matrix = create_rotation_matrix(laser_ldmrs_pose.orientation);
    rotation_matrix *front_bulbar_to_car_rotation_matrix = create_rotation_matrix(front_bullbar_pose.orientation);

    int range_max_points = compute_velodyne_points(&point_cloud[last_ldmrs_position], &partial_scan_message, vertical_correction, 4,
			ldmrs_to_front_bulbar_rotation_matrix, front_bulbar_to_car_rotation_matrix, laser_ldmrs_pose.position, front_bullbar_pose.position);
    point_cloud[last_ldmrs_position].num_points -= range_max_points;

	free(partial_scan_message.partial_scan);
	destroy_rotation_matrix(ldmrs_to_front_bulbar_rotation_matrix);
    destroy_rotation_matrix(front_bulbar_to_car_rotation_matrix);

    add_point_cloud(ldmrs_drawer, point_cloud[last_ldmrs_position]);
}


carmen_vector_3D_t
change_point_reference(carmen_vector_3D_t point_position, carmen_pose_3D_t reference_offset)
{
    rotation_matrix* r_matrix = create_rotation_matrix(reference_offset.orientation);
    carmen_vector_3D_t point_new_reference = multiply_matrix_vector(r_matrix, point_position);
    point_new_reference = add_vectors(point_new_reference, reference_offset.position);
    destroy_rotation_matrix(r_matrix);

    return point_new_reference;
}


carmen_vector_3D_t
get_laser_reading_position_car_reference(double angle, double range)
{
    carmen_vector_3D_t laser_reference;

    laser_reference.x = cos(angle) * range;
    laser_reference.y = sin(angle) * range;
    laser_reference.z = 0;

    rotation_matrix* laser_to_board_matrix = create_rotation_matrix(laser_pose.orientation);
    carmen_vector_3D_t board_reference = multiply_matrix_vector(laser_to_board_matrix, laser_reference);
    board_reference = add_vectors(board_reference, laser_pose.position);

	rotation_matrix* board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);
    carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
    car_reference = add_vectors(car_reference, sensor_board_1_pose.position);

    destroy_rotation_matrix(laser_to_board_matrix);
    destroy_rotation_matrix(board_to_car_matrix);

    return car_reference;
}


carmen_vector_3D_t
get_world_position(carmen_vector_3D_t position, int graphSize, carmen_pose_3D_t* sceneGraph[], int fromIndex)
{
	for (int i = fromIndex; i < graphSize; i++)
	{
		carmen_pose_3D_t* parent = sceneGraph[i];
		
		rotation_matrix* local_rotation = create_rotation_matrix(parent->orientation);
		
		// rotate
		position = multiply_matrix_vector(local_rotation, position);
		
		// translate
		position = add_vectors(position, parent->position);
		
		destroy_rotation_matrix(local_rotation);
	}

	return (position);
}


carmen_vector_3D_t
get_laser_reading_position_from_reference(double angle, double range, int parentsSize, carmen_pose_3D_t* parents[])
{
    carmen_vector_3D_t laser_point;

    laser_point.x = cos(angle) * range;
    laser_point.y = sin(angle) * range;
    laser_point.z = 0;
	
	return get_world_position(laser_point, parentsSize, parents, 0);
}


carmen_vector_3D_t
get_ldmrs_reading_position_from_reference(double hAngle, double vAngle, double range, int parentsSize, carmen_pose_3D_t* parents[])
{
    carmen_vector_3D_t laser_point;

    laser_point.x = range * cos(hAngle) * cos(vAngle);
    laser_point.y = range * sin(hAngle);
    laser_point.z = range * cos(hAngle) * sin(vAngle);

	return get_world_position(laser_point, parentsSize, parents, 0);
}


carmen_vector_3D_t
get_world_position(int graphSize, carmen_pose_3D_t* sceneGraph[])
{
	if (graphSize <= 0)
		fprintf(stderr, "Viewer3D: error: Call to get_world_position with empty arguments");
	
	carmen_pose_3D_t* pose = sceneGraph[0];
	return get_world_position(pose->position, graphSize, sceneGraph, 1);
}


void
generate_octomap_file(point_cloud current_reading, carmen_fused_odometry_message odometry_message, char *filename)
{
    int j;
    FILE *fp;

    static double initial_time = 0.0;

    if (initial_time == 0.0)
        initial_time = odometry_message.timestamp;

    if (current_reading.num_points > 0 && (odometry_message.timestamp - initial_time) >= 90.0 && (odometry_message.timestamp - initial_time) <= 100.0)
    {
        fp = fopen(filename, "a");

        if (fp == NULL)
        {
            fprintf(stderr, "Can't open output file %s!\n", filename);
            exit(1);
        }

        fprintf(fp, "NODE %f %f %f %f %f %f\n",
                odometry_message.pose.position.x + laser_pose.position.x, odometry_message.pose.position.y, odometry_message.pose.position.z + laser_pose.position.z, odometry_message.pose.orientation.roll, odometry_message.pose.orientation.pitch, odometry_message.pose.orientation.yaw + carmen_degrees_to_radians(90.0));

        for (j = 0; j < current_reading.num_points; j++)
            fprintf(fp, "%f %f %f\n", current_reading.points[j].x, current_reading.points[j].y, current_reading.points[j].z);

        fclose(fp);
    }
}


static void
carmen_ldmrs_add_point_cloud(point_cloud* point_cloud, int last_ldmrs_position, int parentsSize, carmen_pose_3D_t** parents, double hAngle, double vAngle, double range, int *j)
{
    if (range > 0.0 && range <= 200.0)
    {
    	point_cloud[last_ldmrs_position].points[(*j)] = get_ldmrs_reading_position_from_reference(hAngle, vAngle, range, parentsSize, parents);

    	point_cloud[last_ldmrs_position].point_color[(*j)].x = 1.0;
    	point_cloud[last_ldmrs_position].point_color[(*j)].y = 1.0;
    	point_cloud[last_ldmrs_position].point_color[(*j)].z = 1.0;
    	(*j)++;
    }
    else
    {
    	// This removes points at infinity
        point_cloud[last_ldmrs_position].num_points--;
    }
}


static void
carmen_ldmrs_draw_dispatcher(carmen_laser_ldmrs_message* laser_message, int parentsSize, carmen_pose_3D_t** parents, point_cloud* point_cloud, int* current_ldmrs_position)
{
	// circular list of points history
    *current_ldmrs_position = (*current_ldmrs_position + 1) % ldmrs_size;
	int last_ldmrs_position = *current_ldmrs_position;

    int num_points = laser_message->scan_points;

    if (point_cloud[last_ldmrs_position].points == NULL || point_cloud[last_ldmrs_position].point_color == NULL)
    {
        point_cloud[last_ldmrs_position].points = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));
        point_cloud[last_ldmrs_position].point_color = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));
        point_cloud[last_ldmrs_position].num_points = num_points;
    }
    else
    {
    	free(point_cloud[last_ldmrs_position].points);
    	free(point_cloud[last_ldmrs_position].point_color);

    	point_cloud[last_ldmrs_position].points = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));
		point_cloud[last_ldmrs_position].point_color = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));

    }

    point_cloud[last_ldmrs_position].num_points = num_points;
    point_cloud[last_ldmrs_position].car_position = car_fused_pose.position;
    point_cloud[last_ldmrs_position].timestamp = laser_message->timestamp;

    int j = 0;
    double hAngle, vAngle, range;
    for (int i = 0; i < num_points; i++)
    {
        hAngle = laser_message->arraypoints[i].horizontal_angle;
        vAngle = laser_message->arraypoints[i].vertical_angle;
        range = laser_message->arraypoints[i].radial_distance;
        carmen_ldmrs_add_point_cloud(point_cloud, last_ldmrs_position, parentsSize, parents, hAngle, vAngle, range, &j);
    }
    add_point_cloud(ldmrs_drawer, point_cloud[last_ldmrs_position]);
}


static void
carmen_laser_ldmrs_message_handler(carmen_laser_ldmrs_message* laser_message)
{
    ldmrs_initialized = 1;
	carmen_ldmrs_draw_dispatcher(laser_message, FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE, front_bullbar_middle_hierarchy, front_bullbar_middle_laser_points, &front_bullbar_middle_laser_points_idx);
}


static void
carmen_laser_ldmrs_new_message_handler(carmen_laser_ldmrs_new_message* laser_message)
{
    ldmrs_initialized = 1;
	carmen_ldmrs_new_draw_dispatcher(laser_message, FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE, front_bullbar_middle_hierarchy, front_bullbar_middle_laser_points, &front_bullbar_middle_laser_points_idx);
}

//object
void
carmen_laser_ldmrs_objects_message_handler(carmen_laser_ldmrs_objects_message* laser_message)
{
    ldmrs_initialized = 1;

    if(laser_message->num_objects > 0)
    {
		if(num_ldmrs_objects != laser_message->num_objects)
		{
			num_ldmrs_objects = laser_message->num_objects;
			ldmrs_objects_tracking = (carmen_laser_ldmrs_object *) realloc(ldmrs_objects_tracking, num_ldmrs_objects * sizeof(carmen_laser_ldmrs_object));
			carmen_test_alloc(ldmrs_objects_tracking);
		}

		for(int i = 0; i < num_ldmrs_objects; i++)
		{
			ldmrs_objects_tracking[i].id = laser_message->objects_list[i].id;
			ldmrs_objects_tracking[i].lenght = laser_message->objects_list[i].lenght;
			ldmrs_objects_tracking[i].width = laser_message->objects_list[i].width;
			ldmrs_objects_tracking[i].orientation = carmen_normalize_theta(laser_message->objects_list[i].orientation + car_fused_pose.orientation.yaw);
			ldmrs_objects_tracking[i].velocity = laser_message->objects_list[i].velocity;
			double x = (laser_message->objects_list[i].x + front_bullbar_pose.position.x) * cos(car_fused_pose.orientation.yaw) - (laser_message->objects_list[i].y + front_bullbar_pose.position.y) * sin(car_fused_pose.orientation.yaw);
			double y = (laser_message->objects_list[i].x + front_bullbar_pose.position.x) * sin(car_fused_pose.orientation.yaw) + (laser_message->objects_list[i].y + front_bullbar_pose.position.y) * cos(car_fused_pose.orientation.yaw);
			ldmrs_objects_tracking[i].x = x + car_fused_pose.position.x;
			ldmrs_objects_tracking[i].y = y + car_fused_pose.position.y;
			ldmrs_objects_tracking[i].classId = laser_message->objects_list[i].classId;
		}
    }
}


//object
static void
carmen_laser_ldmrs_objects_data_message_handler(carmen_laser_ldmrs_objects_data_message* laser_message)
{
    ldmrs_initialized = 1;

    if(laser_message->num_objects > 0)
    {
		if(num_ldmrs_objects != laser_message->num_objects)
		{
			num_ldmrs_objects = laser_message->num_objects;
			ldmrs_objects_tracking = (carmen_laser_ldmrs_object *) realloc(ldmrs_objects_tracking, num_ldmrs_objects * sizeof(carmen_laser_ldmrs_object));
			carmen_test_alloc(ldmrs_objects_tracking);
		}

		for(int i = 0; i < num_ldmrs_objects; i++)
		{
			ldmrs_objects_tracking[i].id = laser_message->objects_data_list[i].object_id;
			//pode estar invertido
			ldmrs_objects_tracking[i].width = laser_message->objects_data_list[i].object_box_lenght;
			ldmrs_objects_tracking[i].lenght = laser_message->objects_data_list[i].object_box_width;
			//orientacao
			ldmrs_objects_tracking[i].orientation = carmen_normalize_theta(laser_message->objects_data_list[i].object_box_orientation + car_fused_pose.orientation.yaw);

			// calcula a velocidade
//			if(laser_message->objects_data_list[i].abs_velocity_sigma_x < 1.5 && laser_message->objects_data_list[i].abs_velocity_sigma_y < 1.5)
//			{
				double abs_velocity = sqrt( pow(laser_message->objects_data_list[i].abs_velocity_x,2) + pow(laser_message->objects_data_list[i].abs_velocity_y,2));
				ldmrs_objects_tracking[i].velocity = abs_velocity;

				if(laser_message->objects_data_list[i].object_age < 15)
				{
					ldmrs_objects_tracking[i].velocity = 0.0;
				}

//				printf("id %3d age %5d r_x %8.5lf r_y %8.5lf\n", laser_message->objects_data_list[i].object_id,
//						laser_message->objects_data_list[i].object_age, laser_message->objects_data_list[i].reference_point_x, laser_message->objects_data_list[i].reference_point_y);
//			}
//			else
//			{
//				ldmrs_objects_tracking[i].velocity = 0.0;
//			}

			// calcula a posição
			double x = (laser_message->objects_data_list[i].object_box_center_x + front_bullbar_pose.position.x) * cos(car_fused_pose.orientation.yaw) - (laser_message->objects_data_list[i].object_box_center_y + front_bullbar_pose.position.y) * sin(car_fused_pose.orientation.yaw);
			double y = (laser_message->objects_data_list[i].object_box_center_x + front_bullbar_pose.position.x) * sin(car_fused_pose.orientation.yaw) + (laser_message->objects_data_list[i].object_box_center_y + front_bullbar_pose.position.y) * cos(car_fused_pose.orientation.yaw);
			ldmrs_objects_tracking[i].x = x + car_fused_pose.position.x;
			ldmrs_objects_tracking[i].y = y + car_fused_pose.position.y;

			ldmrs_objects_tracking[i].classId = laser_message->objects_data_list[i].class_id;
		}
//		printf("\n");
    }
}


static void
carmen_laser_draw_dispatcher(carmen_laser_laser_message* laser_message, int parentsSize, carmen_pose_3D_t** parents, point_cloud* point_cloud, int* current_laser_position)
{
	// circular list of points history
    *current_laser_position = (*current_laser_position + 1) % laser_size;
	int last_laser_position = *current_laser_position;

    int num_points = laser_message->num_readings;

    if (point_cloud[last_laser_position].points == NULL || point_cloud[last_laser_position].point_color == NULL)
    {
        point_cloud[last_laser_position].points = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));
        point_cloud[last_laser_position].point_color = (carmen_vector_3D_t*) malloc (num_points * sizeof (carmen_vector_3D_t));

        point_cloud[last_laser_position].num_points = num_points;
    }

    point_cloud[last_laser_position].num_points = num_points;
    point_cloud[last_laser_position].car_position = car_fused_pose.position;
    point_cloud[last_laser_position].timestamp = laser_message->timestamp;

    //rotation_matrix* r_matrix_car_to_global = create_rotation_matrix(car_fused_pose.orientation);

    int i;
    int j = 0;
    for (i = 0; i < num_points; i++)
    {
        double angle = laser_message->config.start_angle + i * laser_message->config.angular_resolution;
        double range = laser_message->range[i];

        //point_cloud[last_laser_position].points[j] = get_point_position_global_reference(point_cloud[last_laser_position].car_position, point_cloud_car[last_laser_position].points[j], //r_matrix_car_to_global);

		point_cloud[last_laser_position].points[j] = get_laser_reading_position_from_reference(angle, range, parentsSize, parents);

        //point_cloud[last_laser_position].points[j].z = 1.0;

        point_cloud[last_laser_position].point_color[j].x = 1.0;
        point_cloud[last_laser_position].point_color[j].y = 1.0;
        point_cloud[last_laser_position].point_color[j].z = 1.0;

        // This removes points at infinity
        if (range > 50.0)
        {
            point_cloud[last_laser_position].num_points--;
        }
        else
        {
            j++;
        }
    }
    //destroy_rotation_matrix(r_matrix_car_to_global);
    add_point_cloud(laser_drawer, point_cloud[last_laser_position]);
}


static void
carmen_laser_laser_message_handler(carmen_laser_laser_message* laser_message)
{
	//printf("carmen_laser_laser_message_handler %d\n", laser_message->id);
//    if (!odometry_initialized)
//        return;
	//printf("Odometry OK\n");
//    if (!draw_points_flag)
//        return;

    laser_initialized = 1;
	//printf("Laser message id: %d\n", laser_message->id);
	if (laser_message->id == front_bullbar_left_corner_laser_id) {
		carmen_laser_draw_dispatcher(laser_message, FRONT_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, front_bullbar_left_corner_hierarchy, front_bullbar_left_corner_laser_points, &front_bullbar_left_corner_laser_points_idx);
	}
	if (laser_message->id == front_bullbar_right_corner_laser_id) {
		carmen_laser_draw_dispatcher(laser_message,  FRONT_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, front_bullbar_right_corner_hierarchy, front_bullbar_right_corner_laser_points, &front_bullbar_right_corner_laser_points_idx);
	}
	if (laser_message->id == rear_bullbar_left_corner_laser_id) {
		carmen_laser_draw_dispatcher(laser_message,  REAR_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, rear_bullbar_left_corner_hierarchy, rear_bullbar_left_corner_laser_points, &rear_bullbar_left_corner_laser_points_idx);
	}
	if (laser_message->id == rear_bullbar_right_corner_laser_id) {
		carmen_laser_draw_dispatcher(laser_message,  REAR_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, rear_bullbar_right_corner_hierarchy, rear_bullbar_right_corner_laser_points, &rear_bullbar_right_corner_laser_points_idx);
	}
	if (laser_message->id == sensor_board_1_laser_id) {
		carmen_laser_draw_dispatcher(laser_message,  BOARD_1_LASER_HIERARCHY_SIZE, board_1_laser_hierarchy, laser_points, &last_laser_position);
	}
}

void
init_moving_objects_tracking(int c_num_point_clouds, int p_num_point_clouds)
{
	if (c_num_point_clouds != p_num_point_clouds)
	{
		free(moving_objects_tracking);
//		moving_objects_tracking = (moving_objects_tracking_t*) malloc
//				(c_num_point_clouds * sizeof(moving_objects_tracking_t));
	}
	moving_objects_tracking = (moving_objects_tracking_t*) malloc
			(c_num_point_clouds * sizeof(moving_objects_tracking_t));
}

static void
init_moving_objects_point_cloud(int num_points, double timestamp)
{
	int i;

	if (moving_objects_point_clouds[last_moving_objects_point_clouds].num_points != num_points)
	{
		for (i = 0; i < moving_objects_point_clouds_size; i++)
		{
			free(moving_objects_point_clouds[i].points);
			free(moving_objects_point_clouds[i].point_color);
		}

		moving_objects_point_clouds[last_moving_objects_point_clouds].points = (carmen_vector_3D_t*) malloc(num_points * sizeof(carmen_vector_3D_t));
		moving_objects_point_clouds[last_moving_objects_point_clouds].point_color = (carmen_vector_3D_t*) malloc(num_points * sizeof(carmen_vector_3D_t));

		moving_objects_point_clouds[last_moving_objects_point_clouds].num_points = num_points;

	}
	moving_objects_point_clouds[last_moving_objects_point_clouds].num_points = num_points;
	moving_objects_point_clouds[last_moving_objects_point_clouds].car_position = car_fused_pose.position;
	moving_objects_point_clouds[last_moving_objects_point_clouds].timestamp = timestamp;

}

int account_number_of_points_point_clouds(int num_points,
		carmen_moving_objects_point_clouds_message* moving_objects_point_clouds_message)
{
	int i;

	current_num_point_clouds = moving_objects_point_clouds_message->num_point_clouds;

	num_points = 0;

	for (i = 0; i < current_num_point_clouds; i++)
		num_points += moving_objects_point_clouds_message->point_clouds[i].point_size;

	return num_points;
}

static void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	int i, j;
	int num_points;
	num_points = 0;

	if (!odometry_initialized)
        return;

	moving_objects_point_clouds_initialized = 1;

	last_moving_objects_point_clouds++;
	if (last_moving_objects_point_clouds >= moving_objects_point_clouds_size)
	{
		last_moving_objects_point_clouds = 0;
	}

	num_points = account_number_of_points_point_clouds(num_points, moving_objects_point_clouds_message);
	init_moving_objects_point_cloud(num_points, moving_objects_point_clouds_message->timestamp);

	init_moving_objects_tracking(current_num_point_clouds, previous_num_point_clouds);
	previous_num_point_clouds = current_num_point_clouds;

	int k;

	j = 0;

	for (i = 0; i < current_num_point_clouds; i++)
	{
		moving_objects_tracking[i].moving_objects_pose.orientation.yaw = moving_objects_point_clouds_message->point_clouds[i].orientation;
		moving_objects_tracking[i].moving_objects_pose.orientation.roll = 0.0;
		moving_objects_tracking[i].moving_objects_pose.orientation.pitch = 0.0;
		moving_objects_tracking[i].moving_objects_pose.position.x = moving_objects_point_clouds_message->point_clouds[i].object_pose.x;
		moving_objects_tracking[i].moving_objects_pose.position.y = moving_objects_point_clouds_message->point_clouds[i].object_pose.y;
		moving_objects_tracking[i].moving_objects_pose.position.z = moving_objects_point_clouds_message->point_clouds[i].object_pose.z;
		moving_objects_tracking[i].length = moving_objects_point_clouds_message->point_clouds[i].length;
		moving_objects_tracking[i].width = moving_objects_point_clouds_message->point_clouds[i].width;
		moving_objects_tracking[i].height = moving_objects_point_clouds_message->point_clouds[i].height;
		moving_objects_tracking[i].linear_velocity = moving_objects_point_clouds_message->point_clouds[i].linear_velocity;
		moving_objects_tracking[i].geometric_model = moving_objects_point_clouds_message->point_clouds[i].geometric_model;
		moving_objects_tracking[i].model_features = moving_objects_point_clouds_message->point_clouds[i].model_features;
		moving_objects_tracking[i].num_associated = moving_objects_point_clouds_message->point_clouds[i].num_associated;

		//fixme para a visualização das particulas
//		moving_objects_tracking[i].particulas = (particle_print_t*) malloc(400 * sizeof(particle_print_t));
//		for(int k = 0; k < 400; k++){
//			moving_objects_tracking[i].particulas[k] = moving_objects_point_clouds_message->point_clouds[i].particulas[k];
//		}

		for (k = 0; k < moving_objects_point_clouds_message->point_clouds[i].point_size; k++, j++)
		{
			moving_objects_point_clouds[last_moving_objects_point_clouds].points[j].x = moving_objects_point_clouds_message->point_clouds[i].points[k].x;
			moving_objects_point_clouds[last_moving_objects_point_clouds].points[j].y = moving_objects_point_clouds_message->point_clouds[i].points[k].y;
			moving_objects_point_clouds[last_moving_objects_point_clouds].points[j].z = moving_objects_point_clouds_message->point_clouds[i].points[k].z;

			moving_objects_point_clouds[last_moving_objects_point_clouds].point_color[j].x = moving_objects_point_clouds_message->point_clouds[i].r;
			moving_objects_point_clouds[last_moving_objects_point_clouds].point_color[j].y = moving_objects_point_clouds_message->point_clouds[i].g;
			moving_objects_point_clouds[last_moving_objects_point_clouds].point_color[j].z = moving_objects_point_clouds_message->point_clouds[i].b;
		}
	}
}

static void
stereo_point_cloud_message_handler(carmen_stereo_point_cloud_message* stereo_point_cloud_message)
{
    if (!odometry_initialized)
        return;

    stereo_initialized = 1;

    last_stereo_point_cloud++;
    if (last_stereo_point_cloud >= stereo_point_cloud_size)
    {
        last_stereo_point_cloud = 0;
    }

    int num_points = stereo_point_cloud_message->num_points;

    //printf("Stereo point cloud: %d points.\n", num_points);

    if (num_points > stereo_point_cloud[last_stereo_point_cloud].num_points)
    {
        stereo_point_cloud[last_stereo_point_cloud].points = (carmen_vector_3D_t*) realloc(stereo_point_cloud[last_stereo_point_cloud].points, num_points * sizeof (carmen_vector_3D_t));
        stereo_point_cloud[last_stereo_point_cloud].point_color = (carmen_vector_3D_t*) realloc(stereo_point_cloud[last_stereo_point_cloud].point_color, num_points * sizeof (carmen_vector_3D_t));
    }
    stereo_point_cloud[last_stereo_point_cloud].num_points = num_points;
    stereo_point_cloud[last_stereo_point_cloud].car_position.x = car_fused_pose.position.x;
    stereo_point_cloud[last_stereo_point_cloud].car_position.y = car_fused_pose.position.y;
    stereo_point_cloud[last_stereo_point_cloud].car_position.z = car_fused_pose.position.z;
    stereo_point_cloud[last_stereo_point_cloud].timestamp = stereo_point_cloud_message->timestamp;

    rotation_matrix* r_matrix_car_to_global = create_rotation_matrix(car_fused_pose.orientation);

    int i;
    for (i = 0; i < num_points; i++)
    {
        carmen_vector_3D_t point_car_reference = change_point_reference(stereo_point_cloud_message->points[i], camera_pose);

        stereo_point_cloud[last_stereo_point_cloud].points[i] = get_point_position_global_reference(stereo_point_cloud[last_stereo_point_cloud].car_position, point_car_reference, r_matrix_car_to_global);
        stereo_point_cloud[last_stereo_point_cloud].point_color[i] = stereo_point_cloud_message->point_color[i];

    }

    destroy_rotation_matrix(r_matrix_car_to_global);
}

static void
gps_nmea_hdt_message_handler(carmen_gps_gphdt_message *gps_nmea_hdt_raw_message)
{
	gps_heading = gps_nmea_hdt_raw_message->heading;
	gps_heading_valid = gps_nmea_hdt_raw_message->valid;
}

static void
gps_xyz_message_handler(carmen_gps_xyz_message *gps_xyz_raw_message)
{
    static int k = 0;

    // GPS is not initializing properly on the first message, so it's waiting for 10 messages to initialize
    if (k < 10)
    {
        k++;
        return;
    }

    if (!gps_initialized)
    {
        gps_initial_pos.x = gps_xyz_raw_message->x;
        gps_initial_pos.y = gps_xyz_raw_message->y;
        gps_initial_pos.z = gps_xyz_raw_message->z;

        gps_initialized = 1;
    }

    carmen_vector_3D_t offset = get_position_offset();

    carmen_vector_3D_t new_pos;
    new_pos.x = gps_xyz_raw_message->x - offset.x;
    new_pos.y = gps_xyz_raw_message->y - offset.y;
    new_pos.z = gps_xyz_raw_message->z - offset.z;

    gps_trail[last_gps_trail] = new_pos;
    gps_nr[last_gps_trail] = gps_xyz_raw_message->nr;

    last_gps_trail++;

    if (last_gps_trail >= gps_size)
        last_gps_trail -= gps_size;

    if ((gps_xyz_raw_message->nr == 1) || (gps_xyz_raw_message->nr == 2)) // Trimble ou Reach1
        gps_fix_flag = gps_xyz_raw_message->gps_quality;
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static carmen_compact_map_t *compact_cost_map = NULL;
	static carmen_map_t static_cost_map;
	static carmen_map_t *cost_map;

	cost_map = &static_cost_map;

	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(cost_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(cost_map->complete_map, 0, cost_map->config.x_size * cost_map->config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t*) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(cost_map, compact_cost_map);
	}

	cost_map->config = message->config;

	if (!first_map_received)
    {
        first_map_received = 1;
        first_map_origin.x = message->config.x_origin;
        first_map_origin.y = message->config.y_origin;
        first_map_origin.z = 0.0;
    }
    else if ((first_map_origin.x == 0.0) && (first_map_origin.y == 0.0) && ((message->config.x_origin != 0.0) || (message->config.y_origin != 0.0)))
    {
        first_map_origin.x = message->config.x_origin;
        first_map_origin.y = message->config.y_origin;
        first_map_origin.z = 0.0;
    }

    if (draw_map_flag)// && (time_since_last_draw < 1.0 / 30.0))
    {
        add_map_message(m_drawer, static_cost_map);
    }
}

//static void
//mapper_map_message_handler(carmen_mapper_map_message *message)
//{
////    double time_since_last_draw = carmen_get_time() - lastDisplayTime;
//
//    if (!first_map_received)
//    {
//        first_map_received = 1;
//        first_map_origin.x = message->config.x_origin;
//        first_map_origin.y = message->config.y_origin;
//        first_map_origin.z = 0.0;
//    }
//    else if ((first_map_origin.x == 0.0) && (first_map_origin.y == 0.0) && ((message->config.x_origin != 0.0) || (message->config.y_origin != 0.0)))
//    {
//        first_map_origin.x = message->config.x_origin;
//        first_map_origin.y = message->config.y_origin;
//        first_map_origin.z = 0.0;
//    }
//
//    if (draw_map_flag)// && (time_since_last_draw < 1.0 / 30.0))
//    {
//        add_map_message(m_drawer, message);
//    }
//}

static void
plan_message_handler(carmen_navigator_ackerman_plan_message *message)
{
    add_trajectory_message(t_drawer1, message);
}

void
obstacle_avoider_message_handler(carmen_navigator_ackerman_plan_message *message)
{
    add_trajectory_message(t_drawer2, message);
}

void
motion_path_handler(carmen_navigator_ackerman_plan_message *message)
{
    add_trajectory_message(t_drawer3, message);
}

static void
navigator_goal_list_message_handler(carmen_behavior_selector_goal_list_message *goals)
{
    add_goal_list_message(t_drawer1, goals);
}


static void
plan_tree_handler(carmen_navigator_ackerman_plan_tree_message *msg)
{
	t_drawerTree.clear();
	t_drawerTree.resize(msg->num_path);

	for (int i = 0; i < msg->num_path; i++)
	{
		carmen_navigator_ackerman_plan_message tempMessage;
		tempMessage.path = msg->paths[i];
		tempMessage.path_length = msg->path_size[i];

		t_drawerTree[i] = create_trajectory_drawer(1.0, 1.0, 1.0);
		add_trajectory_message(t_drawerTree[i], &tempMessage);
	}

}


static void
carmen_download_map_handler(carmen_download_map_message *message)
{
    download_map_message.height = message->height;
    download_map_message.width = message->width;
    download_map_message.image_data = message->image_data;
    download_map_message.position = message->position;
    download_map_message.map_center = message->map_center;

    if (!first_download_map_have_been_aquired)
        first_download_map_have_been_aquired = 1;

    new_map_has_been_received = 1;
}

static void
carmen_localize_neural_base_message_handler(carmen_localize_neural_imagepos_message *message)
{
    if (!localize_imagepos_base_initialized)
    {
    	localize_imagepos_base_initialized = 1;
    	localize_imagepos_base_message = *message;
        localize_imagepos_base_message.image_data = (char*) malloc(message->size * sizeof(char));
    }
    localize_imagepos_base_message.pose = message->pose;
    memcpy(localize_imagepos_base_message.image_data, message->image_data, message->size * sizeof(char));
}

static void
carmen_localize_neural_curr_message_handler(carmen_localize_neural_imagepos_message *message)
{
    if (!localize_imagepos_curr_initialized)
    {
    	localize_imagepos_curr_initialized = 1;
    	localize_imagepos_curr_message = *message;
        localize_imagepos_curr_message.image_data = (char*) malloc(message->size * sizeof(char));
    }
    localize_imagepos_curr_message.pose = message->pose;
    memcpy(localize_imagepos_curr_message.image_data, message->image_data, message->size * sizeof(char));
}


static void
carmen_localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	gps_position_at_turn_on = {initialize_msg->mean->x, initialize_msg->mean->y, 0.0};
    odometry_initialized = 1;
}


#ifdef TEST_LANE_ANALYSIS
static void lane_analysis_handler(carmen_elas_lane_analysis_message * message) {
	carmen_vector_3D_t position_offset = get_position_offset();
	position_offset.z = 0;
	add_to_trail(message, lane_drawer, position_offset);
}
#endif

static void
init_moving_objects_point_clouds(void)
{
	moving_objects_point_clouds_initialized = 0; // Only considered initialized when first message is received

	moving_objects_point_clouds = (point_cloud*) malloc(moving_objects_point_clouds_size * sizeof (point_cloud));

    int i;
    for (i = 0; i < moving_objects_point_clouds_size; i++)
    {
    	moving_objects_point_clouds[i].points = NULL;
    	moving_objects_point_clouds[i].point_color = NULL;
    	moving_objects_point_clouds[i].num_points = 0;
    	moving_objects_point_clouds[i].timestamp = carmen_get_time();
    }

    last_moving_objects_point_clouds = 0;
}

static void
init_velodyne(void)
{
    velodyne_initialized = 0; // Only considered initialized when first message is received

    velodyne_points = (point_cloud*) malloc(velodyne_size * sizeof (point_cloud));

    int i;
    for (i = 0; i < velodyne_size; i++)
    {
        velodyne_points[i].points = NULL;
        velodyne_points[i].point_color = NULL;
        velodyne_points[i].num_points = 0;
        velodyne_points[i].timestamp = carmen_get_time();
    }

    last_velodyne_position = 0;
}

static void
init_laser(void)
{
    laser_initialized = 0; // Only considered initialized when first message is received

    laser_points = (point_cloud*) malloc(laser_size * sizeof (point_cloud));
	
	front_bullbar_left_corner_laser_points = (point_cloud*) malloc(laser_size * sizeof (point_cloud));
	front_bullbar_right_corner_laser_points = (point_cloud*) malloc(laser_size * sizeof (point_cloud));
	rear_bullbar_left_corner_laser_points = (point_cloud*) malloc(laser_size * sizeof (point_cloud));
	rear_bullbar_right_corner_laser_points = (point_cloud*) malloc(laser_size * sizeof (point_cloud));

    int i;
    for (i = 0; i < laser_size; i++)
    {
        laser_points[i].points = NULL;
        laser_points[i].point_color = NULL;
        laser_points[i].num_points = 0;
        laser_points[i].timestamp = carmen_get_time();

		front_bullbar_left_corner_laser_points[i].points = NULL;
        front_bullbar_left_corner_laser_points[i].point_color = NULL;
        front_bullbar_left_corner_laser_points[i].num_points = 0;
        front_bullbar_left_corner_laser_points[i].timestamp = carmen_get_time();
		
		front_bullbar_right_corner_laser_points[i].points = NULL;
        front_bullbar_right_corner_laser_points[i].point_color = NULL;
        front_bullbar_right_corner_laser_points[i].num_points = 0;
        front_bullbar_right_corner_laser_points[i].timestamp = carmen_get_time();
		
		rear_bullbar_left_corner_laser_points[i].points = NULL;
        rear_bullbar_left_corner_laser_points[i].point_color = NULL;
        rear_bullbar_left_corner_laser_points[i].num_points = 0;
        rear_bullbar_left_corner_laser_points[i].timestamp = carmen_get_time();
		
		rear_bullbar_right_corner_laser_points[i].points = NULL;
        rear_bullbar_right_corner_laser_points[i].point_color = NULL;
        rear_bullbar_right_corner_laser_points[i].num_points = 0;
        rear_bullbar_right_corner_laser_points[i].timestamp = carmen_get_time();
    }

    last_laser_position = 0;
}


static void
init_ldmrs(void)
{
    ldmrs_initialized = 0; // Only considered initialized when first message is received

    ldmrs_points = (point_cloud*) malloc(ldmrs_size * sizeof (point_cloud));

	front_bullbar_middle_laser_points = (point_cloud*) malloc(ldmrs_size * sizeof (point_cloud));

    int i;
    for (i = 0; i < ldmrs_size; i++)
    {
        ldmrs_points[i].points = NULL;
        ldmrs_points[i].point_color = NULL;
        ldmrs_points[i].num_points = 0;
        ldmrs_points[i].timestamp = carmen_get_time();

		front_bullbar_middle_laser_points[i].points = NULL;
        front_bullbar_middle_laser_points[i].point_color = NULL;
        front_bullbar_middle_laser_points[i].num_points = 0;
        front_bullbar_middle_laser_points[i].timestamp = carmen_get_time();
    }

    last_ldmrs_position = 0;
}

static void
init_gps(void)
{
    gps_initialized = 0; // Only considered initialized when first message is received

    gps_trail = (carmen_vector_3D_t *) malloc(gps_size * sizeof (carmen_vector_3D_t));
    gps_nr = (int *) malloc(gps_size * sizeof(int));

    carmen_vector_3D_t init_pos;

    init_pos.x = 0;
    init_pos.y = 0;
    init_pos.z = 0;

    int i;
    for (i = 0; i < gps_size; i++)
    {
        gps_trail[i] = init_pos;
        gps_nr[i] = 0;
    }

    last_gps_trail = 0;
}

static void
init_xsens(void)
{
    xsens_orientation.roll = 0.0;
    xsens_orientation.pitch = 0.0;
    xsens_orientation.yaw = 0.0;

    xsens_yaw_bias = 0.0;
}

static void
init_xsens_xyz(void)
{
    xsens_xyz_initialized = 0; // Only considered initialized when first message is received

    xsens_xyz_trail = (carmen_vector_3D_t *) malloc(gps_size * sizeof (carmen_vector_3D_t));

    carmen_vector_3D_t init_pos;

    init_pos.x = 0;
    init_pos.y = 0;
    init_pos.z = 0;

    gps_fix_flag = 0;

    int i;
    for (i = 0; i < gps_size; i++)
        xsens_xyz_trail[i] = init_pos;

    next_xsens_xyz_trail = 0;
}

static void
init_stereo_point_cloud(void)
{
    stereo_initialized = 0; // Only considered initialized when first message is received

    stereo_point_cloud = (point_cloud*) malloc(stereo_point_cloud_size * sizeof (point_cloud));

    int i;
    for (i = 0; i < stereo_point_cloud_size; i++)
    {
        stereo_point_cloud[i].points = NULL;
        stereo_point_cloud[i].point_color = NULL;
        stereo_point_cloud[i].num_points = 0;
        stereo_point_cloud[i].timestamp = carmen_get_time();
    }

    last_stereo_point_cloud = 0;
}

static void
init_odometry(void)
{
    odometry_initialized = 0; // Only considered initialized when first message is received

    odometry_trail = (carmen_vector_3D_t*) malloc(odometry_size * sizeof (carmen_vector_3D_t));

    carmen_vector_3D_t init_pos;

    init_pos.x = 0;
    init_pos.y = 0;
    init_pos.z = 0;

    int i;
    for (i = 0; i < odometry_size; i++)
    {
        odometry_trail[i] = init_pos;
    }

    last_odometry_trail = 0;
}

static void
init_localize_ackerman_trail(void)
{
    localize_ackerman_trail = (carmen_vector_3D_t*) malloc(localize_ackerman_size * sizeof (carmen_vector_3D_t));

    carmen_vector_3D_t init_pos;

    init_pos.x = 0;
    init_pos.y = 0;
    init_pos.z = 0;

    int i;
    for (i = 0; i < localize_ackerman_size; i++)
    {
        localize_ackerman_trail[i] = init_pos;
    }

    last_localize_ackerman_trail = 0;
}

static void
init_particle_trail(void)
{
    fused_odometry_particles_pos = NULL;
    fused_odometry_particles_weight = NULL;
    num_fused_odometry_particles = 0;

    localizer_prediction_particles_pos = NULL;
    localizer_prediction_particles_weight = NULL;
    num_localizer_prediction_particles = 0;

    localizer_correction_particles_pos = NULL;
    localizer_correction_particles_weight = NULL;
    num_localizer_correction_particles = 0;
}

static void
init_flags(void)
{
    draw_particles_flag = 0;
    draw_points_flag = 0;
    draw_velodyne_flag = 2;
    draw_stereo_cloud_flag = 0;
    draw_car_flag = 1;
    draw_rays_flag = 0;
    draw_map_image_flag = 0;
    draw_localize_image_flag = 0;
    weight_type_flag = 2;
    draw_gps_flag = 0;
    draw_odometry_flag = 0;
    draw_xsens_gps_flag = 0;
    follow_car_flag = 1;
    draw_map_flag = 0;
    zero_z_flag = 1;
    draw_trajectory_flag1 = 1;
    draw_trajectory_flag2 = 1;
    draw_trajectory_flag3 = 1;
    draw_xsens_orientation_flag = 0;
    draw_localize_ackerman_flag = 1;
    draw_annotation_flag = 0;
    draw_moving_objects_flag = 0;
    draw_gps_axis_flag = 1;
#ifdef TEST_LANE_ANALYSIS
    draw_lane_analysis_flag = 1;
#endif
}

////Desabilitar coisas na tela do viewer para filmar legal
//static void
//init_flags(void)
//{
//    draw_particles_flag = 0;
//    draw_points_flag = 0;
//    draw_velodyne_flag = 0;//2;
//    draw_stereo_cloud_flag = 0;
//    draw_car_flag = 1;
//    draw_rays_flag = 0;
//    draw_map_image_flag = 0;
//    weight_type_flag = 2;
//    draw_gps_flag = 0;
//    draw_odometry_flag = 0;
//    draw_xsens_gps_flag = 0;
//    follow_car_flag = 1;
//    draw_map_flag = 1;
//    zero_z_flag = 1;
//    draw_trajectory_flag1 = 1;
//    draw_trajectory_flag2 = 1;
//    draw_trajectory_flag3 = 1;
//    draw_xsens_orientation_flag = 0;
//    draw_localize_ackerman_flag = 0;
//    draw_annotation_flag = 0;
//    draw_moving_objects_flag = 0;
//    draw_gps_axis_flag = 0;
//#ifdef TEST_LANE_ANALYSIS
//    draw_lane_analysis_flag = 1;
//#endif
//}


void
init_drawers(int argc, char** argv, int bumblebee_basic_width, int bumblebee_basic_height)
{
    carmen_pose_3D_t stereo_velodyne_pose = get_stereo_velodyne_pose_3D(argc, argv, camera);
    car_drawer = createCarDrawer(argc, argv);
    laser_drawer = create_point_cloud_drawer(laser_size);
    ldmrs_drawer = create_point_cloud_drawer(ldmrs_size);
    velodyne_drawer = create_point_cloud_drawer(velodyne_size);
    v_360_drawer = create_velodyne_360_drawer(velodyne_pose, sensor_board_1_pose);

    // *********************************************************************
    // TODO: pegar a pose do variable_velodyne do param_daemon e usar na funcao abaixo. Do jeito que esta sempre que o stereo_velodyne eh chamado da seg fault
    // *********************************************************************
    var_v_drawer = create_variable_velodyne_drawer(stereo_velodyne_flipped, stereo_velodyne_pose, stereo_velodyne_num_points_cloud, stereo_velodyne_vertical_resolution, camera, stereo_velodyne_vertical_roi_ini, stereo_velodyne_vertical_roi_end, stereo_velodyne_horizontal_roi_ini, stereo_velodyne_horizontal_roi_end, bumblebee_basic_width, bumblebee_basic_height);
    // *********************************************************************

    i_drawer = create_interface_drawer();
    m_drawer = create_map_drawer();
    t_drawer1 = create_trajectory_drawer(0.0, 0.0, 1.0);
    t_drawer2 = create_trajectory_drawer(1.0, 0.0, 0.0);
    t_drawer3 = create_trajectory_drawer(0.0, 1.0, 0.0);
    v_int_drawer = create_velodyne_intensity_drawer(velodyne_pose, sensor_board_1_pose);
    annotation_drawer = createAnnotationDrawer(argc, argv);
#ifdef TEST_LANE_ANALYSIS
    lane_drawer = create_lane_analysis_drawer();
#endif
}

void
destroy_drawers()
{
    destroyCarDrawer(car_drawer);
    destroy_point_cloud_drawer(laser_drawer);
    destroy_point_cloud_drawer(ldmrs_drawer);
    destroy_point_cloud_drawer(velodyne_drawer);
    destroy_velodyne_360_drawer(v_360_drawer);
    //destroy_variable_velodyne_drawer(var_v_drawer);
    destroy_interface_drawer(i_drawer);
    destroy_map_drawer(m_drawer);
    destroy_trajectory_drawer(t_drawer1);
    destroy_trajectory_drawer(t_drawer2);
    destroy_trajectory_drawer(t_drawer3);
    destroy_velodyne_intensity_drawer(v_int_drawer);
    destroyAnnotationDrawer(annotation_drawer);
#ifdef TEST_LANE_ANALYSIS
    destroy_lane_analysis_drawer(lane_drawer);
#endif
}

void
init_stuff(int argc, char** argv)
{
    int num_items;
    int horizontal_resolution;
    char camera_string[256];
    char stereo_string[256];
    char stereo_velodyne_string[256];
    double b_red;
    double b_green;
    double b_blue;

    int bumblebee_basic_width, bumblebee_basic_height;

    if (camera != 0)
    {
        sprintf(camera_string, "%s%d", "bumblebee_basic", camera);
        sprintf(stereo_string, "%s%d", "stereo", camera);
        sprintf(stereo_velodyne_string, "%s%d", "stereo_velodyne", camera);

        carmen_param_t param_list[] = {
            {(char*) "viewer_3D", (char*) "laser_size", CARMEN_PARAM_INT, &laser_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "ldmrs_size", CARMEN_PARAM_INT, &ldmrs_size, 0, NULL},
			{(char*) "viewer_3D", (char*) "ldmrs_min_velocity", CARMEN_PARAM_DOUBLE, &ldmrs_min_velocity, 0, NULL},
            {(char*) "viewer_3D", (char*) "velodyne_size", CARMEN_PARAM_INT, &velodyne_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "odometry_size", CARMEN_PARAM_INT, &odometry_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "gps_size", CARMEN_PARAM_INT, &gps_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "stereo_point_cloud_size", CARMEN_PARAM_INT, &stereo_point_cloud_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "localize_ackerman_size", CARMEN_PARAM_INT, &localize_ackerman_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_red", CARMEN_PARAM_DOUBLE, &b_red, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_green", CARMEN_PARAM_DOUBLE, &b_green, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_blue", CARMEN_PARAM_DOUBLE, &b_blue, 0, NULL},
            {(char*) "viewer_3D", (char*) "point_size", CARMEN_PARAM_INT, &point_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "camera_square_size", CARMEN_PARAM_INT, &camera_square_size, 0, NULL},

            {(char*) "sensor_board_1", (char*) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x), 0, NULL},
            {(char*) "sensor_board_1", (char*) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y), 0, NULL},
            {(char*) "sensor_board_1", (char*) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z), 0, NULL},
            {(char*) "sensor_board_1", (char*) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll), 0, NULL},
            {(char*) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch), 0, NULL},
            {(char*) "sensor_board_1", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw), 0, NULL},
            {(char*) "sensor_board_1", (char*) "laser_id", CARMEN_PARAM_INT, &(sensor_board_1_laser_id), 0, NULL},
		
            {(char*) "front_bullbar", (char*) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.x), 0, NULL},
            {(char*) "front_bullbar", (char*) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.y), 10, NULL},
            {(char*) "front_bullbar", (char*) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.z), 0, NULL},
            {(char*) "front_bullbar", (char*) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.roll), 0, NULL},
            {(char*) "front_bullbar", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.pitch), 0, NULL},
            {(char*) "front_bullbar", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.yaw), 0, NULL},
            
            {(char*) "front_bullbar_middle", (char*) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.position.x), 0, NULL},
            {(char*) "front_bullbar_middle", (char*) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.position.y), 0, NULL},
            {(char*) "front_bullbar_middle", (char*) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.position.z), 0, NULL},
            {(char*) "front_bullbar_middle", (char*) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.orientation.roll), 0, NULL},
            {(char*) "front_bullbar_middle", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.orientation.pitch), 0, NULL},
            {(char*) "front_bullbar_middle", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_middle_pose.orientation.yaw), 0, NULL},

            {(char*) "front_bullbar_left_corner", (char*) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.position.x), -10, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.position.y), 0, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.position.z), 0, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.orientation.roll), 0, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.orientation.pitch), 0, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_left_corner_pose.orientation.yaw), 0, NULL},
            {(char*) "front_bullbar_left_corner", (char*) "laser_id", CARMEN_PARAM_INT, &(front_bullbar_left_corner_laser_id), 0, NULL},
            
            {(char*) "front_bullbar_right_corner", (char*) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.position.x), 10, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.position.y), 0, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.position.z), 0, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.orientation.roll), 0, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.orientation.pitch), 0, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_right_corner_pose.orientation.yaw), 0, NULL},
            {(char*) "front_bullbar_right_corner", (char*) "laser_id", CARMEN_PARAM_INT, &(front_bullbar_right_corner_laser_id), 0, NULL},
            
            {(char*) "rear_bullbar", (char*) "x", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.x), 0, NULL},
            {(char*) "rear_bullbar", (char*) "y", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.y), -10, NULL},
            {(char*) "rear_bullbar", (char*) "z", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.z), 0, NULL},
            {(char*) "rear_bullbar", (char*) "roll", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.roll), 0, NULL},
            {(char*) "rear_bullbar", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.pitch), 0, NULL},
            {(char*) "rear_bullbar", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.yaw), 0, NULL},
            
            {(char*) "rear_bullbar_left_corner", (char*) "x", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.position.x), -10, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "y", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.position.y), 0, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "z", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.position.z), 0, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "roll", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.orientation.roll), 0, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.orientation.pitch), 0, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(rear_bullbar_left_corner_pose.orientation.yaw), 0, NULL},
            {(char*) "rear_bullbar_left_corner", (char*) "laser_id", CARMEN_PARAM_INT, &(rear_bullbar_left_corner_laser_id), 0, NULL},
            
            {(char*) "rear_bullbar_right_corner", (char*) "x", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.position.x), 10, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "y", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.position.y), 0, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "z", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.position.z), 0, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "roll", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.orientation.roll), 0, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.orientation.pitch), 0, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(rear_bullbar_right_corner_pose.orientation.yaw), 0, NULL},
            {(char*) "rear_bullbar_right_corner", (char*) "laser_id", CARMEN_PARAM_INT, &(rear_bullbar_right_corner_laser_id), 0, NULL},

            {(char*) "xsens", (char*) "x", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.x), 0, NULL},
            {(char*) "xsens", (char*) "y", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.y), 0, NULL},
            {(char*) "xsens", (char*) "z", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.z), 0, NULL},
            {(char*) "xsens", (char*) "roll", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.roll), 0, NULL},
            {(char*) "xsens", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.pitch), 0, NULL},
            {(char*) "xsens", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.yaw), 0, NULL},

            {(char*) "laser", (char*) "num_laser_devices", CARMEN_PARAM_INT, &num_laser_devices, 0, NULL},

            {(char*) "laser", (char*) "x", CARMEN_PARAM_DOUBLE, &(laser_pose.position.x), 0, NULL},
            {(char*) "laser", (char*) "y", CARMEN_PARAM_DOUBLE, &(laser_pose.position.y), 0, NULL},
            {(char*) "laser", (char*) "z", CARMEN_PARAM_DOUBLE, &(laser_pose.position.z), 0, NULL},
            {(char*) "laser", (char*) "roll", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.roll), 0, NULL},
            {(char*) "laser", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.pitch), 0, NULL},
            {(char*) "laser", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.yaw), 0, NULL},

            {(char*) "camera", (char*) "x", CARMEN_PARAM_DOUBLE, &(camera_pose.position.x), 0, NULL},
            {(char*) "camera", (char*) "y", CARMEN_PARAM_DOUBLE, &(camera_pose.position.y), 0, NULL},
            {(char*) "camera", (char*) "z", CARMEN_PARAM_DOUBLE, &(camera_pose.position.z), 0, NULL},
            {(char*) "camera", (char*) "roll", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.roll), 0, NULL},
            {(char*) "camera", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.pitch), 0, NULL},
            {(char*) "camera", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.yaw), 0, NULL},
            {camera_string, (char*) "width", CARMEN_PARAM_INT, &(bumblebee_basic_width), 0, NULL},
            {camera_string, (char*) "height", CARMEN_PARAM_INT, &(bumblebee_basic_height), 0, NULL},

            {(char*) "car", (char*) "x", CARMEN_PARAM_DOUBLE, &(car_pose.position.x), 0, NULL},
            {(char*) "car", (char*) "y", CARMEN_PARAM_DOUBLE, &(car_pose.position.y), 0, NULL},
            {(char*) "car", (char*) "z", CARMEN_PARAM_DOUBLE, &(car_pose.position.z), 0, NULL},
            {(char*) "car", (char*) "roll", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.roll), 0, NULL},
            {(char*) "car", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.pitch), 0, NULL},
            {(char*) "car", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.yaw), 0, NULL},

            {(char*) "velodyne", (char*) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
            {(char*) "velodyne", (char*) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
            {(char*) "velodyne", (char*) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
            {(char*) "velodyne", (char*) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
            {(char*) "velodyne", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
            {(char*) "velodyne", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char *) "laser_ldmrs",  (char *) "x", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.x), 0, NULL},
			{(char *) "laser_ldmrs",  (char *) "y", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.y), 0, NULL},
			{(char *) "laser_ldmrs",  (char *) "z", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.z), 0, NULL},
			{(char *) "laser_ldmrs",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.roll), 0, NULL},
			{(char *) "laser_ldmrs",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.pitch), 0, NULL},
			{(char *) "laser_ldmrs",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.yaw), 0, NULL},

			{(char*) stereo_velodyne_string, (char*) "vertical_resolution", CARMEN_PARAM_INT, &stereo_velodyne_vertical_resolution, 0, NULL},
            {(char*) stereo_velodyne_string, (char*) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
            {(char*) stereo_velodyne_string, (char*) "flipped", CARMEN_PARAM_ONOFF, &stereo_velodyne_flipped, 0, NULL},
            {(char*) stereo_velodyne_string, (char*) "num_points_cloud", CARMEN_PARAM_INT, &stereo_velodyne_num_points_cloud, 0, NULL},
            {stereo_velodyne_string, (char*) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL},
            {stereo_velodyne_string, (char*) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL},
            {stereo_velodyne_string, (char*) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL},
            {stereo_velodyne_string, (char*) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL},
            {(char*) "velodyne", (char*) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &time_spent_by_each_scan, 0, NULL},
            {(char*) "robot", (char*) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 0, NULL},
        };

        num_items = sizeof (param_list) / sizeof (param_list[0]);
        carmen_param_install_params(argc, argv, param_list, num_items);

        if (stereo_velodyne_vertical_resolution > (stereo_velodyne_vertical_roi_end - stereo_velodyne_vertical_roi_ini))
        {
            carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
        }

        if (stereo_velodyne_flipped)
        {
            stereo_velodyne_vertical_resolution = horizontal_resolution;
        }
    }
    else
    {
        carmen_param_t param_list[] = {
            {(char*) "viewer_3D", (char*) "laser_size", CARMEN_PARAM_INT, &laser_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "velodyne_size", CARMEN_PARAM_INT, &velodyne_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "odometry_size", CARMEN_PARAM_INT, &odometry_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "gps_size", CARMEN_PARAM_INT, &gps_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "stereo_point_cloud_size", CARMEN_PARAM_INT, &stereo_point_cloud_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "localize_ackerman_size", CARMEN_PARAM_INT, &localize_ackerman_size, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_red", CARMEN_PARAM_DOUBLE, &b_red, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_green", CARMEN_PARAM_DOUBLE, &b_green, 0, NULL},
            {(char*) "viewer_3D", (char*) "background_blue", CARMEN_PARAM_DOUBLE, &b_blue, 0, NULL},

            {(char*) "sensor_board_1", (char*) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x), 0, NULL},
            {(char*) "sensor_board_1", (char*) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y), 0, NULL},
            {(char*) "sensor_board_1", (char*) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z), 0, NULL},
            {(char*) "sensor_board_1", (char*) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll), 0, NULL},
            {(char*) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch), 0, NULL},
            {(char*) "sensor_board_1", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw), 0, NULL},

            {(char*) "xsens", (char*) "x", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.x), 0, NULL},
            {(char*) "xsens", (char*) "y", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.y), 0, NULL},
            {(char*) "xsens", (char*) "z", CARMEN_PARAM_DOUBLE, &(xsens_pose.position.z), 0, NULL},
            {(char*) "xsens", (char*) "roll", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.roll), 0, NULL},
            {(char*) "xsens", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.pitch), 0, NULL},
            {(char*) "xsens", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(xsens_pose.orientation.yaw), 0, NULL},

            {(char*) "laser", (char*) "x", CARMEN_PARAM_DOUBLE, &(laser_pose.position.x), 0, NULL},
            {(char*) "laser", (char*) "y", CARMEN_PARAM_DOUBLE, &(laser_pose.position.y), 0, NULL},
            {(char*) "laser", (char*) "z", CARMEN_PARAM_DOUBLE, &(laser_pose.position.z), 0, NULL},
            {(char*) "laser", (char*) "roll", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.roll), 0, NULL},
            {(char*) "laser", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.pitch), 0, NULL},
            {(char*) "laser", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(laser_pose.orientation.yaw), 0, NULL},

            {(char*) "camera", (char*) "x", CARMEN_PARAM_DOUBLE, &(camera_pose.position.x), 0, NULL},
            {(char*) "camera", (char*) "y", CARMEN_PARAM_DOUBLE, &(camera_pose.position.y), 0, NULL},
            {(char*) "camera", (char*) "z", CARMEN_PARAM_DOUBLE, &(camera_pose.position.z), 0, NULL},
            {(char*) "camera", (char*) "roll", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.roll), 0, NULL},
            {(char*) "camera", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.pitch), 0, NULL},
            {(char*) "camera", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.yaw), 0, NULL},

            {(char*) "car", (char*) "x", CARMEN_PARAM_DOUBLE, &(car_pose.position.x), 0, NULL},
            {(char*) "car", (char*) "y", CARMEN_PARAM_DOUBLE, &(car_pose.position.y), 0, NULL},
            {(char*) "car", (char*) "z", CARMEN_PARAM_DOUBLE, &(car_pose.position.z), 0, NULL},
            {(char*) "car", (char*) "roll", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.roll), 0, NULL},
            {(char*) "car", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.pitch), 0, NULL},
            {(char*) "car", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.yaw), 0, NULL},

            {(char*) "velodyne", (char*) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
            {(char*) "velodyne", (char*) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
            {(char*) "velodyne", (char*) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
            {(char*) "velodyne", (char*) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
            {(char*) "velodyne", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
            {(char*) "velodyne", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
        };

        num_items = sizeof (param_list) / sizeof (param_list[0]);
        carmen_param_install_params(argc, argv, param_list, num_items);
    }

    init_moving_objects_point_clouds();
    init_laser();
    init_ldmrs();
    init_velodyne();
    init_gps();
    init_xsens_xyz();
    init_xsens();
    init_stereo_point_cloud();
    init_odometry();
    init_localize_ackerman_trail();
    init_particle_trail();
    init_flags();
    init_drawers(argc, argv, bumblebee_basic_width, bumblebee_basic_height);

    set_background_color(b_red, b_green, b_blue);
}


void
destroy_stuff()
{
    int i;

    for (i = 0; i < stereo_point_cloud_size; i++)
    {
        free(stereo_point_cloud[i].points);
        free(stereo_point_cloud[i].point_color);
    }
    free(stereo_point_cloud);

    for (i = 0; i < laser_size; i++)
    {
        free(laser_points[i].points);
        free(laser_points[i].point_color);
		
		free(front_bullbar_left_corner_laser_points[i].points);
        free(front_bullbar_left_corner_laser_points[i].point_color);
		free(front_bullbar_right_corner_laser_points[i].points);
        free(front_bullbar_right_corner_laser_points[i].point_color);
		
		free(rear_bullbar_left_corner_laser_points[i].points);
        free(rear_bullbar_left_corner_laser_points[i].point_color);
		free(rear_bullbar_right_corner_laser_points[i].points);
        free(rear_bullbar_right_corner_laser_points[i].point_color);
    }
    free(laser_points);
	free(front_bullbar_left_corner_laser_points);
	free(front_bullbar_right_corner_laser_points);
	free(rear_bullbar_left_corner_laser_points);
	free(rear_bullbar_right_corner_laser_points);
	
    for (i = 0; i < velodyne_size; i++)
    {
        free(velodyne_points[i].points);
        free(velodyne_points[i].point_color);
    }
    free(velodyne_points);

	for (i = 0; i < moving_objects_point_clouds_size; i++)
	{
		free(moving_objects_point_clouds[i].points);
		free(moving_objects_point_clouds[i].point_color);
	}
	free(moving_objects_point_clouds);

    free(odometry_trail);
    free(localize_ackerman_trail);
    free(gps_trail);
    free(gps_nr);
    free(xsens_xyz_trail);

    free(fused_odometry_particles_pos);
    free(fused_odometry_particles_weight);
    free(localizer_prediction_particles_pos);
    free(localizer_prediction_particles_weight);
    free(localizer_correction_particles_pos);
    free(localizer_correction_particles_weight);

    free(moving_objects_tracking);
    free(ldmrs_objects_tracking);

    destroy_drawers();
}


void
draw_loop(window *w)
{
    glPointSize(point_size);
    lastDisplayTime = carmen_get_time();
    double fps = 30.0;

    while (showWindow(w))
    {
        if (!processWindow(w, mouseFunc, keyPress, keyRelease))
        {
            break;
        }

        double sleepTime = 1.0 / fps - (carmen_get_time() - lastDisplayTime);
        if (sleepTime < 0.0)
        {
            sleepTime = 0.01;
        }
        carmen_ipc_sleep(sleepTime);
        lastDisplayTime = carmen_get_time();

        draw_variable_velodyne(var_v_drawer);

        if (follow_car_flag)
        {
            set_camera_offset(car_fused_pose.position);

//            set_camera_offset({0,0,0});
//            set_camera ({{-5,0,3}, {0,0,0}});
        }

        reset_camera();

        if (draw_annotation_flag)
        {
            glPointSize(5);
            glColor3f(1.0f, 1.0f, 1.0f);
			
            glPushMatrix();
			
            // printf("Annotation %lf %lf %lf\n", annotation_p1.x, annotation_p1.y, annotation_p1.z);
            glTranslatef(annotation_point.x, annotation_point.y, annotation_point.z);
            glutSolidSphere(0.5, 8, 8);
            glPopMatrix();
            glPointSize(point_size);
        //}
			glPointSize(1);
			glColor3f(1.0f, 0.0f, 0.0f);

			glPushMatrix();
			
			carmen_vector_3D_t position = get_world_position(FRONT_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, front_bullbar_left_corner_hierarchy);
			
			glTranslatef(position.x, position.y, position.z);
			glutSolidSphere(0.1, 8, 8);
			glPopMatrix();
			glPointSize(point_size);
			
			glPointSize(1);
			glColor3f(0.0f, 1.0f, 0.0f);

			glPushMatrix();
			position = get_world_position(FRONT_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, front_bullbar_right_corner_hierarchy);
			glTranslatef(position.x, position.y, position.z);
			glutSolidSphere(0.1, 8, 8);
			glPopMatrix();
			glPointSize(point_size);
			
			glPointSize(1);
			glColor3f(0.0f, 0.0f, 1.0f);
			rear_bullbar_pose.orientation.yaw += 0.2;
			glPushMatrix();
			position = get_world_position(REAR_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, rear_bullbar_left_corner_hierarchy);
			glTranslatef(position.x, position.y, position.z);
			glutSolidSphere(0.1, 8, 8);
			glPopMatrix();
			glPointSize(point_size);
			
			glPointSize(1);
			glColor3f(1.0f, 1.0f, 1.0f);

			glPushMatrix();
			position = get_world_position(REAR_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, rear_bullbar_right_corner_hierarchy);
			glTranslatef(position.x, position.y, position.z);
			glutSolidSphere(0.1, 8, 8);
			glPopMatrix();
			glPointSize(point_size);
		}

        // printf("annotations.size(): %ld\n", annotations.size());

        if (annotations.size() > 0)
            draw_annotations(annotations, car_pose.position, get_position_offset());

        if (draw_xsens_orientation_flag)
        {
            glColor3f(0.4, 1.0, 0.4);
            draw_xsens_orientation(xsens_orientation, xsens_yaw_bias, xsens_pose, sensor_board_1_pose, car_fused_pose);
            glColor3f(1.0, 0.4, 0.4);
            draw_xsens_orientation(xsens_orientation, 0.0, xsens_pose, sensor_board_1_pose, car_fused_pose);
        }
        //draw_orientation_instruments(car_fused_pose.orientation, 1.0, 1.0, 0.0);
        //draw_orientation_instruments(xsens_orientation, xsens_yaw_bias, 1.0, 0.5, 0.0);

        if (draw_gps_axis_flag)
        {
			if (gps_fix_flag == 4)
				glColor3f(0.0, 1.0, 0.0);
			else if (gps_fix_flag == 5)
				glColor3f(0.0, 0.0, 1.0);
			else
				glColor3f(1.0, 0.5, 0.0);

            draw_gps_orientation(gps_heading, gps_heading_valid, xsens_orientation, xsens_pose, sensor_board_1_pose, car_fused_pose);
        }

        if (draw_car_flag)
        {
            draw_car_at_pose(car_drawer, car_fused_pose);
        }

        if (draw_stereo_cloud_flag)
        {
//            draw_stereo_point_cloud(stereo_point_cloud, stereo_point_cloud_size);
            draw_variable_velodyne(var_v_drawer);
        }

        if (draw_points_flag == 1)
        {
            draw_laser_points(laser_points, laser_size);
			
			draw_laser_points(front_bullbar_left_corner_laser_points, laser_size);
			draw_laser_points(front_bullbar_right_corner_laser_points, laser_size);
			draw_laser_points(rear_bullbar_left_corner_laser_points, laser_size);
			draw_laser_points(rear_bullbar_right_corner_laser_points, laser_size);
			
            draw_laser_points(ldmrs_points, ldmrs_size);
			draw_laser_points(front_bullbar_middle_laser_points, ldmrs_size);

        }
        else if (draw_points_flag == 2)
        {
            draw_point_cloud(laser_drawer);
            draw_point_cloud(ldmrs_drawer);
        }

        if (draw_velodyne_flag == 1)
        {
            if (draw_annotation_flag)
                glPointSize(5);
            draw_velodyne_points(velodyne_points, velodyne_size);
            glPointSize(point_size);
        }
        else if (draw_velodyne_flag == 2)
        {
            //draw_velodyne_points(&(velodyne_points[last_velodyne_position]), 1);
            if (draw_annotation_flag)
                glPointSize(5);
            draw_point_cloud(velodyne_drawer);
            glPointSize(point_size);
        }
        else if (draw_velodyne_flag == 3)
        {
            draw_velodyne_360(v_360_drawer, car_fused_pose);
        }
        else if (draw_velodyne_flag == 4)
        {
            draw_variable_velodyne(var_v_drawer);
        }
        else if (draw_velodyne_flag == 5)
        {
            draw_velodyne_intensity(v_int_drawer);
        }

        if (draw_rays_flag)
        {
//            carmen_vector_3D_t offset = get_position_offset();
//            offset.z += sensor_board_1_pose.position.z;

        	//draw_moving_objects_point_clouds(moving_objects_point_clouds, moving_objects_point_clouds_size, offset);
        	if (draw_velodyne_flag > 0)
        		draw_laser_rays(velodyne_points[last_velodyne_position], get_world_position(VELODYNE_HIERARCHY_SIZE,velodyne_hierarchy));
			if (draw_points_flag > 0)
				draw_laser_rays(front_bullbar_middle_laser_points[last_laser_position], get_world_position(FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE, front_bullbar_middle_hierarchy));

//			draw_laser_rays(front_bullbar_left_corner_laser_points[last_laser_position], get_world_position(FRONT_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, front_bullbar_left_corner_hierarchy));
//			draw_laser_rays(front_bullbar_right_corner_laser_points[last_laser_position], get_world_position(FRONT_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, front_bullbar_right_corner_hierarchy));
//			draw_laser_rays(rear_bullbar_left_corner_laser_points[last_laser_position], get_world_position(REAR_BULLBAR_LEFT_CORNER_HIERARCHY_SIZE, rear_bullbar_left_corner_hierarchy));
//			draw_laser_rays(rear_bullbar_right_corner_laser_points[last_laser_position], get_world_position(REAR_BULLBAR_RIGHT_CORNER_HIERARCHY_SIZE, rear_bullbar_right_corner_hierarchy));
//			draw_laser_rays(front_bullbar_middle_laser_points[last_laser_position], get_world_position(FRONT_BULLBAR_MIDDLE_HIERARCHY_SIZE, front_bullbar_middle_hierarchy));
        }

        if(draw_moving_objects_flag)
        {
		   carmen_vector_3D_t offset = get_position_offset();
           offset.z += sensor_board_1_pose.position.z;

		   draw_moving_objects_point_clouds(moving_objects_point_clouds, moving_objects_point_clouds_size, offset);
		   draw_tracking_moving_objects(moving_objects_tracking, current_num_point_clouds, offset, draw_particles_flag);

		   draw_ldmrs_objects(ldmrs_objects_tracking, num_ldmrs_objects, ldmrs_min_velocity);
        }

        if (draw_gps_flag)
            draw_gps(gps_trail, gps_nr, gps_size);

        if (draw_xsens_gps_flag)
            draw_gps_xsens_xyz(xsens_xyz_trail, gps_size);

        if (draw_odometry_flag)
            draw_odometry(odometry_trail, odometry_size);

        if (draw_localize_ackerman_flag)
            draw_localize_ackerman(localize_ackerman_trail, localize_ackerman_size);

        if (draw_particles_flag)
        {
            draw_particles(fused_odometry_particles_pos, fused_odometry_particles_weight, num_fused_odometry_particles, 0);
            draw_particles(localizer_prediction_particles_pos, localizer_prediction_particles_weight, num_localizer_prediction_particles, 1);
            draw_particles(localizer_correction_particles_pos, localizer_correction_particles_weight, num_localizer_correction_particles, 2);
        }

        if (draw_map_flag)
            draw_map(m_drawer, get_position_offset());

        if (draw_trajectory_flag1)
        {
            draw_trajectory(t_drawer1, get_position_offset());
        	for (unsigned int i = 0; i < t_drawerTree.size(); i++)
        		draw_trajectory(t_drawerTree[i], get_position_offset());
        }

        if (draw_trajectory_flag2)
            draw_trajectory(t_drawer2, get_position_offset());

        if (draw_trajectory_flag3)
            draw_trajectory(t_drawer3, get_position_offset());

        if (draw_map_image_flag)
        {
            if (first_download_map_have_been_aquired)
            {
                IplImage *img = NULL;

                if (new_map_has_been_received)
                {
                    img = cvCreateImageHeader(cvSize(download_map_message.width, download_map_message.height), IPL_DEPTH_8U, 3);
                    img->imageData = download_map_message.image_data;

                    new_map_has_been_received = 0;
                }

                draw_map_image(get_position_offset(), download_map_message.map_center, 153.6 /* 0.3 pixels per meter * 512 pixels of the image */, img);

                if (img != NULL)
                    cvReleaseImageHeader(&img);
            }
        }

        if (draw_localize_image_flag && localize_imagepos_base_initialized)
        {
                draw_localize_image(true,
                		get_position_offset(),
                		localize_imagepos_base_message.pose,
						localize_imagepos_base_message.image_data,
						localize_imagepos_base_message.width,
						localize_imagepos_base_message.height,
						camera_square_size
						);
        }

        if (draw_localize_image_flag && localize_imagepos_curr_initialized)
        {
                draw_localize_image(false,
                		get_position_offset(),
                		localize_imagepos_curr_message.pose,
						localize_imagepos_curr_message.image_data,
						localize_imagepos_curr_message.width,
						localize_imagepos_curr_message.height,
						camera_square_size
						);
        }

        if (!gps_fix_flag)
            draw_gps_fault_signal();

#ifdef TEST_LANE_ANALYSIS
        if (draw_lane_analysis_flag) draw_lane_analysis(lane_drawer);
#endif

        draw_interface(i_drawer);
    }
}


void
draw_loop_for_picking(window *w)
{
    w = w;
    lastDisplayTime = carmen_get_time();
    double fps = 30.0;

    //    while (showWindow(w))
    {
        //        if (!processWindow(w, mouseFunc, keyPress, keyRelease))
        //        {
        //            break;
        //        }

        double sleepTime = 1.0 / fps - (carmen_get_time() - lastDisplayTime);
        if (sleepTime < 0.0)
        {
            sleepTime = 0.01;
        }
        carmen_ipc_sleep(sleepTime);
        lastDisplayTime = carmen_get_time();

        if (follow_car_flag)
        {
            set_camera_offset(car_fused_pose.position);
        }

        reset_camera();

        if (draw_xsens_orientation_flag)
        {
            glColor3f(0.4, 1.0, 0.4);
            draw_xsens_orientation(xsens_orientation, xsens_yaw_bias, xsens_pose, sensor_board_1_pose, car_fused_pose);
            glColor3f(1.0, 0.4, 0.4);
            draw_xsens_orientation(xsens_orientation, 0.0, xsens_pose, sensor_board_1_pose, car_fused_pose);
        }

        if (draw_car_flag)
        {
            draw_car_at_pose(car_drawer, car_fused_pose);
        }

        if (draw_stereo_cloud_flag)
        {
//            draw_stereo_point_cloud(stereo_point_cloud, stereo_point_cloud_size);
            draw_variable_velodyne(var_v_drawer);
        }

        if (draw_points_flag == 1)
        {
            draw_laser_points(laser_points, laser_size);
            draw_laser_points(ldmrs_points, ldmrs_size);
        }
        else if (draw_points_flag == 2)
        {
            draw_point_cloud(laser_drawer);
            draw_point_cloud(ldmrs_drawer);
        }

        if (draw_velodyne_flag == 1)
        {
            draw_velodyne_points(velodyne_points, velodyne_size);
        }
        else if (draw_velodyne_flag == 2)
        {
            //draw_velodyne_points(&(velodyne_points[last_velodyne_position]), 1);           
            draw_point_cloud(velodyne_drawer);
        }
        else if (draw_velodyne_flag == 3)
        {
            draw_velodyne_360(v_360_drawer, car_fused_pose);
        }
        else if (draw_velodyne_flag == 4)
        {
            draw_variable_velodyne(var_v_drawer);
        }
        else if (draw_velodyne_flag == 5)
        {
            draw_velodyne_intensity(v_int_drawer);
        }

        if (draw_rays_flag)
        {
//            carmen_vector_3D_t offset = get_position_offset();
//            draw_moving_objects_point_clouds(moving_objects_point_clouds, moving_objects_point_clouds_size, offset);
            draw_laser_rays(laser_points[last_laser_position], get_laser_position(car_fused_pose.position));
        }

        if(draw_moving_objects_flag)
        {
           carmen_vector_3D_t offset = get_position_offset();
           offset.z += sensor_board_1_pose.position.z;

           draw_moving_objects_point_clouds(moving_objects_point_clouds, moving_objects_point_clouds_size, offset);
           draw_tracking_moving_objects(moving_objects_tracking, current_num_point_clouds, offset, draw_particles_flag);

           draw_ldmrs_objects(ldmrs_objects_tracking, num_ldmrs_objects, ldmrs_min_velocity);
        }

        if (draw_gps_flag)
            draw_gps(gps_trail, gps_nr, gps_size);

        if (draw_xsens_gps_flag)
            draw_gps_xsens_xyz(xsens_xyz_trail, gps_size);

        if (draw_odometry_flag)
            draw_odometry(odometry_trail, odometry_size);

        if (draw_localize_ackerman_flag)
            draw_localize_ackerman(localize_ackerman_trail, localize_ackerman_size);

        if (draw_particles_flag)
        {
            draw_particles(fused_odometry_particles_pos, fused_odometry_particles_weight, num_fused_odometry_particles, 0);
            draw_particles(localizer_prediction_particles_pos, localizer_prediction_particles_weight, num_localizer_prediction_particles, 1);
            draw_particles(localizer_correction_particles_pos, localizer_correction_particles_weight, num_localizer_correction_particles, 2);
        }

        if (draw_map_flag)
            draw_map(m_drawer, get_position_offset());

        if (draw_trajectory_flag1)
            draw_trajectory(t_drawer1, get_position_offset());

        if (draw_trajectory_flag2)
            draw_trajectory(t_drawer2, get_position_offset());

        if (draw_trajectory_flag3)
            draw_trajectory(t_drawer3, get_position_offset());

        if (draw_map_image_flag)
        {
            if (first_download_map_have_been_aquired)
            {
                IplImage *img = NULL;

                if (new_map_has_been_received)
                {
                    img = cvCreateImageHeader(cvSize(download_map_message.width, download_map_message.height), IPL_DEPTH_8U, 3);
                    img->imageData = download_map_message.image_data;

                    new_map_has_been_received = 0;
                }

                draw_map_image(get_position_offset(), download_map_message.map_center, 153.6 /* 0.3 pixels per meter * 512 pixels of the image */, img);

                if (img != NULL)
                    cvReleaseImageHeader(&img);
            }
        }

        if (!gps_fix_flag)
            draw_gps_fault_signal();

        //draw_interface(i_drawer);
    }
}

static void
subscribe_ipc_messages(void)
{
    carmen_rddf_define_messages();

    carmen_fused_odometry_subscribe_fused_odometry_message(NULL,
																(carmen_handler_t) carmen_fused_odometry_message_handler,
																CARMEN_SUBSCRIBE_LATEST);
    carmen_fused_odometry_subscribe_fused_odometry_particle_message(NULL,
                                                                    (carmen_handler_t) carmen_fused_odometry_particle_message_handler,
                                                                    CARMEN_SUBSCRIBE_LATEST);
    // subscribe na mensagem de cada laser definido na config
    for (int i = 1; i <= num_laser_devices; i++)
    {
        printf("carmen_laser_subscribe: laser_id: %d\n", i);
        carmen_laser_subscribe_laser_message(i, NULL, (carmen_handler_t) carmen_laser_laser_message_handler, CARMEN_SUBSCRIBE_LATEST);
    }

    carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_new_message_handler, CARMEN_SUBSCRIBE_LATEST);

//    carmen_laser_subscribe_ldmrs_objects_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_objects_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_ldmrs_objects_data_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_objects_data_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_laser_message(6, NULL, (carmen_handler_t) carmen_laser_laser_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_moving_objects_point_clouds_subscribe_message(NULL,
    												 (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler,
    												 CARMEN_SUBSCRIBE_LATEST);

    carmen_gps_xyz_subscribe_message(NULL,
                                     (carmen_handler_t) gps_xyz_message_handler,
                                     CARMEN_SUBSCRIBE_LATEST);

    carmen_gps_subscribe_nmea_hdt_message(NULL,
                                     (carmen_handler_t) gps_nmea_hdt_message_handler,
                                     CARMEN_SUBSCRIBE_LATEST);

    carmen_xsens_subscribe_xsens_global_matrix_message(NULL,
                                                       (carmen_handler_t) xsens_matrix_message_handler,
                                                       CARMEN_SUBSCRIBE_LATEST);

    carmen_xsens_xyz_subscribe_message(NULL,
                                       (carmen_handler_t) xsens_xyz_message_handler,
                                       CARMEN_SUBSCRIBE_LATEST);

    carmen_xsens_subscribe_xsens_global_quat_message(NULL,
                                                     (carmen_handler_t) xsens_mti_message_handler,
                                                     CARMEN_SUBSCRIBE_LATEST);

    carmen_stereo_point_cloud_subscribe_stereo_point_cloud_message(NULL,
                                                                   (carmen_handler_t) stereo_point_cloud_message_handler,
                                                                   CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL,
                                                   (carmen_handler_t) velodyne_partial_scan_message_handler,
                                                   CARMEN_SUBSCRIBE_LATEST);

    carmen_download_map_subscribe_message(NULL,
                                          (carmen_handler_t) carmen_download_map_handler,
                                          CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_neural_subscribe_imagepos_keyframe_message(NULL,
                                          (carmen_handler_t) carmen_localize_neural_base_message_handler,
                                          CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_neural_subscribe_imagepos_curframe_message(NULL,
                                          (carmen_handler_t) carmen_localize_neural_curr_message_handler,
                                          CARMEN_SUBSCRIBE_LATEST);

    carmen_stereo_velodyne_subscribe_scan_message(camera, NULL,
                                                  (carmen_handler_t) velodyne_variable_scan_message_handler,
                                                  CARMEN_SUBSCRIBE_LATEST);

    carmen_stereo_velodyne_subscribe_scan_message(1, NULL,
                                                      (carmen_handler_t) sick_variable_scan_message_handler,
                                                      CARMEN_SUBSCRIBE_LATEST);

//    carmen_mapper_subscribe_map_message(NULL,
//                                          (carmen_handler_t) mapper_map_message_handler,
//                                          CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_compact_cost_map(NULL,
													(carmen_handler_t) map_server_compact_cost_map_message_handler,
													CARMEN_SUBSCRIBE_LATEST);

    carmen_navigator_ackerman_subscribe_plan_message(NULL,
                                                     (carmen_handler_t) plan_message_handler,
                                                     CARMEN_SUBSCRIBE_LATEST);

    carmen_obstacle_avoider_subscribe_motion_planner_path_message(NULL,
                                                                  (carmen_handler_t) motion_path_handler,
                                                                  CARMEN_SUBSCRIBE_LATEST);

    carmen_obstacle_avoider_subscribe_path_message(NULL,
                                                   (carmen_handler_t) obstacle_avoider_message_handler,
                                                   CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_goal_list_message(NULL,
                                                         (carmen_handler_t) navigator_goal_list_message_handler,
                                                         CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL,
                                                         (carmen_handler_t) localize_ackerman_handler,
                                                         CARMEN_SUBSCRIBE_LATEST);
    carmen_rddf_subscribe_annotation_message(NULL,
                                             (carmen_handler_t) rddf_annotation_handler,
                                             CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_plan_tree_message(
			NULL,
			(carmen_handler_t)plan_tree_handler,
			CARMEN_SUBSCRIBE_LATEST);
#ifdef TEST_LANE_ANALYSIS
	carmen_elas_lane_analysis_subscribe(NULL, (carmen_handler_t) lane_analysis_handler, CARMEN_SUBSCRIBE_LATEST);
#endif

	carmen_localize_ackerman_subscribe_particle_prediction_message(NULL,
			(carmen_handler_t) carmen_localize_ackerman_particle_prediction_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_particle_correction_message(NULL,
			(carmen_handler_t) carmen_localize_ackerman_particle_correction_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_initialize_message(NULL,
			(carmen_handler_t) carmen_localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
check_annotation_equal_zero()
{
    if (annotation_point_world.x == 0 && annotation_point_world.y == 0 && annotation_point_world.z == 0)
        return 1;
    return 0;
}


int
distance_near(carmen_vector_3D_t annotation_pose, carmen_vector_3D_t delete_pose)
{
    return (sqrt((annotation_pose.x - delete_pose.x) * (annotation_pose.x - delete_pose.x) +
                 (annotation_pose.y - delete_pose.y) * (annotation_pose.y - delete_pose.y)+
                 (annotation_pose.z - delete_pose.z) * (annotation_pose.z - delete_pose.z)));
}

// DELETE
//carmen_annotation_t
//find_nearest_annotation()
//{
//	double d;
//	double smallest_distance = DBL_MAX;
//	int index = 0;
//
//	for (uint i = 0; i < annotations.size(); i++)
//	{
//		d = distance_near(annotations[i].annotation_point, annotation_point_world);
//
//		if (d <= smallest_distance)
//		{
//			smallest_distance = d;
//			index = i;
//		}
//	}
//
//	return (annotations[index]);
//}


void
set_flag_viewer_3D(int flag_num, int value)
{
    switch (flag_num)
    {
    case 0:
        draw_particles_flag = value;
        break;

    case 1:
        draw_points_flag = value;
        break;

    case 2:
        draw_velodyne_flag = value;
        break;

    case 3:
        draw_stereo_cloud_flag = value;
        break;

    case 4:
        draw_car_flag = value;
        break;

    case 5:
        draw_rays_flag = value;
        break;

    case 6:
        draw_map_image_flag = value;
        break;

    case 7:
        weight_type_flag = value;
        break;

    case 8:
        draw_gps_flag = value;
        break;

    case 9:
        draw_odometry_flag = value;
        break;

    case 10:
        draw_xsens_gps_flag = value;
        break;

    case 11:
        follow_car_flag = value;
        break;

    case 12:
        draw_map_flag = value;
        break;

    case 13:
        zero_z_flag = value;
        break;

    case 14:
        draw_trajectory_flag1 = value;
        break;

    case 15:
        draw_xsens_orientation_flag = value;
        break;

    case 16:
        draw_localize_ackerman_flag = value;
        break;

    case 17:
        draw_trajectory_flag2 = value;
        break;

    case 18:
        draw_trajectory_flag3 = value;
        break;

    case 19:
        draw_annotation_flag = value;
        break;

    case 20:

        if (!check_annotation_equal_zero())
            carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT), RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT, 0);
        break;

    case 21:
        if (!check_annotation_equal_zero())
        {
            if (value == 0)
            {
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation,
                		rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_BUMP),
						RDDF_ANNOTATION_TYPE_BUMP, RDDF_ANNOTATION_CODE_NONE);
            }
            else if (value == 20)
            {
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation,
                		rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN),
						RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN, RDDF_ANNOTATION_CODE_SPEED_LIMIT_20);
            }
            else if (value == 30)
            {
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation,
                		rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN),
						RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN, RDDF_ANNOTATION_CODE_SPEED_LIMIT_30);
            }
        }
        break;

    case 22:
        if (!check_annotation_equal_zero())
            carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK), RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK, 0);
        break;

    case 23:
        if (!check_annotation_equal_zero())
            carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_STOP), RDDF_ANNOTATION_TYPE_STOP, 0);
        break;

    case 24:
        if (!check_annotation_equal_zero())
            carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_BARRIER), RDDF_ANNOTATION_TYPE_BARRIER, 0);
        break;

    case 25:
        if (!check_annotation_equal_zero())
            carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_BUMP), RDDF_ANNOTATION_TYPE_BUMP, 0);
        break;

    case 26:
        if (!check_annotation_equal_zero())
        {
            if (value == 0)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_0);
            else if (value == 5)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_5);
            else if (value == 10)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_10);
            else if (value == 15)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_15);
            else if (value == 20)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_20);
            else if (value == 30)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_30);
            else if (value == 40)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_40);
            else if (value == 60)
                carmen_rddf_publish_add_annotation_message(annotation_point_world, orientation, rddf_get_annotation_description_by_type(RDDF_ANNOTATION_TYPE_SPEED_LIMIT), RDDF_ANNOTATION_TYPE_SPEED_LIMIT, RDDF_ANNOTATION_CODE_SPEED_LIMIT_60);
        }
        break;
    case 27:
        //carmen_rddf_annotation_message near = get_near_message_to_delete();
        //carmen_rddf_publish_delete_annotation_message(annotation_point_world);
        break;
    case 28:
    	draw_moving_objects_flag = value;
    	break;
    case 29:
    	draw_gps_axis_flag = value;
    	break;
    case 30:
    	draw_localize_image_flag = value;
        break;
    }
}

void
picking(int ev_x, int ev_y)
{
    const int BUFSIZE = 512;
    GLuint selectBuf[BUFSIZE];
    GLint hits;
    GLint viewport[4];
    int x, y;

    GLfloat modelview[16];
    GLfloat projection[16];

    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    x = ev_x;
    y = viewport[3] - ev_y;

    glSelectBuffer(BUFSIZE, selectBuf);
    glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPickMatrix((GLdouble) x, (GLdouble) y, 5.0, 5.0, viewport);
    glMultMatrixf(projection);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixf(modelview);

    draw_loop_for_picking(w);

    glPopMatrix();
    glFlush();

    hits = glRenderMode(GL_RENDER);

    GLdouble hx, hy, hz;

    GLdouble modelviewd[16];
    GLdouble projectiond[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelviewd);
    glGetDoublev(GL_PROJECTION_MATRIX, projectiond);

    GLfloat winZ;
    glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    //only to do gcc happy
    if (hits)
    {
    }

    //printf("WinZ %f\n", winZ); //point in 3D
    if (winZ != 1.000000)
    {
        gluUnProject(x, y, winZ,
                     modelviewd,
                     projectiond,
                     viewport,
                     &hx, &hy, &hz);
        //printf("Point at: %lf %lf %lf \n", hx, hy, hz); // OpenGL Draw

        annotation_point.x = hx;
        annotation_point.y = hy;
        annotation_point.z = hz;
        annotation_point_world.x = hx + get_position_offset().x;
        annotation_point_world.y = hy + get_position_offset().y;
        annotation_point_world.z = hz; // + get_position_offset().z;
        orientation = car_fused_pose.orientation.yaw;

        //printf("Pos at: %lf %lf %lf \n", annotation_point_world.x, annotation_point_world.y, annotation_point_world.z); //Position at World
    }
}

void
mouseFunc(int type, int button, int x, int y)
{
    static int lastX = 0;
    static int lastY = 0;
    static int pressed = 0;

    interface_mouse_func(i_drawer, type, button, x, y);

    if (type == 4)
        pressed = 1;
    else if (type == 5)
        pressed = 0;

    if (button == 4)
    {
        // zoom in
        carmen_vector_3D_t displacement = {1.0, 0.0, 0.0};
        move_camera(displacement);
    }
    else if (button == 5)
    {
        // zoom out
        carmen_vector_3D_t displacement = {-1.0, 0.0, 0.0};
        move_camera(displacement);
    }

    double dx = (x - lastX) / 2.0;
    double dy = (y - lastY) / 2.0;

    lastX = x;
    lastY = y;

    if (pressed)
    {
        carmen_orientation_3D_t rotation = {0.0, carmen_degrees_to_radians(dy), carmen_degrees_to_radians(dx)};
        rotate_camera_offset(rotation);
    }

    if (draw_annotation_flag)
    {
        if (button == 3 && type == 5)
        {
            picking(x, y);
            //take_position_for_annotation_unproject(x,y);
        }
    }
    //printf("mouse - type: %d, button: %d, x: %d, y: %d\n", type, button, x, y);
}

void
keyPress(int code)
{
    switch (code)
    {
    case 111: // UP
    {
        carmen_orientation_3D_t rotation = {0.0, carmen_degrees_to_radians(4.0), 0.0};
        rotate_camera(rotation);
    }
        break;

    case 113: // LEFT
    {
        carmen_orientation_3D_t rotation = {-carmen_degrees_to_radians(4.0), 0.0, 0.0};
        rotate_camera(rotation);
    }
        break;

    case 114: // RIGHT
    {
        carmen_orientation_3D_t rotation = {carmen_degrees_to_radians(4.0), 0.0, 0.0};
        rotate_camera(rotation);
    }
        break;

    case 116: // DOWN
    {
        carmen_orientation_3D_t rotation = {0.0, -carmen_degrees_to_radians(4.0), 0.0};
        rotate_camera(rotation);
    }
        break;

    case 65: // SPACE
    {
        follow_car_flag = !follow_car_flag;
    }
        break;

    case 27: // R
    {
        draw_rays_flag = !draw_rays_flag;
    }
        break;

    case 33: // P
    {
        draw_particles_flag = !draw_particles_flag;
    }
        break;

    case 55: // V
    {
        draw_velodyne_flag = (draw_velodyne_flag + 1) % 5;
    }
        break;

    case 58: // M
    {
        draw_points_flag = (draw_points_flag + 1) % 3;
    }
        break;

    case 57: //N
    {
        draw_stereo_cloud_flag = !draw_stereo_cloud_flag;
    }
        break;

    case 54: // C
    {
        draw_car_flag = !draw_car_flag;
    }
        break;

    case 31: // I
    {
        draw_map_image_flag = !draw_map_image_flag;
    }
        break;

    case 24: // Q
    {
        carmen_vector_3D_t displacement = {0.0, 0.5, 0.0};
        move_camera(displacement);
    }
        break;

    case 26: // E
    {
        carmen_vector_3D_t displacement = {0.0, -0.5, 0.0};
        move_camera(displacement);
    }
        break;

    case 25: // W
    {
        carmen_vector_3D_t displacement = {0.5, 0.0, 0.0};
        move_camera(displacement);
    }
        break;

    case 38: // A
    {
        carmen_orientation_3D_t rotation = {0.0, 0.0, carmen_degrees_to_radians(4.0)};
        rotate_camera(rotation);

    }
        break;

    case 39: // S
    {
        carmen_vector_3D_t displacement = {-0.5, 0.0, 0.0};
        move_camera(displacement);
    }
        break;

    case 40: // D
    {
        carmen_orientation_3D_t rotation = {0.0, 0.0, -carmen_degrees_to_radians(4.0)};
        rotate_camera(rotation);
    }
        break;

    case 28: // T
    {
        weight_type_flag = (weight_type_flag + 1) % 3;
    }
        break;

    case 44: // J
    {
        draw_gps_flag = !draw_gps_flag;
    }
        break;

    case 45: // K
    {
        draw_odometry_flag = !draw_odometry_flag;
    }
        break;

    case 46: // L
    {
        draw_xsens_gps_flag = !draw_xsens_gps_flag;
    }
        break;

    case 29: // U
    {
        // zoom in
        carmen_vector_3D_t displacement = {1.0, 0.0, 0.0};
        move_camera(displacement);
    }
        break;

    case 30: // I
    {
        // zoom out
        carmen_vector_3D_t displacement = {-1.0, 0.0, 0.0};
        move_camera(displacement);
    }
        break;
    }
}

void
keyRelease(int code)
{
    code = code; // just to make gcc happy
}

void read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "commandline",	(char *) "fv_flag",		CARMEN_PARAM_ONOFF,	&(force_velodyne_flag),		0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char** argv)
{
    argc_g = argc;
    argv_g = argv;

    w = initWindow();
    initGl();

    carmen_ipc_initialize(argc_g, argv_g);
    carmen_param_check_version(argv[0]);

    read_parameters(argc, argv);

    init_stuff(argc_g, argv_g);

    subscribe_ipc_messages();

    draw_loop(w);

    carmen_ipc_disconnect();

    destroy_stuff();

    destroyWindow(w);

    return 0;
}
