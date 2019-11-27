#include <stdio.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>
#include <list>
#include <limits>

using namespace std;

#include <carmen/carmen.h>
#include <carmen/readlog.h>
#include <carmen/carmen_stdio.h>
#include <carmen/velodyne_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/collision_detection.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/voice_interface_messages.h>
#include <carmen/voice_interface_interface.h>
#include <carmen/road_mapper.h>
#include <carmen/grid_mapping.h>

#include "rddf_interface.h"
#include "rddf_messages.h"
#include "rddf_index.h"

#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>
#include <kml/engine/kml_file.h>
#include "g2o/types/slam2d/se2.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>

using namespace g2o;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;

#include "rddf_util.h"

static bool use_road_map = false;
static bool robot_pose_queued = false;
static carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
static carmen_simulator_ackerman_truepos_message *current_truepos_msg = NULL;
static carmen_map_p current_road_map = NULL;

static char *carmen_rddf_filename = NULL;

static int carmen_rddf_perform_loop = 0;
static int carmen_rddf_num_poses_ahead_max = 100;
static double rddf_min_distance_between_waypoints = 0.5;
static double distance_between_front_and_rear_axles;
static double distance_between_front_car_and_front_wheels;
static double turning_radius = 0.0;
static double maximum_curvature = numeric_limits<double>::max();
static double default_search_radius = 1.0;
static int road_mapper_kernel_size = 7;
static int traffic_sign_is_on = false;
static int traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
static double traffic_sign_curvature = 0.0;
static int state_traffic_sign_is_on = false;
static int state_traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
static double state_traffic_sign_curvature = 0.0;
static carmen_point_t state_traffic_sign_pose = {0.0, 0.0, 0.0};
static int dynamic_plot_state = 0;

static int carmen_rddf_end_point_is_set = 0;
static carmen_point_t carmen_rddf_end_point;

static int carmen_rddf_nearest_waypoint_is_set = 0;
static carmen_point_t carmen_rddf_nearest_waypoint_to_end_point;

static carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead = NULL;
static carmen_ackerman_traj_point_t *carmen_rddf_poses_back = NULL;
static int carmen_rddf_num_poses_ahead = 0;
static int carmen_rddf_num_poses_back = 0;
static int *annotations_codes;
static int *annotations;

static int carmen_rddf_pose_initialized = 0;
static int already_reached_nearest_waypoint_to_end_point = 0;

char *carmen_annotation_filename = NULL;
vector<carmen_annotation_t> annotation_read_from_file;
typedef struct
{
	carmen_annotation_t annotation;
	size_t index;
} annotation_and_index;
vector<annotation_and_index> annotations_to_publish;
carmen_rddf_annotation_message annotation_queue_message;

static int use_truepos = 0;

static int traffic_lights_camera = 3;
carmen_traffic_light_message *traffic_lights = NULL;

deque<carmen_rddf_dynamic_annotation_message> dynamic_annotation_messages;

carmen_moving_objects_point_clouds_message *moving_objects = NULL;

bool simulated_pedestrian_on = false;


static void
carmen_rddf_play_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}


int
carmen_rddf_play_nearest_waypoint_reached(carmen_ackerman_traj_point_t pose)
{
	if (sqrt(pow(pose.x - carmen_rddf_nearest_waypoint_to_end_point.x, 2) + pow(pose.y - carmen_rddf_nearest_waypoint_to_end_point.y, 2)) < 2.0)
		return 1;
	else
		return 0;
}


int
carmen_rddf_play_find_position_of_nearest_waypoint(carmen_ackerman_traj_point_t *poses_ahead, int num_poses)
{
	int i, position = -1;

	for (i = 0; i < num_poses; i++)
	{
		if (carmen_rddf_play_nearest_waypoint_reached(poses_ahead[i]))
		{
			position = i;
			break;
		}
	}

	return position;
}


int
carmen_rddf_play_adjust_poses_ahead_and_add_end_point_to_list(carmen_ackerman_traj_point_t *poses_ahead, int num_poses, int nearest_end_waypoint_position, int *rddf_annotations)
{
	int position_nearest_waypoint = nearest_end_waypoint_position;
	int position_end_point = nearest_end_waypoint_position + 1;

	//
	// se o waypoint mais proximo ao end point for o ultimo da lista,
	// a sua posicao eh decrementada para abrir espaco para o end point
	//

	if (nearest_end_waypoint_position == (num_poses - 1))
	{
		position_nearest_waypoint--;
		position_end_point--;
	}

	poses_ahead[position_nearest_waypoint].x = carmen_rddf_nearest_waypoint_to_end_point.x;
	poses_ahead[position_nearest_waypoint].y = carmen_rddf_nearest_waypoint_to_end_point.y;
	rddf_annotations[position_nearest_waypoint] = RDDF_ANNOTATION_TYPE_NONE;

	poses_ahead[position_end_point].x = carmen_rddf_end_point.x;
	poses_ahead[position_end_point].y = carmen_rddf_end_point.y;
	poses_ahead[position_end_point].theta = carmen_rddf_end_point.theta;
	poses_ahead[position_end_point].v = 0.0;
	rddf_annotations[position_end_point] = RDDF_ANNOTATION_TYPE_END_POINT_AREA;

	return (position_end_point + 1);
}


int
carmen_rddf_play_check_if_end_point_is_reachable(carmen_ackerman_traj_point_t *poses_ahead, int num_poses, int *rddf_annotations)
{
	if (carmen_rddf_nearest_waypoint_is_set)
	{
		// se o robo ja passou pelo waypoint mais proximo do end point, so o end point eh publicado
		if (already_reached_nearest_waypoint_to_end_point)
		{
			poses_ahead[0].x = carmen_rddf_end_point.x;
			poses_ahead[0].y = carmen_rddf_end_point.y;
			poses_ahead[0].theta = carmen_rddf_end_point.theta;
			poses_ahead[0].v = 0.0;
			rddf_annotations[0] = RDDF_ANNOTATION_TYPE_END_POINT_AREA;

			return 1;
		}

		// verifica se algum dos waypoints esta a uma distancia minima do waypoint mais proximo do end point
		int nearest_end_waypoint_position = carmen_rddf_play_find_position_of_nearest_waypoint (poses_ahead, num_poses);

		if (nearest_end_waypoint_position != -1)
		{
			// se um dos dois primeiros waypoints esta a uma distancia minima do waypoint mais proximo do end point, passamos a publicar somente o end point
			if (nearest_end_waypoint_position < 2)
				already_reached_nearest_waypoint_to_end_point = 1;

			return carmen_rddf_play_adjust_poses_ahead_and_add_end_point_to_list(poses_ahead, num_poses, nearest_end_waypoint_position, rddf_annotations);
		}
		else
			return num_poses;
	}
	else
		return num_poses;
}


static void
clear_annotations(int *rddf_annotations, int num_annotations)
{
	int i;

	for(i = 0; i < num_annotations; i++)
	{
		rddf_annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
}


void
clear_annotations()
{
	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
}


static int
carmen_rddf_play_find_nearest_poses_ahead(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_ahead_max, int *rddf_annotations)
{
	clear_annotations(rddf_annotations, num_poses_ahead_max);

	int num_poses_ahead = carmen_search_next_poses_index(x, y, yaw, timestamp, poses_ahead, poses_back, num_poses_back, num_poses_ahead_max, rddf_annotations, carmen_rddf_perform_loop);
	return carmen_rddf_play_check_if_end_point_is_reachable(poses_ahead, num_poses_ahead, rddf_annotations);
}


int
get_key_non_blocking(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return (ch);
}


bool
pedestrian_track_busy(carmen_moving_objects_point_clouds_message *moving_objects, carmen_annotation_t pedestrian_track_annotation)
{
	int ch = get_key_non_blocking();

	if (ch == 'p')
		simulated_pedestrian_on = true;
	if (ch == ' ')
		simulated_pedestrian_on = false;

	if (simulated_pedestrian_on)
		return (true);

	if (moving_objects == NULL)
		return (false);

	carmen_vector_2D_t world_point;
	double displacement = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
	double theta = pedestrian_track_annotation.annotation_orientation;
	world_point.x = pedestrian_track_annotation.annotation_point.x + displacement * cos(theta);
	world_point.y = pedestrian_track_annotation.annotation_point.y + displacement * sin(theta);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if ((strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0) &&
			(DIST2D(moving_objects->point_clouds[i].object_pose, world_point) < pedestrian_track_annotation.annotation_point.z))
			return (true);
	}
	return (false);
}



typedef enum
{
	Free_Crosswalk,
	Stopping_Busy_Crosswalk,
	Stopped_Busy_Crosswalk,
	Leaving_Crosswalk
} carmen_rddf_play_state;


carmen_rddf_play_state crosswalk_state = Free_Crosswalk;


carmen_vector_2D_t
get_displaced_annotation_position(carmen_annotation_t pedestrian_track_annotation)    // The crosswalk annotated position is displaced by the distance from rear axle to car front
{                                                                                     // because the annotation position is made this way
	carmen_vector_2D_t desplaced_crosswalk_pose;
	double displacement = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
	double theta = pedestrian_track_annotation.annotation_orientation;
	desplaced_crosswalk_pose.x = pedestrian_track_annotation.annotation_point.x + displacement * cos(theta);
	desplaced_crosswalk_pose.y = pedestrian_track_annotation.annotation_point.y + displacement * sin(theta);

	return (desplaced_crosswalk_pose);
}


bool
pedestrian_about_to_enter_crosswalk(t_point_cloud_struct moving_object, carmen_annotation_t pedestrian_track_annotation, double ray)
{
	carmen_vector_2D_t desplaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	double pedestrian_time_to_crosswalk_border = 999.9;//DBL_MAX;
	double car_time_to_crosswalk_center = 999.9;//DBL_MAX;
	double orientantion;

	orientantion = carmen_normalize_theta(atan2(desplaced_crosswalk_pose.y - moving_object.object_pose.y, desplaced_crosswalk_pose.x - moving_object.object_pose.x));

	if (abs(carmen_normalize_theta(orientantion - moving_object.orientation)) > 0.53)   // ~30 degrees; Pedestrian is not walking towards crosswalk
	{
		return (false);
	}
	double x_border = ray * cos((orientantion)) + desplaced_crosswalk_pose.x;            // Position the pedestrian will be when it hits crosswalk border
	double y_border = ray * sin((orientantion)) + desplaced_crosswalk_pose.y;

	double d_x = x_border - moving_object.object_pose.x;
	double d_y = y_border - moving_object.object_pose.y;

	if (moving_object.linear_velocity > 0.2)
		pedestrian_time_to_crosswalk_border = abs((sqrt((d_x * d_x) + (d_y * d_y)))) / moving_object.linear_velocity;

	if (current_globalpos_msg->v > 0.5)
		car_time_to_crosswalk_center = abs(DIST2D(current_globalpos_msg->globalpos, desplaced_crosswalk_pose)) / current_globalpos_msg->v;

	printf("%lf %lf ", orientantion, moving_object.orientation);
	printf("Bx %lf By %lf ", x_border, y_border);
	printf("TC %lf TP %lf", car_time_to_crosswalk_center, pedestrian_time_to_crosswalk_border);
	printf("C %lf P %lf\n", current_globalpos_msg->v, moving_object.linear_velocity);

	if (car_time_to_crosswalk_center < pedestrian_time_to_crosswalk_border)
		return (false);

	//printf("about_to_enter\n");
	return (true);
}


bool
pedestrian_in_crosswalk(carmen_moving_objects_point_clouds_message *moving_objects, carmen_annotation_t pedestrian_track_annotation)
{
	double ray = pedestrian_track_annotation.annotation_point.z;
	carmen_vector_2D_t desplaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if ((strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0 &&
			 DIST2D(moving_objects->point_clouds[i].object_pose, desplaced_crosswalk_pose) < ray)
			 ||
			 pedestrian_about_to_enter_crosswalk(moving_objects->point_clouds[i], pedestrian_track_annotation, ray))
		{
			//printf("In\n");
			return (true);
		}
	}
	//printf("Out\n");
	return (false);
}


bool                                // TODO checar se o pedestre não esta no caminho
pedestrian_crossing(carmen_moving_objects_point_clouds_message *moving_objects_msg, carmen_annotation_t pedestrian_track_annotation)
{
	double ray = pedestrian_track_annotation.annotation_point.z;
	carmen_vector_2D_t desplaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if (strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0)
		{
			if ((DIST2D(moving_objects->point_clouds[i].object_pose, desplaced_crosswalk_pose) < ray &&                    // Inside the crosswalk circle
				 moving_objects_msg->point_clouds[i].linear_velocity > 0.2 &&                                              // Moving faster than 0.2m/s
				 abs(current_globalpos_msg->globalpos.theta - moving_objects_msg->point_clouds[i].orientation) > 0.2)      // Not moving parallel to the car (sideways with the crosswalk)
				 )//||
				 //pedestrian_about_to_enter_crosswalk(moving_objects_msg->point_clouds[i], desplaced_crosswalk_pose, ray))
			{
					return (true);
			}
		}
	}
	return (false);
}


bool
pedestrian_track_busy_new(carmen_moving_objects_point_clouds_message *moving_objects_msg, carmen_annotation_t pedestrian_track_annotation)
{
	if (moving_objects_msg == NULL || moving_objects_msg->num_point_clouds < 1)
		return (false);

	carmen_vector_2D_t desplaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	switch (crosswalk_state)
	{
		case Free_Crosswalk:
			//printf("Free_Crosswalk \n");
			if (pedestrian_in_crosswalk(moving_objects_msg, pedestrian_track_annotation))
			{
				crosswalk_state = Stopping_Busy_Crosswalk;
				return (true);
			}
			return (false);

		case Stopping_Busy_Crosswalk:
			//printf("Stopping_Busy_Crosswalk %lf %lf\n", current_globalpos_msg->v, DIST2D(current_globalpos_msg->globalpos, desplaced_crosswalk_pose));
			if (!pedestrian_in_crosswalk(moving_objects_msg, pedestrian_track_annotation))
			{
				crosswalk_state = Free_Crosswalk;
				return (false);
			}
			else if (current_globalpos_msg->v < 0.15 && DIST2D(current_globalpos_msg->globalpos, desplaced_crosswalk_pose) < 20.0) // || dist stop point < 2.0
			{
				crosswalk_state = Stopped_Busy_Crosswalk;
			}
			return (true);

		case Stopped_Busy_Crosswalk:
			//printf("Stopped_Busy_Crosswalk \n");
			if (!pedestrian_crossing(moving_objects_msg, pedestrian_track_annotation))
			{
				crosswalk_state = Leaving_Crosswalk;
				return (false);
			}
			return (true);

		case Leaving_Crosswalk:
			//printf("Leaving_Crosswalk %lf\n", DIST2D(current_globalpos_msg->globalpos, desplaced_crosswalk_pose));
			if (pedestrian_crossing(moving_objects_msg, pedestrian_track_annotation))
			{
				printf("pedestrian_crossing \n");
				crosswalk_state = Stopped_Busy_Crosswalk;
				return (true);
			}
			else if (DIST2D(current_globalpos_msg->globalpos, desplaced_crosswalk_pose) < 2.0)
			{
				crosswalk_state = Free_Crosswalk;
			}
			return (false);
	}
	return (true);
}


bool
add_annotation(double x, double y, double theta, size_t annotation_index)
{
	double dx = annotation_read_from_file[annotation_index].annotation_point.x - x;
	double dy = annotation_read_from_file[annotation_index].annotation_point.y - y;
	double dist = sqrt(pow(dx, 2) + pow(dy, 2));
	double angle_to_annotation = carmen_radians_to_degrees(fabs(carmen_normalize_theta(theta - annotation_read_from_file[annotation_index].annotation_orientation)));

	if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if ((dist < MAX_TRAFFIC_LIGHT_DISTANCE) && orientation_ok)
		{
			if ((traffic_lights != NULL) &&
				(traffic_lights->num_traffic_lights > 0)) // @@@ Alberto: deveria verificar a maioria...
			{
				int num_red = 0;
				for (int i = 0; i < traffic_lights->num_traffic_lights; i++)
					if (traffic_lights->traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED)
						num_red++;

				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				if (num_red > 0)
					annotation_i.annotation.annotation_code = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;
				else
					annotation_i.annotation.annotation_code = traffic_lights->traffic_lights[0].color;
				annotations_to_publish.push_back(annotation_i);
			}
			else
			{
				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				annotations_to_publish.push_back(annotation_i);
			}
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if ((dist < 100.0) && orientation_ok)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			if (pedestrian_track_busy_new(moving_objects, annotation_read_from_file[annotation_index]))
				annotation_i.annotation.annotation_code = RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY;
			else
				annotation_i.annotation.annotation_code = RDDF_ANNOTATION_CODE_NONE;
			annotations_to_publish.push_back(annotation_i);
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_STOP)
	{
		bool orientation_ok = angle_to_annotation < 15.0 ? true : false;

		if ((dist < 20.0) && orientation_ok)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_code == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF)
	{
		if (dist < 20.0)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST)
	{
		if (dist < 100.0)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
			return (true);
		}
	}
	else if (dist < 20.0)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if (orientation_ok)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
		}
		return (true);
	}

	return (false);
}


bool
is_in_the_zone_of_influence(carmen_point_t robot_pose, carmen_point_t annotation_point, double max_delta_angle, double max_distance)
{
	double delta_angle = fabs(robot_pose.theta - annotation_point.theta);
	double distance = DIST2D(robot_pose, annotation_point);
	bool delta_angle_ok = (carmen_radians_to_degrees(carmen_normalize_theta(delta_angle)) < max_delta_angle);
	bool distance_ok  = (distance < max_distance);
	bool in_the_zone_of_influence = (delta_angle_ok && distance_ok);

	return (in_the_zone_of_influence);
}


bool
is_in_the_zone_of_influence(carmen_point_t robot_pose, carmen_point_t annotation_point, double max_delta_angle, double max_distance_across, double max_distance_aligned)
{
	double delta_angle = fabs(robot_pose.theta - annotation_point.theta);
	double distance = DIST2D(robot_pose, annotation_point);
	double distance_across  = fabs(distance * sin(delta_angle));
	double distance_aligned = fabs(distance * cos(delta_angle));
	bool delta_angle_ok = (carmen_radians_to_degrees(carmen_normalize_theta(delta_angle)) < max_delta_angle);
	bool distance_across_ok  = (distance_across < max_distance_across);
	bool distance_aligned_ok = (distance_aligned < max_distance_aligned);
	bool in_the_zone_of_influence = (delta_angle_ok && distance_across_ok && distance_aligned_ok);

	return (in_the_zone_of_influence);
}


void
find_direction_traffic_sign(carmen_point_t robot_pose, bool ahead)
{
	size_t annotation_index;

	for (annotation_index = 0; annotation_index < annotation_read_from_file.size(); annotation_index++)
	{
		if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN)
		{
			carmen_ackerman_traj_point_t annotation_point;
			annotation_point.x = annotation_read_from_file[annotation_index].annotation_point.x;
			annotation_point.y = annotation_read_from_file[annotation_index].annotation_point.y;
			annotation_point.theta = annotation_read_from_file[annotation_index].annotation_orientation;
			double distance_car_pose_car_front = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
			carmen_point_t actual_annotation_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&annotation_point, distance_car_pose_car_front);
			if (is_in_the_zone_of_influence(robot_pose, actual_annotation_point, 70.0, default_search_radius  /*, 1.5 * rddf_min_distance_between_waypoints */ ))
				break;
		}
	}

	if (annotation_index < annotation_read_from_file.size())
	{
		double curvature = annotation_read_from_file[annotation_index].annotation_point.z;
		traffic_sign_curvature = MIN(fabs(curvature), fabs(maximum_curvature)) * sign(curvature);
		traffic_sign_code = annotation_read_from_file[annotation_index].annotation_code;

		if (ahead)
			traffic_sign_is_on = (traffic_sign_code != RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF);
		else
			traffic_sign_is_on = (traffic_sign_code == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF);
	}
}


void
carmen_check_for_annotations(carmen_point_t robot_pose,
		carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead, carmen_ackerman_traj_point_t *carmen_rddf_poses_back,
		int carmen_rddf_num_poses_ahead, int carmen_rddf_num_poses_back, double timestamp)
{
	for (size_t annotation_index = 0; annotation_index < annotation_read_from_file.size(); annotation_index++)
	{
		if (add_annotation(robot_pose.x, robot_pose.y, robot_pose.theta, annotation_index))
			continue;

		bool added = false;
		for (int j = 0; j < carmen_rddf_num_poses_ahead; j++)
		{
			if (add_annotation(carmen_rddf_poses_ahead[j].x, carmen_rddf_poses_ahead[j].y, carmen_rddf_poses_ahead[j].theta, annotation_index))
			{
				added = true;
				break;
			}
		}

		if (!added)
		{
			for (int j = 0; j < carmen_rddf_num_poses_back; j++)
				if (add_annotation(carmen_rddf_poses_back[j].x, carmen_rddf_poses_back[j].y, carmen_rddf_poses_back[j].theta, annotation_index))
					break;
		}
	}

	for (size_t j = 0; j < dynamic_annotation_messages.size(); j++)
	{
		if ((timestamp - dynamic_annotation_messages[j].timestamp) < 2.0)
		{
			carmen_annotation_t annotation;
			annotation.annotation_type = dynamic_annotation_messages[j].annotation_type;
			annotation.annotation_code = dynamic_annotation_messages[j].annotation_code;
			annotation.annotation_point = dynamic_annotation_messages[j].annotation_point;
			annotation.annotation_description = dynamic_annotation_messages[j].annotation_description;
			annotation.annotation_orientation = dynamic_annotation_messages[j].annotation_orientation;
			annotation_and_index annotation_i = {annotation, 0};
			annotations_to_publish.push_back(annotation_i);
		}
		else
			dynamic_annotation_messages.erase(dynamic_annotation_messages.begin() + j);
	}
}


void
find_nearest_waypoint_and_dist(carmen_annotation_t annotation, int *nearest_pose_out, double *nearest_pose_dist_out)
{
	int nearest_pose;
	double min_distance_to_annotation;
	double distance_to_annotation;

	nearest_pose = -1;
	min_distance_to_annotation = DBL_MAX;

	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		distance_to_annotation = sqrt(pow(carmen_rddf_poses_ahead[i].x - annotation.annotation_point.x, 2) + pow(carmen_rddf_poses_ahead[i].y - annotation.annotation_point.y, 2));

		if (distance_to_annotation < min_distance_to_annotation)
		{
			min_distance_to_annotation = distance_to_annotation;
			nearest_pose = i;
		}
	}

	(*nearest_pose_out) = nearest_pose;
	(*nearest_pose_dist_out) = min_distance_to_annotation;
}


int
annotation_is_forward_from_robot(carmen_point_t pose, carmen_annotation_t annotation)
{
	carmen_vector_3D_t annotation_point;

	annotation_point = annotation.annotation_point;

	SE2 robot_pose_mat(pose.x, pose.y, pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return 1;
	else
		return 0;
}


void
set_annotations(carmen_point_t robot_pose)
{
	int nearest_pose;
	double nearest_pose_dist;

	for (uint i = 0; i < annotations_to_publish.size(); i++)
	{
		find_nearest_waypoint_and_dist(annotations_to_publish[i].annotation, &nearest_pose, &nearest_pose_dist);

		if ((nearest_pose >= 0) && nearest_pose_dist < 10.0 && (annotation_is_forward_from_robot(robot_pose, annotations_to_publish[i].annotation)))
		{
			annotations[nearest_pose] = annotations_to_publish[i].annotation.annotation_type;
			annotations_codes[nearest_pose] = annotations_to_publish[i].annotation.annotation_code;
		}
	}
}


void
calculate_phi_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = L * atan(delta_theta / l);
	}

	for (int i = 1; i < (num_poses - 1); i++)
	{
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
	}
}


void
calculate_phi_back(carmen_ackerman_traj_point_t *path, int num_poses)
{
	double L = distance_between_front_and_rear_axles;

	for (int i = (num_poses - 1); i > 0; i--)
	{
		double delta_theta = carmen_normalize_theta(path[i - 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i - 1]);
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = L * atan(delta_theta / l);
	}

	for (int i = (num_poses - 2); i > 0; i--)
	{
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
	}
}


void
calculate_theta_ahead(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
		path[i].theta = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
	if (num_poses > 1)
		path[num_poses - 1].theta = path[num_poses - 2].theta;
}


void
calculate_theta_back(carmen_ackerman_traj_point_t *path, int num_poses)
{
	for (int i = 1; i < num_poses; i++)
//		path[i].theta = atan2(path[i - 1].y - path[i].y, path[i - 1].x - path[i].x);
		path[i].theta = carmen_normalize_theta(atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x) + M_PI);
}


void
calculate_theta_and_phi(carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_back)
{
	calculate_theta_ahead(poses_ahead, num_poses_ahead);
	poses_back[0].theta = poses_ahead[0].theta;
	calculate_theta_back(poses_back, num_poses_back);

	calculate_phi_ahead(poses_ahead, num_poses_ahead);
	poses_back[0].phi = poses_ahead[0].phi;
	calculate_phi_back(poses_back, num_poses_back);
}


//////////////////////////////////////////////////////////////////////////////////

/* GSL - GNU Scientific Library
 * Multidimensional Minimization
 * https://www.gnu.org/software/gsl/doc/html/multimin.html
 *
 * Sebastian Thrun
 */


//Function to be minimized summation[x(i+1)-2x(i)+x(i-1)]
double
my_f(const gsl_vector *v, void *params)
{
	list<carmen_ackerman_traj_point_t> *p = (list<carmen_ackerman_traj_point_t> *) params;
	int i, j, size = (p->size() - 2);           //we have to discount the first and last point that wont be optimized
	double a = 0.0, b = 0.0, sum = 0.0;

	double x_prev = p->front().x;				//x(i-1)
	double x      = gsl_vector_get(v, 0);		//x(i)
	double x_next = gsl_vector_get(v, 1);		//x(i+1)

	double y_prev = p->front().y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);

	for (i = 2, j = (size+2); i < size; i++, j++)
	{
		a = x_next - (2*x) + x_prev;
		b = y_next - (2*y) + y_prev;
		sum += (a*a + b*b);

		x_prev = x;
		x      = x_next;
		x_next = gsl_vector_get(v, i);

		y_prev = y;
		y      = y_next;
		y_next = gsl_vector_get(v, j);
	}

	x_prev = x;
	x      = x_next;
	x_next = p->back().x;

	y_prev = y;
	y      = y_next;
	y_next = p->back().y;

	a = x_next - (2*x) + x_prev;
	b = y_next - (2*y) + y_prev;
	sum += (a*a + b*b);

	return (sum);
}


//The gradient of f, df = (df/dx, df/dy)
//derivative in each point [2x(i-2)-8x(i-1)+12x(i)-8x(i+1)+2x(i+2)]
void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	list<carmen_ackerman_traj_point_t> *p = (list<carmen_ackerman_traj_point_t> *) params;
	int i, j, size =(p->size() - 2);

	double x_prev2= 0;
	double x_prev = p->front().x;
	double x      = gsl_vector_get(v, 0);
	double x_next = gsl_vector_get(v, 1);
	double x_next2= gsl_vector_get(v, 2);
	double sum_x  =  (10*x) - (8*x_next) + (2*x_next2) - (4*x_prev);
	gsl_vector_set(df, 0, sum_x);

	double y_prev2= 0;
	double y_prev = p->front().y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	double y_next2= gsl_vector_get(v, size+2);
	double sum_y  = (10*y) - (8*y_next) + (2*y_next2) - (4*y_prev);
	gsl_vector_set(df, size, sum_y);

	for (i = 3, j = (size+3); i < size; i++, j++)
	{
		x_prev2= x_prev;
		x_prev = x;
		x      = x_next;
		x_next = x_next2;
		x_next2= gsl_vector_get(v, i);
		sum_x = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
		gsl_vector_set(df, (i-2), sum_x);

		y_prev2= y_prev;
		y_prev = y;
		y      = y_next;
		y_next = y_next2;
		y_next2= gsl_vector_get(v, j);
		sum_y = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
		gsl_vector_set(df, (j-2), sum_y);
	}

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	x_next2= p->back().x;
	sum_x  = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
	gsl_vector_set(df, size-2, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	y_next2= p->back().y;
	sum_y  = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
	gsl_vector_set(df, (2*size)-2, sum_y);

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	sum_x  = (2*x_prev2) - (8*x_prev) + (10*x) - (4*x_next);
	gsl_vector_set(df, size-1, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	sum_y  = (2*y_prev2) - (8*y_prev) + (10*y) - (4*y_next);
	gsl_vector_set(df, (2*size)-1, sum_y);
}


// Compute both f and df together
void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


int
smooth_rddf_using_conjugate_gradient(carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_back)
{
	int iter = 0;
	int status, i = 0, j = 0, size;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	list<carmen_ackerman_traj_point_t>::iterator it;
	list<carmen_ackerman_traj_point_t> path;

	for (i = (num_poses_back - 1); i > 0; i--) // skip poses_back[0], because it is equal to poses_ahead[0]
		path.push_back(poses_back[i]);

	for (i = 0; i < num_poses_ahead; i++)
		path.push_back(poses_ahead[i]);

	if (path.size() < 5)
		return (1);

	size = path.size();

	my_func.n = (2 * size) - 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &path;

	v = gsl_vector_alloc ((2 * size) - 4);

	static int count = 0;
	count++;
//	FILE *plot = fopen("gnuplot_smooth_lane.m", "w");

//	fprintf(plot, "a%d = [\n", count);
	it = path.begin();
//	fprintf(plot, "%f %f\n", it->x, it->y);

	it++; // skip the first pose
	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
//		fprintf(plot, "%f %f\n", it->x, it->y);

		gsl_vector_set (v, i, it->x);
		gsl_vector_set (v, j, it->y);
	}

//	fprintf(plot, "%f %f]\n\n", it->x, it->y);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2 * size) - 4);

	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.01);  //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);
		if (status) // error code
			return (0);

		status = gsl_multimin_test_gradient (s->gradient, 0.2);   //(gsl_vector, epsabs) and  |g| < epsabs
		// status == 0 (GSL_SUCCESS), if a minimum has been found
	} while (status == GSL_CONTINUE && iter < 999);

//	printf("status %d, iter %d\n", status, iter);
//	fflush(stdout);
	it = path.begin();

//	fprintf(plot, "b%d = [   \n%f %f\n", count, it->x, it->y);

	it++; // skip the first pose
	for (i = 0, j = (size - 2); i < (size - 2); i++, j++, it++)
	{
		it->x = gsl_vector_get (s->x, i);
		it->y = gsl_vector_get (s->x, j);

//		fprintf(plot, "%f %f\n", it->x, it->y);
	}

//	fprintf(plot, "%f %f]\n\n", it->x, it->y);
//	fprintf(plot, "\nplot (a%d(:,1), a%d(:,2), b%d(:,1), b%d(:,2)); \nstr = input (\"a   :\");\n\n", count, count, count, count);
//	fclose(plot);

	it = path.begin();
	it++;
	for (i = (num_poses_back - 2); i > 0; i--, it++) // skip first and last poses
		poses_back[i] = *it;
	poses_back[0] = *it;

	for (i = 0; i < num_poses_ahead - 1; i++, it++) // skip last pose
		poses_ahead[i] = *it;

	calculate_theta_and_phi(poses_ahead, num_poses_ahead, poses_back, num_poses_back);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);

	return (1);
}

//////////////////////////////////////////////////////////////////////////////////


void
plot_state(carmen_ackerman_traj_point_t *path, int num_points, carmen_ackerman_traj_point_t *path2, int num_points2, bool display)
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xlabel 'x'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'y'\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
		first_time = false;
	}

	if (display)
	{
		system("cp gnuplot_data_lane.txt gnuplot_data_lane_.txt");
		system("cp gnuplot_data_lane2.txt gnuplot_data_lane2_.txt");
	}

	FILE *gnuplot_data_lane  = fopen("gnuplot_data_lane.txt", "w");
	FILE *gnuplot_data_lane2 = fopen("gnuplot_data_lane2.txt", "w");

	for (int i = 0; i < num_points; i++)
	{
		fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf\n", path[i].x, path[i].y,
				0.8 * cos(path[i].theta), 0.8 * sin(path[i].theta), path[i].theta, path[i].phi);
	}
	for (int i = 0; i < num_points2; i++)
	{
		fprintf(gnuplot_data_lane2, "%lf %lf %lf %lf %lf %lf\n", path2[i].x, path2[i].y,
				0.8 * cos(path2[i].theta), 0.8 * sin(path2[i].theta), path2[i].theta, path2[i].phi);
	}
	fclose(gnuplot_data_lane);
	fclose(gnuplot_data_lane2);

	if (display)
	{
		fprintf(gnuplot_pipeMP, "plot "
				"'./gnuplot_data_lane_.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Ahead normal'"
				", './gnuplot_data_lane2_.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Back normal'"
				", './gnuplot_data_lane.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Ahead smooth'"
				", './gnuplot_data_lane2.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane Back smooth' axes x1y1\n");
		fflush(gnuplot_pipeMP);
		dynamic_plot_state = abs(dynamic_plot_state) - 1;
//		fprintf(gnuplot_pipeMP, "plot "
////				"'./gnuplot_data_lane_.txt' using 1:2 w l title 'Lane Ahead normal'"
////				", './gnuplot_data_lane2_.txt' using 1:2 w l title 'Lane Back normal'"
//				"'./gnuplot_data_lane.txt' using 1:2 w l title 'Lane Ahead smooth'"
//				", './gnuplot_data_lane2.txt' using 1:2 w l title 'Lane Back smooth' axes x1y1\n");
//		fflush(gnuplot_pipeMP);
	}
}


int
pose_out_of_map_coordinates(carmen_point_t pose, carmen_map_p map)
{
	double x_min = map->config.x_origin;
	double x_max = map->config.x_origin + map->config.x_size * map->config.resolution;
	double y_min = map->config.y_origin;
	double y_max = map->config.y_origin + map->config.y_size * map->config.resolution;
	int out_of_map = (pose.x < x_min || pose.x >= x_max || pose.y < y_min || pose.y >= y_max);

	return (out_of_map);
}


double
get_lane_prob(carmen_point_t pose, carmen_map_p road_map)
{
	int x = round((pose.x - road_map->config.x_origin) / road_map->config.resolution);
	int y = round((pose.y - road_map->config.y_origin) / road_map->config.resolution);
	if (x < 0 || x >= road_map->config.x_size || y < 0 || y >= road_map->config.y_size)
		return (-1.0);

	double cell = road_map->map[x][y];
	road_prob *cell_prob = (road_prob *) &cell;
//	double prob = (double) (cell_prob->lane_center - cell_prob->off_road - cell_prob->broken_marking - cell_prob->solid_marking) / MAX_PROB;
	double prob = (double) (cell_prob->lane_center) / MAX_PROB;

	return ((prob > 0.0) ? prob : 0.0);
}


int
get_nearest_lane(carmen_point_p lane_pose, carmen_point_t pose, carmen_map_p road_map)
{
	double dx = 1.0 + (pose.x - road_map->config.x_origin) / road_map->config.resolution;
	double dy = 1.0 + (pose.y - road_map->config.y_origin) / road_map->config.resolution;
	if (dx < road_map->config.x_size / 2.0)
		dx = road_map->config.x_size - dx;
	if (dy < road_map->config.y_size / 2.0)
		dy = road_map->config.y_size - dy;
	double max_radius = sqrt(dx * dx + dy * dy);
	double max_angle = 2.0 * M_PI;
	for (double radius = 1.0; radius <= max_radius; radius += 1.0)
	{
		double delta_angle = 1.0 / radius;
		for (double angle = 0.0; angle < max_angle; angle += delta_angle)
		{
			lane_pose->x = pose.x + radius * cos(angle) * road_map->config.resolution;
			lane_pose->y = pose.y + radius * sin(angle) * road_map->config.resolution;
			if (get_lane_prob(*lane_pose, road_map) >= 0.25) // pose is probably located in a road lane
				return (1);
		}
	}

	return (0);
}


int
find_pose_in_vector(vector<carmen_point_t> poses, carmen_point_t pose, double resolution)
{
	int pose_x = round(pose.x / resolution);
	int pose_y = round(pose.y / resolution);
	for (size_t i = 0; i < poses.size(); i++)
	{
		int x = round(poses[i].x / resolution);
		int y = round(poses[i].y / resolution);
		if (x == pose_x && y == pose_y)
			return (i);
	}

	return (-1);
}


int
distance_within_limits(carmen_point_p pose1, carmen_point_p pose2, double dist_min, double dist_max)
{
	if (dist_min == 0 && dist_max == 0)	// dist_max == 0 corresponds to MAX_VALUE
		return (1);
	if (pose1 == NULL || pose2 == NULL)
		return (0);

	double distance = DIST2D(*pose1, *pose2);
	int within_limits = ((distance >= dist_min) && (distance <= dist_max || dist_max == 0));

	return (within_limits);
}


double
get_center_of_mass(carmen_point_p central_pose, carmen_map_p road_map, int kernel_size, double weight(carmen_point_t, carmen_map_p),
		carmen_point_p skip_pose = NULL, double dist_min = 0.0, double dist_max = 0.0)
{
	carmen_point_t pose;
	double sum_wx = 0.0, sum_wy = 0.0, sum_w = 0.0;
	int count = 0;

	pose.x = central_pose->x - floor(kernel_size / 2) * road_map->config.resolution;
	for (int x = 0; x < kernel_size; x++, pose.x += road_map->config.resolution)
	{
		pose.y = central_pose->y - floor(kernel_size / 2) * road_map->config.resolution;
		for (int y = 0; y < kernel_size; y++, pose.y += road_map->config.resolution)
		{
			if (distance_within_limits(&pose, skip_pose, dist_min, dist_max))
			{
				double pose_weight = weight(pose, road_map);
				if (pose_weight >= 0.0)
				{
					sum_wx += pose_weight * pose.x;
					sum_wy += pose_weight * pose.y;
					sum_w  += pose_weight;
					count++;
				}
			}
		}
	}
	if (sum_w == 0.0)
		return (0.0);

	central_pose->x = sum_wx / sum_w;
	central_pose->y = sum_wy / sum_w;
	double mean_w = sum_w / count;

	return (mean_w);
}


carmen_point_t
get_pose_with_max_lane_prob(vector<carmen_point_t> poses, carmen_map_p road_map)
{
	carmen_point_t pose;
	double max_prob = -1.0;

	for (size_t i = 0; i < poses.size(); i++)
	{
		double lane_prob = get_lane_prob(poses[i], road_map);
		if (lane_prob > max_prob)
		{
			pose = poses[i];
			max_prob = lane_prob;
		}
	}

	return (pose);
}


double
get_orthogonal_angle(double x1, double y1, double x2, double y2)
{
	double orthogonal_angle = atan2(x1 - x2, y2 - y1);

	return (orthogonal_angle);
}


double
mean_angle(double angle1, double angle2)
{
	double mean = atan2((sin(angle1) + sin(angle2)) / 2, (cos(angle1) + cos(angle2)) / 2);

	return (mean);
}


carmen_point_t
add_distance_to_pose(carmen_point_t pose, double distance)
{
	carmen_point_t next_pose = pose;
	next_pose.x += distance * cos(pose.theta);
	next_pose.y += distance * sin(pose.theta);

	return (next_pose);
}


carmen_point_t
add_orthogonal_distance_to_pose(carmen_point_t pose, double distance)
{
	carmen_point_t next_pose = pose;
	double orthogonal_theta;
	if (distance >= 0.0)
		orthogonal_theta = carmen_normalize_theta(pose.theta + (M_PI / 2.0));
	else
		orthogonal_theta = carmen_normalize_theta(pose.theta - (M_PI / 2.0));
	next_pose.x += fabs(distance) * cos(orthogonal_theta);
	next_pose.y += fabs(distance) * sin(orthogonal_theta);

	return (next_pose);
}


int
lane_prob_cmp(double left_prob, double right_prob)
{
	double max_prob_diff = 0.04;
	double prob_diff = left_prob - right_prob;
	int cmp = (prob_diff < -max_prob_diff) ? -1 : (prob_diff > max_prob_diff) ? 1 : 0;

	return (cmp);
}


//FILE *rddf_log = NULL;
//int g_num_pose;


double
get_cubic_bezier_reference(double coord1 , double coord2 , double step_fraction)
{
    double ref = coord1 + ((coord2 - coord1) * step_fraction);

    return (ref);
}

// https://stackoverflow.com/questions/785097/how-do-i-implement-a-b%C3%A9zier-curve-in-c
// https://stackoverflow.com/questions/37642168/how-to-convert-quadratic-bezier-curve-code-into-cubic-bezier-curve

carmen_point_t
get_cubic_bezier(double step_fraction, carmen_point_t p1, carmen_point_t p2, carmen_point_t p3, carmen_point_t p4)
{
	// step_fraction ranges from 0.0 (returns the starting point p1) to 1.0 (returns the ending point p4).
	// The cubic Bezier curve is tangent to the segment p1-p2 at point p1 and to the segment p3-p4 at point p4.

	carmen_point_t cubic_bezier_point, green_a, green_b, green_c, blue_m, blue_n;

	green_a.x = get_cubic_bezier_reference(p1.x, p2.x, step_fraction);
	green_a.y = get_cubic_bezier_reference(p1.y, p2.y, step_fraction);
	green_b.x = get_cubic_bezier_reference(p2.x, p3.x, step_fraction);
	green_b.y = get_cubic_bezier_reference(p2.y, p3.y, step_fraction);
	green_c.x = get_cubic_bezier_reference(p3.x, p4.x, step_fraction);
	green_c.y = get_cubic_bezier_reference(p3.y, p4.y, step_fraction);

	blue_m.x = get_cubic_bezier_reference(green_a.x, green_b.x, step_fraction);
	blue_m.y = get_cubic_bezier_reference(green_a.y, green_b.y, step_fraction);
	blue_n.x = get_cubic_bezier_reference(green_b.x, green_c.x, step_fraction);
	blue_n.y = get_cubic_bezier_reference(green_b.y, green_c.y, step_fraction);

	cubic_bezier_point.x = get_cubic_bezier_reference(blue_m.x, blue_n.x, step_fraction);
	cubic_bezier_point.y = get_cubic_bezier_reference(blue_m.y, blue_n.y, step_fraction);

	return (cubic_bezier_point);
}


carmen_point_t
find_pose_by_annotation(carmen_point_t previous_pose, bool ahead)
{
	carmen_point_t rddf_pose = previous_pose;
	double delta_theta = 2.0 * asin(0.5 * rddf_min_distance_between_waypoints * traffic_sign_curvature);

	if (ahead)
	{
		rddf_pose.theta += delta_theta;
		rddf_pose = add_distance_to_pose(rddf_pose, rddf_min_distance_between_waypoints);
	}
	else
	{
		rddf_pose.theta -= delta_theta;
		rddf_pose = add_distance_to_pose(rddf_pose, -rddf_min_distance_between_waypoints);
	}

	return (rddf_pose);
}


carmen_point_t
find_nearest_pose_by_road_map(carmen_point_t rddf_pose_candidate, carmen_map_p road_map)
{
	carmen_point_t rddf_pose = rddf_pose_candidate; // Keep candidate's theta
	double max_lane_prob = get_lane_prob(rddf_pose_candidate, road_map);
	double min_delta_pose = 0.0;

	double step = road_map->config.resolution / 4.0;
	double lane_expected_width = 1.0;
	double left_limit = lane_expected_width / 2.0;
	double right_limit = -lane_expected_width / 2.0;

	for (double delta_pose = right_limit; delta_pose <= left_limit; delta_pose += step)
 	{
		carmen_point_t lane_pose = add_orthogonal_distance_to_pose(rddf_pose_candidate, delta_pose);
 		double lane_prob = get_lane_prob(lane_pose, road_map);
 		if (lane_prob > max_lane_prob ||
 			(lane_prob == max_lane_prob && fabs(delta_pose) < min_delta_pose))
 		{
 			max_lane_prob = lane_prob;
 			min_delta_pose = fabs(delta_pose);
 			rddf_pose.x = lane_pose.x;
 			rddf_pose.y = lane_pose.y;
 		}
 	}

 	return (rddf_pose);
}


double
average_theta(carmen_ackerman_traj_point_t *poses, int curr_index, int num_poses_avg)
{
	double sum_theta_x = 0.0, sum_theta_y = 0.0;
	int num_poses = ((curr_index + 1) >= num_poses_avg) ? num_poses_avg : (curr_index + 1);

	for (int i = 0, index = curr_index; i < num_poses; i++, index--)
	{
		sum_theta_x += cos(poses[index].theta);
		sum_theta_y += sin(poses[index].theta);
	}

	if (sum_theta_x == 0.0 && sum_theta_y == 0.0)
		return (0.0);

	double avg_theta = atan2(sum_theta_y, sum_theta_x);

	return (avg_theta);
}


void
save_traffic_sign_state()
{
	state_traffic_sign_is_on = traffic_sign_is_on;
	state_traffic_sign_code = traffic_sign_code;
	state_traffic_sign_curvature = traffic_sign_curvature;
}


void
restore_traffic_sign_state()
{
	traffic_sign_is_on = state_traffic_sign_is_on;
	traffic_sign_code = state_traffic_sign_code;
	traffic_sign_curvature = state_traffic_sign_curvature;
}


void
check_reset_traffic_sign_state(carmen_point_t new_pose)
{
	if (DIST2D(new_pose, state_traffic_sign_pose) > 5.0)
	{
		state_traffic_sign_is_on = traffic_sign_is_on = false;
		state_traffic_sign_code = traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
		state_traffic_sign_curvature = traffic_sign_curvature = 0.0;
	}
	state_traffic_sign_pose = new_pose;
}


int
fill_in_poses_ahead_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_ahead, int num_poses_desired)
{
	carmen_point_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	double velocity = 10.0, phi = 0.0;
	int num_poses_avg = 5;

//	rddf_log = fopen("/home/rcarneiro/carmen_lcad/src/rddf/log.txt", "w");
//	g_num_pose = 0;

	find_direction_traffic_sign(initial_pose, true);
	save_traffic_sign_state();
	if (traffic_sign_is_on)
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = find_pose_by_annotation(previous_pose, true);
	}
	else
	{
		previous_pose = rddf_pose = find_nearest_pose_by_road_map(initial_pose, road_map);
		rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
	}

	poses_ahead[0] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
	num_poses = 1;
	do
	{
//		g_num_pose = num_poses;
		find_direction_traffic_sign(rddf_pose_candidate, true);
		if (traffic_sign_is_on)
		{
			previous_pose = rddf_pose = rddf_pose_candidate;
			rddf_pose_candidate = find_pose_by_annotation(previous_pose, true);
		}
		else
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map);
			rddf_pose.theta = atan2(rddf_pose.y - previous_pose.y, rddf_pose.x - previous_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_ahead, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, rddf_min_distance_between_waypoints);
		}
		if (pose_out_of_map_coordinates(rddf_pose, road_map))
			break;

		poses_ahead[num_poses] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
		num_poses++;
	} while (num_poses < num_poses_desired);

//	for (int i = 0; i < num_poses; i++)
//		fprintf(rddf_log, "%d\t%lf\t%lf\t%lf\n", i, poses_ahead[i].x, poses_ahead[i].y, poses_ahead[i].theta);

	calculate_theta_ahead(poses_ahead, num_poses);
	calculate_phi_ahead(poses_ahead, num_poses);
	restore_traffic_sign_state();

	return (num_poses);
}


int
fill_in_poses_back_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_back, int num_poses_desired)
{
	carmen_point_t rddf_pose, previous_pose, rddf_pose_candidate;
	int num_poses = 0;
	double velocity = 10.0, phi = 0.0;
	int num_poses_avg = 5;

	find_direction_traffic_sign(initial_pose, false);
	if (traffic_sign_is_on)
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = find_pose_by_annotation(previous_pose, false);
	}
	else
	{
		previous_pose = rddf_pose = initial_pose;
		rddf_pose_candidate = add_distance_to_pose(previous_pose, -rddf_min_distance_between_waypoints);
	}

	poses_back[0] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
	num_poses = 1;
	do
	{
//		g_num_pose = num_poses;
		find_direction_traffic_sign(rddf_pose_candidate, false);
		if (traffic_sign_is_on)
		{
			previous_pose = rddf_pose = rddf_pose_candidate;
			rddf_pose_candidate = find_pose_by_annotation(previous_pose, false);
		}
		else
		{
			rddf_pose = find_nearest_pose_by_road_map(rddf_pose_candidate, road_map);
			rddf_pose.theta = atan2(previous_pose.y - rddf_pose.y, previous_pose.x - rddf_pose.x);
			previous_pose = rddf_pose;
			previous_pose.theta = average_theta(poses_back, num_poses, num_poses_avg);
			rddf_pose_candidate = add_distance_to_pose(previous_pose, -rddf_min_distance_between_waypoints);
		}
		if (pose_out_of_map_coordinates(rddf_pose, road_map))
			break;

		poses_back[num_poses] = create_ackerman_traj_point_struct(rddf_pose.x, rddf_pose.y, velocity, phi, rddf_pose.theta);
		num_poses++;
	} while (num_poses < num_poses_desired);

//	for (int i = 0; i < num_poses; i++)
//		fprintf(rddf_log, "%d\t%lf\t%lf\t%lf\n", i, poses_back[i].x, poses_back[i].y, poses_back[i].theta);
//	fclose(rddf_log);

	calculate_theta_back(poses_back, num_poses);
	calculate_phi_back(poses_back, num_poses);
	restore_traffic_sign_state();

	return (num_poses);
}


int
find_aligned_pose_index(carmen_point_t initial_pose, carmen_ackerman_traj_point_t *poses_ahead, int num_poses_ahead)
{
	int nearest_pose_index = 0;
	double min_distance = DIST2D(initial_pose, poses_ahead[0]);
	for (int i = 0; i < num_poses_ahead; i++)
	{
		double distance = DIST2D(initial_pose, poses_ahead[i]);
		if (distance < min_distance)
		{
			min_distance = distance;
			nearest_pose_index = i;
		}
	}

	return (nearest_pose_index);
}


int
carmen_rddf_play_find_nearest_poses_by_road_map(carmen_point_t initial_pose, carmen_map_p road_map,
		carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_ahead_max)
{
	static int num_poses_ahead;

	num_poses_ahead = fill_in_poses_ahead_by_road_map(initial_pose, road_map, poses_ahead, num_poses_ahead_max);

	initial_pose.x = poses_ahead[0].x;
	initial_pose.y = poses_ahead[0].y;
	initial_pose.theta = poses_ahead[0].theta;
	(*num_poses_back) = fill_in_poses_back_by_road_map(initial_pose, road_map, poses_back, num_poses_ahead_max / 3);
	poses_back[0].phi = poses_ahead[0].phi;

	if (dynamic_plot_state)
		plot_state(poses_ahead, num_poses_ahead, poses_back, *num_poses_back, false);

	smooth_rddf_using_conjugate_gradient(poses_ahead, num_poses_ahead, poses_back, *num_poses_back);

	if (dynamic_plot_state)
		plot_state(poses_ahead, num_poses_ahead, poses_back, *num_poses_back, true);

	return (num_poses_ahead);
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_publish_annotation_queue()
{
	IPC_RETURN_TYPE err;

	if (annotations_to_publish.size() == 0)
	{
		annotation_and_index annotation_i;
		memset(&annotation_i, 0, sizeof(annotation_and_index));
		carmen_annotation_t &annotation = annotation_i.annotation;
		annotation.annotation_type = RDDF_ANNOTATION_TYPE_NONE;
		annotation.annotation_code = RDDF_ANNOTATION_CODE_NONE;
		annotations_to_publish.push_back(annotation_i);
	}

	if (annotation_queue_message.annotations == NULL)
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) calloc(annotations_to_publish.size(), sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::1\n"));
	}
	else if (annotation_queue_message.num_annotations != (int) annotations_to_publish.size())
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) realloc(annotation_queue_message.annotations, annotations_to_publish.size() * sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::2\n"));
	}

	annotation_queue_message.num_annotations = annotations_to_publish.size();

	for (size_t i = 0; i < annotations_to_publish.size(); i++)
		memcpy(&(annotation_queue_message.annotations[i]), &(annotations_to_publish[i].annotation), sizeof(carmen_annotation_t));

	annotation_queue_message.host = carmen_get_host();
	annotation_queue_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &annotation_queue_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
}


static void
carmen_rddf_play_publish_rddf_and_annotations(carmen_point_t robot_pose)
{
	// so publica rddfs quando a pose do robo ja estiver setada
	if ((carmen_rddf_num_poses_ahead > 0) && (carmen_rddf_num_poses_back > 0))
	{
		clear_annotations();
		set_annotations(robot_pose);

		carmen_rddf_publish_road_profile_message(
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead,
			carmen_rddf_num_poses_back,
			annotations,
			annotations_codes);

		carmen_rddf_play_publish_annotation_queue();

		carmen_rddf_publish_traffic_sign_message(state_traffic_sign_code, state_traffic_sign_curvature);
	}
}


void
carmen_rddf_publish_road_profile_around_end_point(carmen_ackerman_traj_point_t *poses_around_end_point, int num_poses_acquired)
{
	carmen_rddf_publish_road_profile_around_end_point_message(poses_around_end_point, num_poses_acquired);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_find_and_publish_poses_around_end_point(double x, double y, double yaw, int num_poses_desired, double timestamp)
{
	int num_poses_acquired = 0;
	carmen_ackerman_traj_point_t *poses_around_end_point;

	poses_around_end_point = (carmen_ackerman_traj_point_t *) calloc (num_poses_desired, sizeof(carmen_ackerman_traj_point_t));
	carmen_test_alloc(poses_around_end_point);

	num_poses_acquired = carmen_find_poses_around(x, y, yaw, timestamp, poses_around_end_point, num_poses_desired);
	carmen_rddf_publish_road_profile_around_end_point(poses_around_end_point, num_poses_acquired);

	free(poses_around_end_point);
}


static void
carmen_rddf_play_load_index(char *rddf_filename)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	if (!carmen_rddf_index_exists(rddf_filename))
	{
		if (strcmp(rddf_filename + (strlen(rddf_filename) - 3), "kml") == 0)
		{
			carmen_rddf_play_open_kml(rddf_filename, &placemark_vector);

			for (unsigned int i = 0; i < placemark_vector.size(); i++)
			{
				if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
					carmen_rddf_index_add(&message, 0, 0, annotation);
			}

			carmen_rddf_index_save(rddf_filename);
		}
		else
		{
			FILE *fptr = fopen(rddf_filename, "r");

			while (!feof(fptr))
			{
				memset(&message, 0, sizeof(message));

				fscanf(fptr, "%lf %lf %lf %lf %lf %lf\n",
					&(message.pose.position.x), &(message.pose.position.y),
					&(message.pose.orientation.yaw), &(message.velocity.x), &(message.phi),
					&(message.timestamp));

				carmen_rddf_index_add(&message, 0, 0, 0);
			}

			carmen_rddf_index_save(rddf_filename);
			fclose(fptr);
		}
	}

	carmen_rddf_load_index(rddf_filename);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
pos_message_handler(carmen_point_t robot_pose, double timestamp)
{
	if (use_road_map)
	{
		robot_pose_queued = (current_road_map == NULL || pose_out_of_map_coordinates(robot_pose, current_road_map));
		if (robot_pose_queued)
			return;
		check_reset_traffic_sign_state(robot_pose);
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_by_road_map(
				robot_pose,
				current_road_map,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max);
	}
	else
	{
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_ahead(
				robot_pose.x,
				robot_pose.y,
				robot_pose.theta,
				timestamp,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max,
				annotations);
	}

	annotations_to_publish.clear();
	carmen_check_for_annotations(robot_pose, carmen_rddf_poses_ahead, carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead, carmen_rddf_num_poses_back, timestamp);

	carmen_rddf_play_publish_rddf_and_annotations(robot_pose);
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_rddf_pose_initialized = 1;
	current_globalpos_msg = msg;
	pos_message_handler(msg->globalpos, msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	carmen_rddf_pose_initialized = 1;
	current_truepos_msg = msg;
	pos_message_handler(msg->truepose, msg->timestamp);
}


static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static bool first_time = true;

	if (first_time)
	{
		current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
		carmen_grid_mapping_initialize_map(current_road_map, msg->config.x_size, msg->config.resolution, 'r');
		first_time = false;
	}

	if (msg->config.x_origin != current_road_map->config.x_origin || msg->config.y_origin != current_road_map->config.y_origin) // new map
	{
		memcpy(current_road_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
		current_road_map->config = msg->config;

		if (robot_pose_queued)
		{
			if (use_truepos)
				simulator_ackerman_truepos_message_handler(current_truepos_msg);
			else
				localize_globalpos_handler(current_globalpos_msg);
		}
	}
}


//static void
//carmen_rddf_play_nearest_waypoint_message_handler(carmen_rddf_nearest_waypoint_message *rddf_nearest_waypoint_message)
//{
//	carmen_rddf_nearest_waypoint_to_end_point = rddf_nearest_waypoint_message->point;
//	carmen_rddf_nearest_waypoint_is_set = 1;
//
//	carmen_rddf_publish_nearest_waypoint_confirmation_message(rddf_nearest_waypoint_message->point);
//	already_reached_nearest_waypoint_to_end_point = 0;
//}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	if (rddf_end_point_message->number_of_poses > 1)
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_end_point = rddf_end_point_message->point;
		carmen_rddf_end_point_is_set = 1;
	}
	else
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_nearest_waypoint_to_end_point = rddf_end_point_message->point;
		carmen_rddf_nearest_waypoint_is_set = 1;

		already_reached_nearest_waypoint_to_end_point = 0;
	}
}


static void
carmen_traffic_light_message_handler(carmen_traffic_light_message *message)
{
	traffic_lights = message;
}


static void
carmen_rddf_dynamic_annotation_message_handler(carmen_rddf_dynamic_annotation_message *message)
{
	// Avoid reinserting the same annotation over and over.
	for (size_t i = 0; i < dynamic_annotation_messages.size(); i++)
	{
		carmen_rddf_dynamic_annotation_message &stored = dynamic_annotation_messages[i];
		if (stored.annotation_type == message->annotation_type &&
			stored.annotation_code == message->annotation_code &&
			DIST2D(stored.annotation_point, message->annotation_point) < 0.5)
		{
			stored = *message;
			return;
		}
	}

	dynamic_annotation_messages.push_front(*message);
	if (dynamic_annotation_messages.size() > 30)
		dynamic_annotation_messages.pop_back();
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	moving_objects = moving_objects_point_clouds_message;
}


void
carmen_voice_interface_command_message_handler(carmen_voice_interface_command_message *message)
{
	if (message->command_id == SET_COURSE)
	{
		printf("New rddf set by voice command: %s\n", message->command);
		carmen_rddf_index_clear();
		char *carmen_home = getenv("CARMEN_HOME");
		static char rddf_file_name[2048];
		strcpy(rddf_file_name, carmen_home);
		strcat(rddf_file_name, "/");
		strcat(rddf_file_name, message->command);

		carmen_rddf_play_load_index(rddf_file_name);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (use_road_map)
		carmen_map_server_subscribe_road_map(NULL, (carmen_handler_t) road_map_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_rddf_subscribe_nearest_waypoint_message(NULL,
//			(carmen_handler_t) carmen_rddf_play_nearest_waypoint_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL,
			(carmen_handler_t) carmen_rddf_play_end_point_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

    carmen_traffic_light_subscribe(traffic_lights_camera, NULL, (carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_dynamic_annotation_message(NULL, (carmen_handler_t) carmen_rddf_dynamic_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_voice_interface_subscribe_command_message(NULL, (carmen_handler_t) carmen_voice_interface_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
carmen_rddf_play_load_annotation_file()
{
	if (carmen_annotation_filename == NULL)
		return;

	FILE *f = fopen(carmen_annotation_filename, "r");
	char line[1024];
	while(fgets(line, 1023, f) != NULL)
	{
		if (line[0] == '#') // comment line
			continue;

		carmen_annotation_t annotation;
		char annotation_description[1024];
		if (sscanf(line, "%s %d %d %lf %lf %lf %lf\n",
				annotation_description,
				&annotation.annotation_type,
				&annotation.annotation_code,
				&annotation.annotation_orientation,
				&annotation.annotation_point.x,
				&annotation.annotation_point.y,
				&annotation.annotation_point.z) == 7)
		{
			annotation.annotation_description = (char *) calloc (1024, sizeof(char));
			strcpy(annotation.annotation_description, annotation_description);
//			printf("%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n", annotation.annotation_description,
//					annotation.annotation_type,
//					annotation.annotation_code,
//					annotation.annotation_orientation,
//					annotation.annotation_point.x,
//					annotation.annotation_point.y,
//					annotation.annotation_point.z);

			/*
			 *	The annotation file's points (x,y) are placed at the front of the car
			 *	The annotation vector's points (x,y) are placed at the car's rear axle
			 *	The annotation orientation is the angle of the rddf orientation in radians
			 *	The value of annotation point z may have different meanings for different annotation types
			 *	For PEDESTRIAN_TRACK type z is the search radius for pedestrians in meters
			 *	For TRAFFIC_SIGN type z is the curvature of the rddf in radians/meter
			 */
			carmen_ackerman_traj_point_t annotation_point;
			annotation_point.x = annotation.annotation_point.x;
			annotation_point.y = annotation.annotation_point.y;
			annotation_point.theta = annotation.annotation_orientation;
			double distance_car_pose_car_front = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
			carmen_point_t new_annotation_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&annotation_point, -distance_car_pose_car_front);

			annotation.annotation_point.x = new_annotation_point.x;
			annotation.annotation_point.y = new_annotation_point.y;
			annotation_read_from_file.push_back(annotation);
		}
	}

	fclose(f);
}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 0, NULL},
		{(char *) "robot", (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &distance_between_front_car_and_front_wheels, 0, NULL},
		{(char *) "robot", (char *) "turning_radius", CARMEN_PARAM_DOUBLE, &turning_radius, 0, NULL},
		{(char *) "rddf", (char *) "num_poses_ahead", CARMEN_PARAM_INT, &carmen_rddf_num_poses_ahead_max, 0, NULL},
		{(char *) "rddf", (char *) "min_distance_between_waypoints", CARMEN_PARAM_DOUBLE, &rddf_min_distance_between_waypoints, 0, NULL},
		{(char *) "rddf", (char *) "loop", CARMEN_PARAM_ONOFF, &carmen_rddf_perform_loop, 0, NULL},
		{(char *) "rddf", (char *) "default_search_radius", CARMEN_PARAM_DOUBLE, &default_search_radius, 0, NULL},
		{(char *) "rddf", (char *) "dynamic_plot_state", CARMEN_PARAM_INT, &dynamic_plot_state, 1, NULL}, /* param_edit para modificar */
		{(char *) "behavior_selector", (char *) "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},
		{(char *) "road_mapper", (char *) "kernel_size", CARMEN_PARAM_INT, &road_mapper_kernel_size, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	if (turning_radius != 0.0)
		maximum_curvature = 1.0 / turning_radius;
}


void
carmen_rddf_play_initialize(void)
{
	carmen_rddf_poses_ahead = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	carmen_rddf_poses_back = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	annotations = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	annotations_codes = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	memset(&annotation_queue_message, 0, sizeof(annotation_queue_message));

	carmen_test_alloc(carmen_rddf_poses_ahead);
	carmen_test_alloc(annotations);

	// publish rddf at a rate of 10Hz
	// carmen_ipc_addPeriodicTimer(1.0 / 10.0, (TIMER_HANDLER_TYPE) carmen_rddf_play_publish_rddf, NULL);
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");
	char *usage[] = {(char *) "<rddf_filename> [<annotation_filename> [<traffic_lights_camera>]]",
			         (char *) "-use_road_map   [<annotation_filename> [<traffic_lights_camera>]]"};

	if (argc >= 2 && strcmp(argv[1], "-h") == 0)
	{
		printf("\nUsage 1: %s %s\nUsage 2: %s %s\n\nCarmen parameters:\n", argv[0], usage[0], argv[0], usage[1]);
		carmen_rddf_play_get_parameters(argc, argv); // display help and exit
	}
	if (argc < 2 || argc > 4)
		exit(printf("Error: Usage 1: %s %s\n       Usage 2: %s %s\n", argv[0], usage[0], argv[0], usage[1]));

	if (strcmp(argv[1], "-use_road_map") == 0)
	{
		use_road_map = true;
		printf("Road map option set.\n");
	}
	else
	{
		carmen_rddf_filename = argv[1];
		printf("RDDF filename: %s.\n", carmen_rddf_filename);
	}

	if (argc >= 3)
		carmen_annotation_filename = argv[2];
	if (carmen_annotation_filename)
		printf("Annotation filename: %s.\n", carmen_annotation_filename);

	if (argc >= 4)
		traffic_lights_camera = atoi(argv[3]);
	printf("Traffic lights camera: %d.\n", traffic_lights_camera);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_rddf_play_get_parameters(argc, argv);
	carmen_rddf_play_initialize();
	carmen_rddf_define_messages();
	carmen_rddf_play_subscribe_messages();
	if (!use_road_map)
		carmen_rddf_play_load_index(carmen_rddf_filename);
	carmen_rddf_play_load_annotation_file();
	signal (SIGINT, carmen_rddf_play_shutdown_module);
	carmen_ipc_dispatch();

	return (0);
}
