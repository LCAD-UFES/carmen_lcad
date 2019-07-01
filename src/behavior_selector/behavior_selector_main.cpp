#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>
#include <carmen/path_planner_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>
#include <carmen/grid_mapping.h>
#include <carmen/udatmo.h>
#include <carmen/udatmo_messages.h>
#include <prob_map.h>
#include <carmen/map_server_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/voice_interface_messages.h>
#include <carmen/voice_interface_interface.h>
#include <carmen/collision_detection.h>

#include "behavior_selector.h"
#include "behavior_selector_messages.h"

// Comment or uncomment this definition to control whether simulated moving obstacles are created.
//#define SIMULATE_MOVING_OBSTACLE
#define SIMULATE_LATERAL_MOVING_OBSTACLE

// Comment or uncomment this definition to control whether moving obstacles are displayed.
#define DISPLAY_MOVING_OBSTACLES

static int necessary_maps_available = 0;
static bool obstacle_avoider_active_recently = false;
static int activate_tracking = 0;


double param_distance_between_waypoints;
double param_change_goal_distance;
double param_distance_interval;

double map_width;

carmen_obstacle_avoider_robot_will_hit_obstacle_message last_obstacle_avoider_robot_hit_obstacle_message;
carmen_rddf_annotation_message last_rddf_annotation_message;
bool last_rddf_annotation_message_valid = false;
carmen_behavior_selector_goal_source_t last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
carmen_behavior_selector_goal_source_t goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static int param_goal_source_onoff = 0;

int param_rddf_num_poses_ahead_limited_by_map;
int param_rddf_num_poses_ahead_min;
int param_rddf_num_poses_by_car_velocity = 1;
int carmen_rddf_num_poses_ahead = 100;

carmen_behavior_selector_road_profile_message road_profile_message;
double robot_max_centripetal_acceleration = 1.5;

int use_truepos = 0;
extern carmen_mapper_virtual_laser_message virtual_laser_message;

static carmen_rddf_road_profile_message *last_rddf_message = NULL;
static carmen_rddf_road_profile_message *last_rddf_message_copy = NULL;

bool autonomous = false;
bool wait_start_moving = false;
static double last_not_autonomous_timestamp = 0.0;

carmen_behavior_selector_state_message behavior_selector_state_message;

static carmen_obstacle_distance_mapper_map_message distance_map;
static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
static carmen_obstacle_distance_mapper_compact_map_message *compact_lane_contents = NULL;

static double original_behaviour_selector_central_lane_obstacles_safe_distance;
static double original_model_predictive_planner_obstacles_safe_distance;

static double carmen_ini_max_velocity;

static bool soft_stop_on = false;


int
compute_max_rddf_num_poses_ahead(carmen_ackerman_traj_point_t current_pose)
{
	int num_poses_ahead_by_velocity = param_rddf_num_poses_ahead_min;
	//	s = vf*vf - v0*v0 / 2*a;
	double common_goal_v = 3.0;
	double distance = 0.0;

	if (common_goal_v < current_pose.v)
	{
		distance = current_pose.v * 6.5;
		if (distance > 0)
			num_poses_ahead_by_velocity = (distance / 0.5) + 1;
	}
	if (num_poses_ahead_by_velocity < param_rddf_num_poses_ahead_min)
		num_poses_ahead_by_velocity = param_rddf_num_poses_ahead_min;
	else if (num_poses_ahead_by_velocity > param_rddf_num_poses_ahead_limited_by_map)
		num_poses_ahead_by_velocity = param_rddf_num_poses_ahead_limited_by_map;

//	printf("\n current_v: %lf distance: %lf a: %lf num_poses: %d \n", current_pose.v, distance, a, num_poses_ahead_by_velocity);
	return num_poses_ahead_by_velocity;
}


static carmen_rddf_road_profile_message *
copy_rddf_message_considering_velocity(carmen_rddf_road_profile_message *last_rddf_message, carmen_rddf_road_profile_message *rddf_msg)
{
	//Now the rddf is number of posses variable with the velocity
	if (!last_rddf_message)
	{
		last_rddf_message = (carmen_rddf_road_profile_message *) malloc(sizeof(carmen_rddf_road_profile_message));
		last_rddf_message->number_of_poses = 0;
		last_rddf_message->number_of_poses_back = 0;
		last_rddf_message->poses = NULL;
		last_rddf_message->poses_back = NULL;
		last_rddf_message->annotations = NULL;
		last_rddf_message->annotations_codes = NULL;
	}

	if ((rddf_msg->number_of_poses < param_rddf_num_poses_ahead_min) && (rddf_msg->number_of_poses > 0))
		carmen_rddf_num_poses_ahead = rddf_msg->number_of_poses;
	else
		carmen_rddf_num_poses_ahead = compute_max_rddf_num_poses_ahead(get_robot_pose());

	if (rddf_msg->number_of_poses_back > carmen_rddf_num_poses_ahead)
		last_rddf_message->number_of_poses_back = carmen_rddf_num_poses_ahead;

	last_rddf_message->timestamp = rddf_msg->timestamp;
	last_rddf_message->number_of_poses = carmen_rddf_num_poses_ahead;
	last_rddf_message->number_of_poses_back = rddf_msg->number_of_poses_back;

	last_rddf_message->poses = (carmen_ackerman_traj_point_t *) realloc(last_rddf_message->poses, sizeof(carmen_ackerman_traj_point_t) * carmen_rddf_num_poses_ahead);
	last_rddf_message->poses_back = (carmen_ackerman_traj_point_t *) realloc(last_rddf_message->poses_back, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses_back);
	last_rddf_message->annotations = (int *) realloc(last_rddf_message->annotations, sizeof(int) * carmen_rddf_num_poses_ahead);
	last_rddf_message->annotations_codes = (int *) realloc(last_rddf_message->annotations_codes, sizeof(int) * carmen_rddf_num_poses_ahead);

	memcpy(last_rddf_message->poses, rddf_msg->poses, sizeof(carmen_ackerman_traj_point_t) * carmen_rddf_num_poses_ahead);
	memcpy(last_rddf_message->poses_back, rddf_msg->poses_back, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses_back);
	memcpy(last_rddf_message->annotations, rddf_msg->annotations, sizeof(int) * carmen_rddf_num_poses_ahead);
	memcpy(last_rddf_message->annotations_codes, rddf_msg->annotations_codes, sizeof(int) * carmen_rddf_num_poses_ahead);

	return (last_rddf_message);
}


static carmen_rddf_road_profile_message *
copy_rddf_message(carmen_rddf_road_profile_message *last_rddf_message, carmen_rddf_road_profile_message *rddf_msg)
{
	if (!last_rddf_message)
	{
		last_rddf_message = (carmen_rddf_road_profile_message*) malloc(sizeof(carmen_rddf_road_profile_message));
		last_rddf_message->number_of_poses = 0;
		last_rddf_message->number_of_poses_back = 0;
		last_rddf_message->poses = NULL;
		last_rddf_message->poses_back = NULL;
		last_rddf_message->annotations = NULL;
		last_rddf_message->annotations_codes = NULL;
	}

	last_rddf_message->timestamp = rddf_msg->timestamp;
	last_rddf_message->number_of_poses = rddf_msg->number_of_poses;
	last_rddf_message->number_of_poses_back = rddf_msg->number_of_poses_back;

	last_rddf_message->poses = (carmen_ackerman_traj_point_t *) realloc(last_rddf_message->poses, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses);
	last_rddf_message->poses_back = (carmen_ackerman_traj_point_t *) realloc(last_rddf_message->poses_back, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses_back);
	last_rddf_message->annotations = (int *) realloc(last_rddf_message->annotations, sizeof(int) * last_rddf_message->number_of_poses);
	last_rddf_message->annotations_codes = (int *) realloc(last_rddf_message->annotations_codes, sizeof(int) * last_rddf_message->number_of_poses);

	memcpy(last_rddf_message->poses, rddf_msg->poses, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses);
	memcpy(last_rddf_message->poses_back, rddf_msg->poses_back, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses_back);
	memcpy(last_rddf_message->annotations, rddf_msg->annotations, sizeof(int) * last_rddf_message->number_of_poses);
	memcpy(last_rddf_message->annotations_codes, rddf_msg->annotations_codes, sizeof(int) * last_rddf_message->number_of_poses);

	return (last_rddf_message);
}


carmen_ackerman_traj_point_t
displace_pose(carmen_ackerman_traj_point_t robot_pose, double displacement)
{
	carmen_point_t displaced_robot_position = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&robot_pose, displacement);

	carmen_ackerman_traj_point_t displaced_robot_pose = robot_pose;
	displaced_robot_pose.x = displaced_robot_position.x;
	displaced_robot_pose.y = displaced_robot_position.y;

	return (displaced_robot_pose);
}


carmen_annotation_t *
get_nearest_specified_annotation(int annotation, carmen_rddf_annotation_message annotation_message,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);

		if ((annotation_message.annotations[i].annotation_type == annotation) &&
			(distance_to_annotation < distance_to_nearest_annotation) &&
			carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, annotation_message.annotations[i].annotation_point))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			nearest_annotation_index = i;
		}
	}

	if (nearest_annotation_index != -1)
		return (&(annotation_message.annotations[nearest_annotation_index]));
	else
		return (NULL);
}


bool
busy_pedestrian_track_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_pedestrian_track_busy_timestamp = 0.0;

//	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
//				&current_robot_pose_v_and_phi, false);
//
//	if (nearest_velocity_related_annotation == NULL)
//		return (false);
//
//	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
//	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_pedestrian_track_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK,
			last_rddf_annotation_message, &displaced_robot_pose);

	if (nearest_pedestrian_track_annotation == NULL)
		return (false);

	if ((nearest_pedestrian_track_annotation->annotation_code == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY))// &&
//		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
//		(distance_to_act_on_annotation >= distance_to_annotation) &&
//		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_pedestrian_track_busy_timestamp = timestamp;

	if (timestamp - last_pedestrian_track_busy_timestamp < 1.5)
		return (true);

	return (false);
}


carmen_annotation_t *
get_nearest_velocity_related_annotation(carmen_rddf_annotation_message annotation_message,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi, bool wait_start_moving)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);
		if (((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) && !wait_start_moving) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) && busy_pedestrian_track_ahead(*current_robot_pose_v_and_phi, carmen_get_time()) && !wait_start_moving) ||
			 ((annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
			  (annotation_message.annotations[i].annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP) && !wait_start_moving) ||
			 (annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BUMP)) &&
			carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, annotation_message.annotations[i].annotation_point) &&
			 (distance_to_annotation < distance_to_nearest_annotation))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			nearest_annotation_index = i;
		}
	}

	if (nearest_annotation_index != -1)
		return (&(annotation_message.annotations[nearest_annotation_index]));
	else
		return (NULL);
}


double
get_distance_to_act_on_annotation(double v0, double va, double distance_to_annotation)
{
	// @@@ Alberto: rever pois esta fazendo a IARA lesmar quando esta lenta e longe de uma anotacao
	// va = v0 + a * t
	// a*t = v - v0
	// t = (va - v0) / a
	// da = va * t + 0.5 * a * t * t

	double a = -get_robot_config()->maximum_acceleration_forward * 1.1;
	double t = (va - v0) / a;
	double daa = v0 * t + 0.5 * a * t * t;

	//printf("t %.2lf, v0 %.1lf, va %.1lf, a %.2lf, tt %.2lf, daa %.1lf, da %.1lf\n", carmen_get_time(), v0, va, a, t, daa, distance_to_annotation);
	if (daa > distance_to_annotation)
		return (daa);// + 3.0 * get_robot_config()->distance_between_front_and_rear_axles);
	else
		return (distance_to_annotation);
}


bool
red_traffic_light_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static double last_red_timestamp = 0.0;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	carmen_annotation_t *nearest_traffic_light_annotation = get_nearest_specified_annotation(RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT,
			last_rddf_annotation_message, &displaced_robot_pose);

	if (nearest_traffic_light_annotation == NULL)
		return (false);

	if (nearest_traffic_light_annotation->annotation_code == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN)
		return (false);
	else if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
		(distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		last_red_timestamp = timestamp;

	if (timestamp - last_red_timestamp < 3.0)
		return (true);

	return (false);
}


double
get_velocity_at_next_annotation(carmen_annotation_t *annotation, carmen_ackerman_traj_point_t current_robot_pose_v_and_phi,
		double timestamp)
{
	double v = 60.0 / 3.6;

	if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
		red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) &&
			 busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
		v = 0.08;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP)
		v = 0.08;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP))
		v = 0.09;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BUMP)
		v = 2.5;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
		v = 2.5;
	else if (annotation->annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)
		v = 2.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_0))
		v = 0.0;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_5))
		v = 5.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_10))
		v = 10.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_15))
		v = 15.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_20))
		v = 20.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_30))
		v = 30.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_40))
		v = 40.0 / 3.6;
	else if ((annotation->annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			 (annotation->annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_60))
		v = 60.0 / 3.6;

	return (v);
}


double
distance_to_moving_obstacle_annotation(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_DYNAMIC) &&
		(nearest_velocity_related_annotation->annotation_code == RDDF_ANNOTATION_CODE_DYNAMIC_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
get_velocity_at_goal(double v0, double va, double dg, double da)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dg*(g-v)%2Fa%2B(v-g)*((g-v)%2F(2*a)))+for+a
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*((g-v)%2Fa)%2B0.5*a*((g-v)%2Fa)%5E2+for+g
	// http://www.physicsclassroom.com/class/1DKin/Lesson-6/Kinematic-Equations

//	double a = -get_robot_config()->maximum_acceleration_forward * 2.5;
	double a = (va * va - v0 * v0) / (2.0 * da);
	// TODO: @@@ Alberto: nao deveria ser 2.0 ao inves de 1.0 abaixo? Com 2.0 freia esponencialmente nos quebra molas...
	double sqrt_val = 1.8 * a * dg + v0 * v0;
	double vg = va;
	if (sqrt_val > 0.0)
		vg = sqrt(sqrt_val);
	if (vg < va)
		vg = va;

//	static double first_time = 0.0;
//	double t = carmen_get_time();
//	if (first_time == 0.0)
//		first_time = t;
	//printf("t %.3lf, v0 %.1lf, va %.1lf, a %.3lf, vg %.2lf, dg %.1lf, da %.1lf\n", t - first_time, v0, va, a, vg, dg, da);
//	printf("t %.3lf, v0 %.1lf, a %.3lf, vg %.2lf, dg %.1lf, tt %.3lf\n", t - first_time, v0, a, vg, dg, (vg - v0) / a);

	return (vg);
}


double
compute_distance_within_rddf(carmen_vector_3D_t annotation_point, carmen_ackerman_traj_point_t  current_robot_pose_v_and_phi)
{
	carmen_rddf_road_profile_message *rddf = last_rddf_message;

	double distance_to_annotation = 1000.0;
	double distance_to_car = 1000.0;
	int index_car_pose = 0;
	int index_annotation = 0;
	for (int i = 0; i < rddf->number_of_poses; i++)
	{
		double distance = DIST2D(annotation_point, rddf->poses[i]);
		if (distance < distance_to_annotation)
		{
			distance_to_annotation = distance;
			index_annotation = i;
		}
		distance = DIST2D(current_robot_pose_v_and_phi, rddf->poses[i]);
		if (distance < distance_to_car)
		{
			distance_to_car = distance;
			index_car_pose = i;
		}
	}

	double distance_within_rddf = 0.0;
	for (int i = index_car_pose; i < index_annotation; i++)
		distance_within_rddf += DIST2D(rddf->poses[i], rddf->poses[i + 1]);

	double min_distance = DIST2D(annotation_point, current_robot_pose_v_and_phi);

	return ((distance_within_rddf < min_distance)? min_distance: distance_within_rddf);
}


double
set_goal_velocity_according_to_annotation(carmen_ackerman_traj_point_t *goal, int goal_type,
		carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi, double timestamp)
{
	static bool clearing_annotation = false;
	static carmen_vector_3D_t previous_annotation_point = {0.0, 0.0, 0.0};

	if (!autonomous)
		clearing_annotation = false;

	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
			current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation != NULL)
	{
//		carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(*current_robot_pose_v_and_phi,
//				get_robot_config()->distance_between_front_and_rear_axles +
//				get_robot_config()->distance_between_front_car_and_front_wheels);

		double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi);
//		double distance_to_annotation = compute_distance_within_rddf(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi);
//		FILE *caco13 = fopen("caco13.txt", "a");
//		fprintf(caco13, "%.2lf %.2lf\n", distance_to_annotation, DIST2D(nearest_velocity_related_annotation->annotation_point, *current_robot_pose_v_and_phi));
//		fflush(caco13);
//		fclose(caco13);

		double velocity_at_next_annotation = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi, timestamp);

		double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi->v, velocity_at_next_annotation,
				distance_to_annotation);
		bool annotation_ahead = carmen_rddf_play_annotation_is_forward(*current_robot_pose_v_and_phi, nearest_velocity_related_annotation->annotation_point);

		double distance_to_goal = carmen_distance_ackerman_traj(current_robot_pose_v_and_phi, goal);

		if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) && 	// Reduz o criterio dos obstaculos moveis se for barreira
			(distance_to_annotation < 35.0))
//			((distance_to_annotation - distance_to_goal) < get_robot_config()->distance_between_front_and_rear_axles))
		{
			get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance = 1.233;	// Padrao da Ida a Guarapari
			get_robot_config()->model_predictive_planner_obstacles_safe_distance = 1.233;			// Padrao da Ida a Guarapari
			udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(1.233);
			udatmo_set_model_predictive_planner_obstacles_safe_distance(1.233);
		}
		else
		{
			get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance = original_behaviour_selector_central_lane_obstacles_safe_distance;
			get_robot_config()->model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance;
			udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(original_behaviour_selector_central_lane_obstacles_safe_distance);
			udatmo_set_model_predictive_planner_obstacles_safe_distance(original_model_predictive_planner_obstacles_safe_distance);
		}

		if (last_rddf_annotation_message_valid &&
			(clearing_annotation ||
			 (((distance_to_annotation < distance_to_act_on_annotation) ||
			   (distance_to_annotation < distance_to_goal)) && annotation_ahead) ||
			   ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
				((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL2)))))
		{
			if (!clearing_annotation)
				previous_annotation_point = nearest_velocity_related_annotation->annotation_point;

			clearing_annotation = true;
			goal->v = carmen_fmin(
					get_velocity_at_goal(current_robot_pose_v_and_phi->v, velocity_at_next_annotation, distance_to_goal, distance_to_annotation),
					goal->v);

			if (((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
					((goal_type == ANNOTATION_GOAL2) || (goal_type == ANNOTATION_GOAL2))))
				goal->v = get_velocity_at_next_annotation(nearest_velocity_related_annotation, *current_robot_pose_v_and_phi,
						timestamp);
		}

		if (!annotation_ahead || (DIST2D(previous_annotation_point, nearest_velocity_related_annotation->annotation_point) > 0.0))
			clearing_annotation = false;

//		FILE *caco = fopen("caco4.txt", "a");
//		fprintf(caco, "ca %d, aa %d, daann %.1lf, dann %.1lf, v %.1lf, vg %.1lf, aif %d, dg %.1lf, av %.1lf, ts %lf\n", clearing_annotation, annotation_ahead,
//				distance_to_act_on_annotation, distance_to_annotation, current_robot_pose_v_and_phi->v,
//				goal->v,
//				carmen_rddf_play_annotation_is_forward(get_robot_pose(), nearest_velocity_related_annotation->annotation_point),
//				distance_to_goal, velocity_at_next_annotation, carmen_get_time());
//		fflush(caco);
//		fclose(caco);
	}
	else
	{
		get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance = original_behaviour_selector_central_lane_obstacles_safe_distance;
		get_robot_config()->model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance;
		udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(original_behaviour_selector_central_lane_obstacles_safe_distance);
		udatmo_set_model_predictive_planner_obstacles_safe_distance(original_model_predictive_planner_obstacles_safe_distance);
	}

	return (goal->v);
}


double
set_goal_velocity_according_to_obstacle_distance(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi)
{
	double distance_to_obstacle = DIST2D_P(current_robot_pose_v_and_phi, goal);

	goal->v = carmen_fmin(
				get_velocity_at_goal(current_robot_pose_v_and_phi->v, 0.0, distance_to_obstacle, distance_to_obstacle),
				goal->v);

	return (goal->v);
}


double
limit_maximum_velocity_according_to_centripetal_acceleration(double target_v, double current_v, carmen_ackerman_traj_point_t *goal,
		carmen_ackerman_traj_point_t *path, int number_of_poses)
{
	if (number_of_poses == 0)
		return (target_v);

	double desired_v = 0.0;
	double max_centripetal_acceleration = 0.0;
	double dist_walked = 0.0;
	double dist_to_max_curvature = 0.0;
	double max_path_phi = 0.0;
	double L = get_robot_config()->distance_between_front_and_rear_axles;

	for (int i = 0; i < (number_of_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		dist_walked += l;
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = L * atan(delta_theta / l);
	}

	for (int i = 1; i < (number_of_poses - 1); i++)
	{
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
		if (fabs(path[i].phi) > 0.001)
		{
			double radius_of_curvature = L / fabs(tan(path[i].phi));
			double centripetal_acceleration = (target_v * target_v) / radius_of_curvature;
			if (centripetal_acceleration > max_centripetal_acceleration)
			{
				dist_to_max_curvature = dist_walked;
				max_path_phi = path[i].phi;
				max_centripetal_acceleration = centripetal_acceleration;
			}
		}
	}

	double limited_target_v = target_v;
	if (max_centripetal_acceleration > robot_max_centripetal_acceleration)
	{
		double radius_of_curvature = L / fabs(tan(max_path_phi));
		desired_v = sqrt(robot_max_centripetal_acceleration * radius_of_curvature);
		if (desired_v < target_v)
		{
			carmen_ackerman_traj_point_t current_robot_pose_v_and_phi = get_robot_pose();
			double dist_to_goal = carmen_distance_ackerman_traj(&current_robot_pose_v_and_phi, goal);
			double velocity_at_goal = get_velocity_at_goal(current_v, desired_v, dist_to_goal, dist_to_max_curvature);
			if (velocity_at_goal < target_v)
				limited_target_v = velocity_at_goal;
		}
	}

	return (carmen_fmin(limited_target_v, target_v));
}

extern SampleFilter filter2;


double
set_goal_velocity_according_to_moving_obstacle(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, double timestamp __attribute__ ((unused)))
{
	double car_pose_to_car_front = get_robot_config()->distance_between_front_and_rear_axles + get_robot_config()->distance_between_front_car_and_front_wheels;
	// um carro de tamanho para cada 10 milhas/h (4.4705 m/s) -> ver "The DARPA Urban Challenge" book, pg. 36.
	double min_dist_according_to_car_v = get_robot_config()->length * (current_robot_pose_v_and_phi->v / 4.4704) + car_pose_to_car_front;
	double desired_distance = carmen_fmax(1.4 * min_dist_according_to_car_v, car_pose_to_car_front + 2.5);

	double distance = udatmo_get_moving_obstacle_distance(*current_robot_pose_v_and_phi, get_robot_config());
	double moving_obj_v = udatmo_speed_front();

	// ver "The DARPA Urban Challenge" book, pg. 36.
	double Kgap = 0.1;
	double new_goal_v;
	if (goal->v > moving_obj_v)
		new_goal_v = moving_obj_v + Kgap * (distance - desired_distance);
	else
		new_goal_v = goal->v;
	SampleFilter_put(&filter2, new_goal_v);
	new_goal_v = SampleFilter_get(&filter2);
	if (new_goal_v < 0.0)
		new_goal_v = 0.0;

	if ((goal_type == MOVING_OBSTACLE_GOAL1) || (goal_type == MOVING_OBSTACLE_GOAL2))//udatmo_obstacle_detected(timestamp))// && (current_robot_pose_v_and_phi->v > moving_obj_v))
		goal->v = carmen_fmin(new_goal_v, goal->v);

//	FILE *caco = fopen("caco.txt", "a");
//	fprintf(caco, "%lf %lf %lf %lf %lf %d %d %d %lf %lf %lf %d ", moving_obj_v, goal->v, current_robot_pose_v_and_phi->v, distance,
//			desired_distance, behavior_selector_state_message.low_level_state, autonomous, goal_type,
//			udatmo_speed_left(), udatmo_speed_right(), udatmo_speed_center(), udatmo_obstacle_detected(timestamp));
//	fflush(caco);
//	fclose(caco);

	return (goal->v);
}


void
set_goal_velocity(carmen_ackerman_traj_point_t *goal, carmen_ackerman_traj_point_t *current_robot_pose_v_and_phi,
		int goal_type, double timestamp)
{
	goal->v = get_max_v();
	if (goal_type == OBSTACLE_GOAL)
		goal->v = set_goal_velocity_according_to_obstacle_distance(goal, current_robot_pose_v_and_phi);

//	FILE *caco = fopen("caco3.txt", "a");
//	fprintf(caco, "gv %lf  ", goal->v);

	goal->v = set_goal_velocity_according_to_moving_obstacle(goal, current_robot_pose_v_and_phi, goal_type, timestamp);

//	fprintf(caco, "gva %lf  ", goal->v);
	goal->v = limit_maximum_velocity_according_to_centripetal_acceleration(goal->v, get_robot_pose().v, goal,
			road_profile_message.poses, road_profile_message.number_of_poses);
//	fprintf(caco, "gvdlc %lf  ", goal->v);

	goal->v = set_goal_velocity_according_to_annotation(goal, goal_type, current_robot_pose_v_and_phi, timestamp);
//	fprintf(caco, "gvda %lf ", goal->v);
//	if (obstacle_avoider_active_recently)
//		goal->v = carmen_fmin(2.5, goal->v);

//	fprintf(caco, "gvf %lf ts %lf\n", goal->v, carmen_get_time());
//	fflush(caco);
//	fclose(caco);
}


void
set_behaviours_parameters(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (fabs(current_robot_pose_v_and_phi.v) < param_distance_interval)
		change_distance_between_waypoints_and_goals(param_distance_between_waypoints, param_change_goal_distance);
	else
		change_distance_between_waypoints_and_goals(4.0 * fabs(current_robot_pose_v_and_phi.v), 4.0 * fabs(current_robot_pose_v_and_phi.v));

	behavior_selector_update_robot_pose(current_robot_pose_v_and_phi);

	static double last_time_obstacle_avoider_detected_obstacle = 0.0;
	if (last_obstacle_avoider_robot_hit_obstacle_message.robot_will_hit_obstacle)
	{
		obstacle_avoider_active_recently = true;
		last_time_obstacle_avoider_detected_obstacle = last_obstacle_avoider_robot_hit_obstacle_message.timestamp;
	}
	else
	{
		if ((timestamp - last_time_obstacle_avoider_detected_obstacle) > 2.0)
			obstacle_avoider_active_recently = false;
	}

	if (!autonomous)
	{
		last_not_autonomous_timestamp = timestamp;
		wait_start_moving = true;
	}
	else if (carmen_get_time() - last_not_autonomous_timestamp < 3.0)
		wait_start_moving = true;
	else if (wait_start_moving && fabs(current_robot_pose_v_and_phi.v) > 0.15)
		wait_start_moving = false;
}


carmen_ackerman_traj_point_t *
compute_simulated_objects(double timestamp)
{
	if (!necessary_maps_available)
		return (NULL);

	carmen_rddf_road_profile_message *rddf = last_rddf_message;
	if (rddf == NULL)
		return (NULL);

	static carmen_ackerman_traj_point_t previous_pose = {0, 0, 0, 0, 0};
	static double previous_timestamp = 0.0;
	static double initial_time = 0.0; // Simulation start time.

	if (initial_time == 0.0)
	{
		previous_pose = rddf->poses[rddf->number_of_poses - 1];
		previous_timestamp = timestamp;
		initial_time = timestamp;
		return &previous_pose;
	}

	// Period of time (counted from initial_time)
	// during which the obstacle is stopped.
	static double stop_t0 = 50, stop_tn = 70;

	double v = (20.0 / 3.6);
	double t = timestamp - initial_time;
	if (stop_t0 <= t && t <= stop_tn)
		v = 0;
//	else if (t > stop_tn)
//		initial_time = timestamp;

	double dt = timestamp - previous_timestamp;
	double dx = v * dt * cos(previous_pose.theta);
	double dy = v * dt * sin(previous_pose.theta);

	carmen_ackerman_traj_point_t pose_ahead;
	pose_ahead.x = previous_pose.x + dx;
	pose_ahead.y = previous_pose.y + dy;

	static carmen_ackerman_traj_point_t next_pose = {0, 0, 0, 0, 0};
	for (int i = 0, n = rddf->number_of_poses - 1; i < n; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, rddf->poses[i], rddf->poses[i + 1], pose_ahead, 0.1);
		if (status == POINT_WITHIN_SEGMENT)
			break;
	}

	previous_pose = next_pose;
	previous_timestamp = timestamp;
	return &next_pose;
}


carmen_ackerman_traj_point_t *
compute_simulated_lateral_objects(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (!necessary_maps_available)
		return (NULL);

	carmen_rddf_road_profile_message *rddf = last_rddf_message;
	if (rddf == NULL)
		return (NULL);

	static carmen_ackerman_traj_point_t previous_pose = {0, 0, 0, 0, 0};
	static carmen_ackerman_traj_point_t returned_pose = {0, 0, 0, 0, 0};
	static double previous_timestamp = 0.0;
	static double initial_time = 0.0; // Simulation start time.
	static double disp = 2.5;

	if (initial_time == 0.0)
	{
		returned_pose = previous_pose = rddf->poses[0];
		returned_pose.x = previous_pose.x + disp * cos(previous_pose.theta + M_PI / 2.0);
		returned_pose.y = previous_pose.y + disp * sin(previous_pose.theta + M_PI / 2.0);

		previous_timestamp = timestamp;
		initial_time = timestamp;

		return (&returned_pose);
	}

	static double stop_t0 = 15;
	static double stop_t1 = 15;

	static double v;
	double t = timestamp - initial_time;
	if (stop_t0 <= t && disp > 0.0)
		disp -= 0.03;
	if (t < stop_t1)
//		v = current_robot_pose_v_and_phi.v + 0.9;
		v = current_robot_pose_v_and_phi.v + 0.5; // Motos!

//	else if (t > stop_tn)
//		initial_time = timestamp;

	double dt = timestamp - previous_timestamp;
	double dx = v * dt * cos(previous_pose.theta);
	double dy = v * dt * sin(previous_pose.theta);

	carmen_ackerman_traj_point_t pose_ahead;
	pose_ahead.x = previous_pose.x + dx;
	pose_ahead.y = previous_pose.y + dy;

	static carmen_ackerman_traj_point_t next_pose = {0, 0, 0, 0, 0};
	for (int i = 0, n = rddf->number_of_poses - 1; i < n; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, rddf->poses[i], rddf->poses[i + 1], pose_ahead, 0.1);
		if ((status == POINT_WITHIN_SEGMENT) || (status == POINT_BEFORE_SEGMENT))
			break;
	}

	returned_pose = previous_pose = next_pose;
	returned_pose.x = previous_pose.x + disp * cos(previous_pose.theta + M_PI / 2.0);
	returned_pose.y = previous_pose.y + disp * sin(previous_pose.theta + M_PI / 2.0);
	previous_timestamp = timestamp;

	return (&returned_pose);
}


void
clear_moving_obstacles_from_compact_lane_map(carmen_obstacle_distance_mapper_compact_map_message *compact_lane_contents)
{
	for (int i = 0; i < compact_lane_contents->size; i++)
	{
		int index = compact_lane_contents->coord_y[i] + compact_lane_contents->config.y_size * compact_lane_contents->coord_x[i];
		compact_lane_contents->x_offset[i] = distance_map.complete_x_offset[index];
		compact_lane_contents->y_offset[i] = distance_map.complete_y_offset[index];
	}
}


void
add_simulated_object(carmen_ackerman_traj_point_t *object_pose)
{
	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x;
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y;
	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
	virtual_laser_message.num_positions++;

	double disp = 0.3;
	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x + disp * cos(object_pose->theta + M_PI / 2.0);
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y + disp * sin(object_pose->theta + M_PI / 2.0);
	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
	virtual_laser_message.num_positions++;

	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x + disp * cos(object_pose->theta - M_PI / 2.0);
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y + disp * sin(object_pose->theta - M_PI / 2.0);
	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
	virtual_laser_message.num_positions++;
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_goal_list(carmen_ackerman_traj_point_t *goal_list, int goal_list_size, double timestamp)
{
	carmen_behavior_selector_goal_list_message goal_list_msg;
	goal_list_msg.goal_list = goal_list;
	goal_list_msg.size = goal_list_size;
	goal_list_msg.timestamp = timestamp;
	goal_list_msg.host = carmen_get_host();

	IPC_RETURN_TYPE err;
	if (goal_list_road_profile_message == last_road_profile_message)
	{
		err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &goal_list_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
	}

	if (last_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
	{
		err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME, &goal_list_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME);
	}

//	FILE *caco = fopen("caco.txt", "a");
//	fprintf(caco, "%lf %lf %lf\n", goal_list->v, distance_to_moving_obstacle_annotation(get_robot_pose()), carmen_get_time());
//	fflush(caco);
//	fclose(caco);
}


void
publish_updatet_lane_contents(double timestamp)
{
	carmen_obstacle_distance_mapper_compact_map_message compact_lane_contents_cpy;
	carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(&compact_lane_contents_cpy, compact_lane_contents);
	clear_moving_obstacles_from_compact_lane_map(&compact_lane_contents_cpy);

	carmen_behaviour_selector_publish_compact_lane_contents_message(&compact_lane_contents_cpy, timestamp);

	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_lane_contents_cpy);
}


void
publish_current_state(carmen_behavior_selector_state_message msg)
{
	IPC_RETURN_TYPE err;
	carmen_behavior_selector_state_t current_state;
	carmen_behavior_selector_algorithm_t following_lane_planner;
	carmen_behavior_selector_algorithm_t parking_planner;
	carmen_behavior_selector_goal_source_t current_goal_source;

	behavior_selector_get_state(&current_state, &following_lane_planner, &parking_planner, &current_goal_source);

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	msg.algorithm = get_current_algorithm();
	msg.state = current_state;

	msg.following_lane_algorithm = following_lane_planner;
	msg.parking_algorithm = parking_planner;

	msg.goal_source = current_goal_source;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


void
publish_behavior_selector_road_profile_message(carmen_rddf_road_profile_message *rddf_msg)
{
	if (param_rddf_num_poses_by_car_velocity && (last_rddf_message != NULL))
	{
		carmen_rddf_road_profile_message *last_rddf_resized;

		last_rddf_resized = last_rddf_message;
		road_profile_message.number_of_poses = last_rddf_resized->number_of_poses;

		road_profile_message.poses = (carmen_ackerman_traj_point_t *) realloc(road_profile_message.poses, sizeof(carmen_ackerman_traj_point_t) * road_profile_message.number_of_poses);
		road_profile_message.annotations = (int *) malloc(sizeof(int) * road_profile_message.number_of_poses);

		if (last_rddf_resized->number_of_poses_back > 0)
		{
			road_profile_message.number_of_poses_back = last_rddf_resized->number_of_poses_back;
			road_profile_message.poses_back = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * road_profile_message.number_of_poses_back);

			for (int j = 0; j < last_rddf_resized->number_of_poses_back; j++)
				road_profile_message.poses_back[j] = last_rddf_resized->poses_back[j];
		}

		for (int i = 0; i < last_rddf_resized->number_of_poses; i++)
		{
			road_profile_message.annotations[i] = last_rddf_resized->annotations[i];
			road_profile_message.poses[i] = last_rddf_resized->poses[i];
		}

		road_profile_message.timestamp = carmen_get_time();
		road_profile_message.host = carmen_get_host();

		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &road_profile_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);

		free(road_profile_message.annotations);
		free(road_profile_message.poses_back);
	}
	else
	{
		road_profile_message.annotations = rddf_msg->annotations;
		road_profile_message.number_of_poses = rddf_msg->number_of_poses;
		road_profile_message.number_of_poses_back = rddf_msg->number_of_poses_back;
		road_profile_message.poses = (carmen_ackerman_traj_point_t *) realloc(road_profile_message.poses, sizeof(carmen_ackerman_traj_point_t) * road_profile_message.number_of_poses);
		for (int i = 0; i < rddf_msg->number_of_poses; i++)
			road_profile_message.poses[i] = rddf_msg->poses[i];
		road_profile_message.poses_back = rddf_msg->poses_back;
		road_profile_message.timestamp = carmen_get_time();
		road_profile_message.host = carmen_get_host();

		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &road_profile_message);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);
	}
}


void
behavior_selector_motion_planner_publish_path_message(carmen_rddf_road_profile_message *rddf_msg, int rddf_num_poses_by_velocity)
{
	if (rddf_num_poses_by_velocity)
		last_rddf_message = copy_rddf_message_considering_velocity(last_rddf_message, rddf_msg);
	else
		last_rddf_message = copy_rddf_message(last_rddf_message, rddf_msg);

	if ((get_current_algorithm() == CARMEN_BEHAVIOR_SELECTOR_RDDF) && (last_rddf_message) && (last_rddf_message->number_of_poses > 0))
		carmen_motion_planner_publish_path_message(last_rddf_message->poses, last_rddf_message->number_of_poses, CARMEN_BEHAVIOR_SELECTOR_RDDF);
}


void
publish_dynamic_annotation(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description,
		int annotation_type, int annotation_code, double timestamp)
{
	carmen_rddf_publish_dynamic_annotation_message(annotation_point, orientation, annotation_description, annotation_type,
			annotation_code, timestamp);
}


void
publish_simulated_objects()
{
	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
	virtual_laser_message.num_positions = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////


bool
stop_sign_ahead(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (false);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);
	double distance_to_act_on_annotation = get_distance_to_act_on_annotation(current_robot_pose_v_and_phi.v, 0.1, distance_to_annotation);
	carmen_ackerman_traj_point_t displaced_robot_pose = displace_pose(current_robot_pose_v_and_phi, -1.0);

	if ((nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP) &&
		(distance_to_act_on_annotation >= distance_to_annotation) &&
		carmen_rddf_play_annotation_is_forward(displaced_robot_pose, nearest_velocity_related_annotation->annotation_point))
		return (true);
	else
		return (false);
}


double
distance_to_stop_sign(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_red_traffic_light(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) &&
		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_traffic_light_stop(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_busy_pedestrian_track(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, wait_start_moving);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) &&
		(nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP))
		return (distance_to_annotation);
	else
		return (1000.0);
}


double
distance_to_pedestrian_track_stop(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi)
{
	carmen_annotation_t *nearest_velocity_related_annotation = get_nearest_velocity_related_annotation(last_rddf_annotation_message,
				&current_robot_pose_v_and_phi, false);

	if (nearest_velocity_related_annotation == NULL)
		return (1000.0);

	double distance_to_annotation = DIST2D(nearest_velocity_related_annotation->annotation_point, current_robot_pose_v_and_phi);

	if (nearest_velocity_related_annotation->annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP)
		return (distance_to_annotation);
	else
		return (1000.0);
}


void
clear_state_output(carmen_behavior_selector_state_message *decision_making_state_msg)
{
    decision_making_state_msg->behaviour_seletor_mode = none;
}


int
perform_state_action(carmen_behavior_selector_state_message *decision_making_state_msg, carmen_ackerman_traj_point_t *goal __attribute__ ((unused)),
		double timestamp __attribute__ ((unused)))
{
//	carmen_vector_3D_t annotation_point;
//	carmen_point_t new_car_pose;
//
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			break;
		case Stopped:
			break;
		case Free_Running:
			break;

		case Following_Moving_Object:
			break;
//		case Stopping_Behind_Moving_Object:
//			new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(goal, 0.0);
//			annotation_point.x = new_car_pose.x;
//			annotation_point.y = new_car_pose.y;
//			annotation_point.z = 0.0;
//			publish_dynamic_annotation(annotation_point, goal->theta, (char *) "RDDF_ANNOTATION_TYPE_DYNAMIC",
//					RDDF_ANNOTATION_TYPE_DYNAMIC, RDDF_ANNOTATION_CODE_DYNAMIC_STOP, timestamp);
//			break;
//		case Stopped_Behind_Moving_Object_S0:
//			carmen_navigator_ackerman_stop();
//
//			new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(goal, 0.0);
//			annotation_point.x = new_car_pose.x;
//			annotation_point.y = new_car_pose.y;
//			annotation_point.z = 0.0;
//			publish_dynamic_annotation(annotation_point, goal->theta, (char *) "RDDF_ANNOTATION_TYPE_DYNAMIC",
//					RDDF_ANNOTATION_TYPE_DYNAMIC, RDDF_ANNOTATION_CODE_DYNAMIC_STOP, timestamp);
//			break;
//		case Stopped_Behind_Moving_Object_S1:
//			break;
//		case Stopped_Behind_Moving_Object_S2:
//			carmen_navigator_ackerman_go();
//			break;

		case Stopping_At_Red_Traffic_Light:
			break;
		case Stopped_At_Red_Traffic_Light_S0:
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			break;
		case Stopped_At_Red_Traffic_Light_S2:
			carmen_navigator_ackerman_go();
			break;

		case Stopping_At_Busy_Pedestrian_Track:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
			carmen_navigator_ackerman_go();
			break;

		case Stopping_At_Stop_Sign:
			break;
		case Stopped_At_Stop_Sign_S0:
			carmen_navigator_ackerman_stop();
			break;
		case Stopped_At_Stop_Sign_S1:
			break;
		default:
			printf("Error: Unknown state in perform_state_action()\n");
			return (1);
	}

	return (0);
}


int
perform_state_transition(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, int goal_type __attribute__ ((unused)), double timestamp)
{
	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			decision_making_state_msg->low_level_state = Stopped;
			break;
		case Stopped:
			if (!autonomous)
				decision_making_state_msg->low_level_state = Stopped;
			else
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Free_Running:
//			if (udatmo_obstacle_detected(timestamp))
//				decision_making_state_msg->low_level_state = Following_Moving_Object;
			if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
			else if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopping_At_Busy_Pedestrian_Track;
			else if (stop_sign_ahead(current_robot_pose_v_and_phi))
				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
			break;

//		case Following_Moving_Object:
//			if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) &&
//				(distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) < udatmo_get_moving_obstacle_distance(current_robot_pose_v_and_phi, get_robot_config())))
//				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
//			else if (stop_sign_ahead(current_robot_pose_v_and_phi) &&
//					 (stop_sign_distance(current_robot_pose_v_and_phi) < udatmo_get_moving_obstacle_distance(current_robot_pose_v_and_phi, get_robot_config())))
//				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
//			else if (!udatmo_obstacle_detected(timestamp))
//				decision_making_state_msg->low_level_state = Free_Running;
//			else if (udatmo_speed_front() < 0.5)
//				decision_making_state_msg->low_level_state = Stopping_Behind_Moving_Object;
//			break;
//		case Stopping_Behind_Moving_Object:
//			if ((goal_type != MOVING_OBSTACLE_GOAL) && (goal_type != OBSTACLE_GOAL) && (goal_type != DYNAMIC_ANNOTATION_GOAL))
//				decision_making_state_msg->low_level_state = Free_Running;
//			else if ((current_robot_pose_v_and_phi.v < 0.1) && (distance_to_moving_obstacle_annotation(current_robot_pose_v_and_phi) < 2.0))
//				decision_making_state_msg->low_level_state = Stopped_Behind_Moving_Object_S0;
//			else if (red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) &&
//				(distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) < udatmo_get_moving_obstacle_distance(current_robot_pose_v_and_phi, get_robot_config())))
//				decision_making_state_msg->low_level_state = Stopping_At_Red_Traffic_Light;
//			else if (stop_sign_ahead(current_robot_pose_v_and_phi) &&
//					 (stop_sign_distance(current_robot_pose_v_and_phi) < udatmo_get_moving_obstacle_distance(current_robot_pose_v_and_phi, get_robot_config())))
//				decision_making_state_msg->low_level_state = Stopping_At_Stop_Sign;
//			else if (udatmo_speed_front() > 0.5)
//				decision_making_state_msg->low_level_state = Following_Moving_Object;
//			break;
//		case Stopped_Behind_Moving_Object_S0:
//			decision_making_state_msg->low_level_state = Stopped_Behind_Moving_Object_S1;
//			break;
//		case Stopped_Behind_Moving_Object_S1:
//			{
//				static int steps = 0;
//
//				if (steps > 3)
//				{
//					if ((udatmo_speed_front() > 0.3) || autonomous) // @@@ Alberto: ou objeto desapareceu da frente da IARA (fazer funcao para checar isso)
//					{
//						decision_making_state_msg->low_level_state = Stopped_Behind_Moving_Object_S2;
//						steps = 0;
//					}
//				}
//				else
//					steps++;
//			}
//
//			break;
//		case Stopped_Behind_Moving_Object_S2:
//			if (current_robot_pose_v_and_phi.v > 0.5)
//				decision_making_state_msg->low_level_state = Following_Moving_Object;
//			else if (udatmo_speed_front() < 0.1)
//				decision_making_state_msg->low_level_state = Stopped_Behind_Moving_Object_S0;
//			break;

		case Stopping_At_Red_Traffic_Light:
//			if (udatmo_obstacle_detected(timestamp) &&
//				(udatmo_get_moving_obstacle_distance(current_robot_pose_v_and_phi, get_robot_config()) < distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp)))
//				decision_making_state_msg->low_level_state = Following_Moving_Object;
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) < 2.0) ||
				 (distance_to_red_traffic_light(current_robot_pose_v_and_phi, timestamp) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S0;
			else if (!red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Red_Traffic_Light_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S1;
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!red_traffic_light_ahead(current_robot_pose_v_and_phi, timestamp) || autonomous)
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Red_Traffic_Light_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Red_Traffic_Light_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_traffic_light_stop(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			break;

		case Stopping_At_Busy_Pedestrian_Track:
			if ((current_robot_pose_v_and_phi.v < 0.15) &&
				((distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) < 2.0) ||
				 (distance_to_busy_pedestrian_track(current_robot_pose_v_and_phi, timestamp) == 1000.0)))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S0;
			else if (!busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S1;
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			{
				static int steps2 = 0;

				if (steps2 > 3)
				{
					if (!busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp) || autonomous)
					{
						steps2 = 0;
						decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S2;
					}
				}
				else
					steps2++;
			}
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5) && (distance_to_pedestrian_track_stop(current_robot_pose_v_and_phi) > 2.0))
				decision_making_state_msg->low_level_state = Free_Running;
			if (busy_pedestrian_track_ahead(current_robot_pose_v_and_phi, timestamp))
				decision_making_state_msg->low_level_state = Stopped_At_Busy_Pedestrian_Track_S0;
			break;

		case Stopping_At_Stop_Sign:
			if ((fabs(current_robot_pose_v_and_phi.v) < 0.01) && (distance_to_stop_sign(current_robot_pose_v_and_phi) < 4.0))
				decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S0;
			break;
		case Stopped_At_Stop_Sign_S0:
			decision_making_state_msg->low_level_state = Stopped_At_Stop_Sign_S1;
			break;
		case Stopped_At_Stop_Sign_S1:
			if (autonomous && (current_robot_pose_v_and_phi.v > 0.5))
				decision_making_state_msg->low_level_state = Free_Running;
			break;
		default:
			printf("Error: Unknown state in perform_state_transition()\n");
			return (2);
	}
	return (0);
}


int
generate_state_output(carmen_behavior_selector_state_message *decision_making_state_msg)
{
	clear_state_output(decision_making_state_msg);

	switch (decision_making_state_msg->low_level_state)
	{
		case Initializing:
			break;
		case Stopped:
			break;
		case Free_Running:
			break;

//		case Following_Moving_Object:
//			decision_making_state_msg->behaviour_seletor_mode = following_moving_object;
//			break;
//		case Stopping_Behind_Moving_Object:
//			break;
//		case Stopped_Behind_Moving_Object_S0:
//			break;
//		case Stopped_Behind_Moving_Object_S1:
//			break;
//		case Stopped_Behind_Moving_Object_S2:
//			break;

		case Stopping_At_Red_Traffic_Light:
			break;
		case Stopped_At_Red_Traffic_Light_S0:
			break;
		case Stopped_At_Red_Traffic_Light_S1:
			break;
		case Stopped_At_Red_Traffic_Light_S2:
			break;

		case Stopping_At_Busy_Pedestrian_Track:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S0:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S1:
			break;
		case Stopped_At_Busy_Pedestrian_Track_S2:
			break;

		case Stopping_At_Stop_Sign:
			break;
		case Stopped_At_Stop_Sign_S0:
			break;
		case Stopped_At_Stop_Sign_S1:
			break;
		default:
			printf("Error: Unknown state in generate_state_output()\n");
			return (3);
	}
	return (0);
}


int
run_decision_making_state_machine(carmen_behavior_selector_state_message *decision_making_state_msg,
		carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, carmen_ackerman_traj_point_t *goal, int goal_type,
		double timestamp)
{
	int error;

	error = perform_state_transition(decision_making_state_msg, current_robot_pose_v_and_phi, goal_type, timestamp);
	if (error != 0)
		return (error);

	error = perform_state_action(decision_making_state_msg, goal, timestamp);
	if (error != 0)
		return (error);

	error = generate_state_output(decision_making_state_msg);
	if (error != 0)
		return (error);

	return (0);
}


carmen_ackerman_traj_point_t *
check_soft_stop(carmen_ackerman_traj_point_t *first_goal, carmen_ackerman_traj_point_t *goal_list, int &goal_type)
{
	static bool soft_stop_in_progress = false;
	static carmen_ackerman_traj_point_t soft_stop_goal;
	static int soft_stop_goal_type;

	if (soft_stop_on)
	{
		if (!soft_stop_in_progress)
		{
			soft_stop_goal = *first_goal;
			soft_stop_goal_type = goal_type;
			soft_stop_in_progress = true;
		}
		else
		{
			goal_list[0] = soft_stop_goal;
			first_goal = &soft_stop_goal;
			goal_type = soft_stop_goal_type;
		}
	}
	else
		soft_stop_in_progress = false;

	return (first_goal);
}


void
select_behaviour(carmen_ackerman_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, compact_lane_contents);

	set_behaviours_parameters(current_robot_pose_v_and_phi, timestamp);

	// Esta funcao altera a mensagem de rddf e funcoes abaixo dela precisam da original
	last_rddf_message_copy = copy_rddf_message(last_rddf_message_copy, last_rddf_message);
	behaviour_selector_fill_goal_list(last_rddf_message_copy, timestamp);

	int goal_list_size;
	carmen_ackerman_traj_point_t *goal_list = behavior_selector_get_goal_list(&goal_list_size);
	carmen_ackerman_traj_point_t *first_goal = &(goal_list[0]);
	int goal_type = behavior_selector_get_goal_type()[0];

	first_goal = check_soft_stop(first_goal, goal_list, goal_type);

	int error = run_decision_making_state_machine(&behavior_selector_state_message, current_robot_pose_v_and_phi,
			first_goal, goal_type, timestamp);
	if (error != 0)
		carmen_die("State machine error code %d\n", error);

	static carmen_ackerman_traj_point_t last_valid_goal;
	static carmen_ackerman_traj_point_t *last_valid_goal_p = NULL;
	if (goal_list_size > 0)
	{
		set_goal_velocity(first_goal, &current_robot_pose_v_and_phi, goal_type, timestamp);
		publish_goal_list(goal_list, goal_list_size, timestamp);

		last_valid_goal = *first_goal;
		last_valid_goal_p = &last_valid_goal;
	}
	else if (last_valid_goal_p != NULL)
	{	// Garante parada suave ao fim do rddf
		last_valid_goal_p->v = 0.0;
		publish_goal_list(last_valid_goal_p, 1, timestamp);
	}

	publish_updatet_lane_contents(timestamp);

// Control whether simulated moving obstacles are created by (un)commenting the
// definition of the macro below at the top of this file.
#ifdef SIMULATE_MOVING_OBSTACLE
	carmen_ackerman_traj_point_t *simulated_object_pose = compute_simulated_objects(timestamp);
	if (simulated_object_pose)
		add_simulated_object(simulated_object_pose);
#endif
#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
	carmen_ackerman_traj_point_t *simulated_object_pose2 = compute_simulated_lateral_objects(current_robot_pose_v_and_phi, timestamp);
	if (simulated_object_pose2)
		add_simulated_object(simulated_object_pose2);
#endif

#if defined(SIMULATE_LATERAL_MOVING_OBSTACLE) || defined(SIMULATE_MOVING_OBSTACLE)
	if (virtual_laser_message.num_positions >= 0)
		publish_simulated_objects();
#endif

	publish_current_state(behavior_selector_state_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (!necessary_maps_available || !last_rddf_message)
		return;

	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->globalpos.x;
	current_robot_pose_v_and_phi.y = msg->globalpos.y;
	current_robot_pose_v_and_phi.theta = msg->globalpos.theta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	select_behaviour(current_robot_pose_v_and_phi, msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	if (!necessary_maps_available || !last_rddf_message)
		return;

	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->truepose.x;
	current_robot_pose_v_and_phi.y = msg->truepose.y;
	current_robot_pose_v_and_phi.theta = msg->truepose.theta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	select_behaviour(current_robot_pose_v_and_phi, msg->timestamp);
}


static void
rddf_handler(carmen_rddf_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	behavior_selector_motion_planner_publish_path_message(rddf_msg, param_rddf_num_poses_by_car_velocity);
	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;

	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
		publish_behavior_selector_road_profile_message(rddf_msg);
}


static void
path_planner_road_profile_handler(carmen_path_planner_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	behavior_selector_motion_planner_publish_path_message((carmen_rddf_road_profile_message *) rddf_msg, 0);
	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;

	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL)
	{
		carmen_behavior_selector_road_profile_message msg;
		msg.annotations = rddf_msg->annotations;
		msg.number_of_poses = rddf_msg->number_of_poses;
		msg.number_of_poses_back = rddf_msg->number_of_poses_back;
		msg.poses = rddf_msg->poses;
		msg.poses_back = rddf_msg->poses_back;
		msg.timestamp = carmen_get_time();
		msg.host = carmen_get_host();

		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);
	}
//	publish_goal_list();
}


//static void
//carmen_obstacle_distance_mapper_message_handler(carmen_obstacle_distance_mapper_map_message *message)
//{
//	behavior_selector_update_map(message);
//
//	necessary_maps_available = 1;
//}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (compact_lane_contents == NULL)
		return;

	if (compact_distance_map == NULL)
	{
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}

	behavior_selector_update_map(&distance_map);

	necessary_maps_available = 1;
}


static void
carmen_obstacle_distance_mapper_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (compact_lane_contents == NULL)
	{
		carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
		compact_lane_contents = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_lane_contents, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_lane_contents, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_lane_contents);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_lane_contents, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
}


static void
obstacle_avoider_robot_hit_obstacle_message_handler(carmen_obstacle_avoider_robot_will_hit_obstacle_message *robot_hit_obstacle_message)
{
	last_obstacle_avoider_robot_hit_obstacle_message = *robot_hit_obstacle_message;
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	last_rddf_annotation_message = *message;
	last_rddf_annotation_message_valid = true;
}


static void
set_algorith_handler(carmen_behavior_selector_set_algorithm_message *msg)
{
	behavior_selector_set_algorithm(msg->algorithm, msg->state);
}


static void
set_goal_source_handler(carmen_behavior_selector_set_goal_source_message *msg)
{
	behavior_selector_set_goal_source(msg->goal_source);
}


static void
set_state_handler(carmen_behavior_selector_set_state_message *msg)
{
	behavior_selector_set_state(msg->state);
}


static void
add_goal_handler(carmen_behavior_selector_add_goal_message *msg)
{
	behavior_selector_add_goal(msg->goal);
}


static void
clear_goal_list_handler()
{
	behavior_selector_clear_goal_list();
}


static void
remove_goal_handler()
{
	behavior_selector_remove_goal();
}


static void
carmen_navigator_ackerman_status_message_handler(carmen_navigator_ackerman_status_message *msg)
{
	autonomous = (msg->autonomous == 1)? true: false;
}


static void
carmen_voice_interface_command_message_handler(carmen_voice_interface_command_message *message)
{
	if (message->command_id == SET_SPEED)
	{
		if (strcmp(message->command, "MAX_SPEED") == 0)
		{
			set_max_v(carmen_ini_max_velocity);
			soft_stop_on = false;
		}
		else if (strcmp(message->command, "0.0") == 0)
		{
			set_max_v(0.0);
			soft_stop_on = true;
		}

		printf("New speed set by voice command: %lf\n", get_max_v());
		fflush(stdout);
	}
}


static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
register_handlers()
{
	carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(
			NULL,
			(carmen_handler_t) obstacle_avoider_robot_hit_obstacle_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_road_profile_message(
			NULL,(carmen_handler_t)rddf_handler,
			CARMEN_SUBSCRIBE_LATEST);

	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_obstacle_distance_mapper_subscribe_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_distance_mapper_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	// **************************************************
	// filipe:: TODO: criar funcoes de subscribe no interfaces!
	// **************************************************

    carmen_subscribe_message((char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_NAME,
			(char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_FMT,
			NULL, sizeof (carmen_path_planner_road_profile_message),
			(carmen_handler_t)path_planner_road_profile_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT,
			NULL, sizeof(carmen_behavior_selector_set_algorithm_message),
			(carmen_handler_t)set_algorith_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_FMT,
			NULL, sizeof(carmen_behavior_selector_set_goal_source_message),
			(carmen_handler_t)set_goal_source_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_STATE_FMT,
			NULL, sizeof(carmen_behavior_selector_set_state_message),
			(carmen_handler_t)set_state_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_add_goal_message),
			(carmen_handler_t)add_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_FMT,
			NULL, sizeof(carmen_behavior_selector_clear_goal_list_message),
			(carmen_handler_t)clear_goal_list_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_remove_goal_message),
			(carmen_handler_t)remove_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_status_message(NULL, (carmen_handler_t) carmen_navigator_ackerman_status_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_voice_interface_subscribe_command_message(NULL, (carmen_handler_t) carmen_voice_interface_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME);

    err = IPC_defineMsg(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
    		CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define",
    		CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);

}


static void
read_parameters(int argc, char **argv)
{
	carmen_robot_ackerman_config_t robot_config;
	double distance_between_waypoints, change_goal_distance, distance_to_remove_annotation_goal;
	carmen_behavior_selector_algorithm_t parking_planner, following_lane_planner;

	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
		{(char *) "robot", (char *) "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
		{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
		{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
		{(char *) "robot", (char *) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
		{(char *) "robot", (char *) "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward, 1, NULL},
		{(char *) "robot", (char *) "max_centripetal_acceleration", CARMEN_PARAM_DOUBLE, &robot_max_centripetal_acceleration, 1, NULL},
		{(char *) "robot", (char *) "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels, 1,NULL},
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
		{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{(char *) "robot", (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels, 1, NULL},
		{(char *) "behavior_selector", (char *) "distance_between_waypoints", CARMEN_PARAM_DOUBLE, &distance_between_waypoints, 1, NULL},
		{(char *) "behavior_selector", (char *) "change_goal_distance", CARMEN_PARAM_DOUBLE, &change_goal_distance, 1, NULL},
		{(char *) "behavior_selector", (char *) "following_lane_planner", CARMEN_PARAM_INT, &following_lane_planner, 1, NULL},
		{(char *) "behavior_selector", (char *) "parking_planner", CARMEN_PARAM_INT, &parking_planner, 1, NULL},
		{(char *) "behavior_selector", (char *) "goal_source_path_planner", CARMEN_PARAM_ONOFF, &param_goal_source_onoff, 0, NULL},
		{(char *) "behavior_selector", (char *) "distance_to_remove_annotation_goal", CARMEN_PARAM_DOUBLE, &distance_to_remove_annotation_goal, 0, NULL},
		{(char *) "behavior_selector", (char *) "rddf_num_poses_ahead_limit", CARMEN_PARAM_INT, &param_rddf_num_poses_ahead_limited_by_map, 0, NULL},
		{(char *) "behavior_selector", (char *) "rddf_num_poses_ahead_min", CARMEN_PARAM_INT, &param_rddf_num_poses_ahead_min, 0, NULL},
		{(char *) "behavior_selector", (char *) "rddf_num_poses_by_car_velocity", CARMEN_PARAM_ONOFF, &param_rddf_num_poses_by_car_velocity, 0, NULL},
		{(char *) "behavior_selector", (char *) "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},
		{(char *) "behavior_selector", (char *) "main_central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.behaviour_selector_main_central_lane_obstacles_safe_distance, 0, NULL},
		{(char *) "behavior_selector", (char *) "central_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.behaviour_selector_central_lane_obstacles_safe_distance, 0, NULL},
		{(char *) "behavior_selector", (char *) "lateral_lane_obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.behaviour_selector_lateral_lane_obstacles_safe_distance, 0, NULL},
		{(char *) "behavior_selector", (char *) "lateral_lane_displacement", CARMEN_PARAM_DOUBLE, &robot_config.behaviour_selector_lateral_lane_displacement, 0, NULL},
		{(char *) "rrt",   			   (char *) "distance_interval", CARMEN_PARAM_DOUBLE, &param_distance_interval, 1, NULL},
		{(char *) "obstacle_avoider", 		  (char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.obstacle_avoider_obstacles_safe_distance, 	1, NULL},
		{(char *) "model_predictive_planner", (char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.model_predictive_planner_obstacles_safe_distance, 	1, NULL},
		{(char *) "grid_mapping",      (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 	1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));


	carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_param_list[] =
	{
		{(char *) "commandline", (char *) "activate_tracking", CARMEN_PARAM_ONOFF, &activate_tracking, 0, NULL}
	};
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	param_distance_between_waypoints = distance_between_waypoints;
	param_change_goal_distance = change_goal_distance;

	carmen_ini_max_velocity = robot_config.max_v;
	behavior_selector_initialize(robot_config, distance_between_waypoints, change_goal_distance, following_lane_planner, parking_planner);

	if (param_goal_source_onoff)
		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;
	else
		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;

	original_behaviour_selector_central_lane_obstacles_safe_distance = robot_config.behaviour_selector_central_lane_obstacles_safe_distance;
	original_model_predictive_planner_obstacles_safe_distance = robot_config.model_predictive_planner_obstacles_safe_distance;
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);
	carmen_ipc_initialize(argc, argv);

	memset(&last_rddf_annotation_message, 0, sizeof(last_rddf_annotation_message));
	memset(&road_profile_message, 0, sizeof(carmen_behavior_selector_road_profile_message));
	memset(&behavior_selector_state_message, 0, sizeof(carmen_behavior_selector_state_message));

	read_parameters(argc, argv);

	define_messages();
	register_handlers();

	carmen_ipc_dispatch();

	return 0;
}
