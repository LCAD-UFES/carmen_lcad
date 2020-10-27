#include <list>
#include <carmen/carmen.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>
#include <carmen/collision_detection.h>
#include <carmen/traffic_light_messages.h>

#include "g2o/types/slam2d/se2.h"

#include "rddf_util.h"
#include "rddf_index.h"

using namespace std;
using namespace g2o;

typedef enum
{
	Free_Crosswalk,
	Stopping_Busy_Crosswalk,
	Stopped_Busy_Crosswalk,
	Leaving_Crosswalk
} carmen_rddf_play_state;

carmen_rddf_play_state crosswalk_state = Free_Crosswalk;


extern double distance_between_front_and_rear_axles;
extern double distance_between_front_car_and_front_wheels;
extern int carmen_rddf_perform_loop;

extern carmen_localize_ackerman_globalpos_message *current_globalpos_msg;
extern carmen_moving_objects_point_clouds_message *moving_objects;
extern carmen_traffic_light_message *traffic_lights;
extern deque<carmen_rddf_dynamic_annotation_message> dynamic_annotation_messages;

extern bool simulated_pedestrian_on;
extern vector<carmen_annotation_t> annotation_read_from_file;
extern vector<annotation_and_index> annotations_to_publish;

extern int *annotations_codes;

extern int carmen_rddf_nearest_waypoint_is_set;
extern int already_reached_nearest_waypoint_to_end_point;
extern carmen_point_t carmen_rddf_end_point;
extern carmen_point_t carmen_rddf_nearest_waypoint_to_end_point;
extern int carmen_rddf_num_poses_ahead;
extern int *annotations;
extern carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead;


void
carmen_rddf_play_clear_annotations()
{
	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
}


void
clear_annotations(int *rddf_annotations, int num_annotations)
{
	int i;

	for(i = 0; i < num_annotations; i++)
	{
		rddf_annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
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


int
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


carmen_vector_2D_t
get_displaced_annotation_position(carmen_annotation_t pedestrian_track_annotation)    // The crosswalk annotated position is displaced by the distance from rear axle to car front
{                                                                                     // because the annotation position is made this way
	carmen_vector_2D_t displaced_crosswalk_pose;
	double displacement = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
	double theta = pedestrian_track_annotation.annotation_orientation;
	displaced_crosswalk_pose.x = pedestrian_track_annotation.annotation_point.x + displacement * cos(theta);
	displaced_crosswalk_pose.y = pedestrian_track_annotation.annotation_point.y + displacement * sin(theta);

	return (displaced_crosswalk_pose);
}


bool
pedestrian_about_to_enter_crosswalk(t_point_cloud_struct moving_object, carmen_annotation_t pedestrian_track_annotation, double radius)
{
	carmen_vector_2D_t displaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	double pedestrian_time_to_crosswalk_border = 999.9;//DBL_MAX;
	double car_time_to_crosswalk_center = 999.9;//DBL_MAX;
	double orientantion;

	orientantion = carmen_normalize_theta(atan2(displaced_crosswalk_pose.y - moving_object.object_pose.y, displaced_crosswalk_pose.x - moving_object.object_pose.x));

	if (abs(carmen_normalize_theta(orientantion - moving_object.orientation)) > 0.53)   // ~30 degrees; Pedestrian is not walking towards crosswalk
		return (false);

	double x_border = radius * cos((orientantion)) + displaced_crosswalk_pose.x;            // Position the pedestrian will be when it hits crosswalk border
	double y_border = radius * sin((orientantion)) + displaced_crosswalk_pose.y;

	double d_x = x_border - moving_object.object_pose.x;
	double d_y = y_border - moving_object.object_pose.y;

	if (moving_object.linear_velocity > 0.4)
		pedestrian_time_to_crosswalk_border = abs((sqrt((d_x * d_x) + (d_y * d_y)))) / moving_object.linear_velocity;

	if (current_globalpos_msg->v > 0.5)
		car_time_to_crosswalk_center = abs(DIST2D(current_globalpos_msg->globalpos, displaced_crosswalk_pose)) / current_globalpos_msg->v;

#ifdef PRINT_DEBUG
	printf("%lf %lf ", orientantion, moving_object.orientation);
	printf("Bx %lf By %lf ", x_border, y_border);
	printf("TC %lf TP %lf", car_time_to_crosswalk_center, pedestrian_time_to_crosswalk_border);
	printf("C %lf P %lf\n", current_globalpos_msg->v, moving_object.linear_velocity);
	fflush(stdout);
#endif

	if (car_time_to_crosswalk_center < pedestrian_time_to_crosswalk_border)
		return (false);

	//printf("about_to_enter\n");
	return (true);
}


bool
pedestrian_in_crosswalk(carmen_moving_objects_point_clouds_message *moving_objects, carmen_annotation_t pedestrian_track_annotation)
{
	double radius = pedestrian_track_annotation.annotation_point.z;
	carmen_vector_2D_t displaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if ((strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0) &&
			(pedestrian_about_to_enter_crosswalk(moving_objects->point_clouds[i], pedestrian_track_annotation, radius) ||
			 DIST2D(moving_objects->point_clouds[i].object_pose, displaced_crosswalk_pose) < radius))
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
	double radius = pedestrian_track_annotation.annotation_point.z;
	carmen_vector_2D_t displaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if (strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0)
		{
			if ((DIST2D(moving_objects->point_clouds[i].object_pose, displaced_crosswalk_pose) < radius &&                    // Inside the crosswalk circle
				 moving_objects_msg->point_clouds[i].linear_velocity > 0.2 &&                                              // Moving faster than 0.2m/s
				 abs(current_globalpos_msg->globalpos.theta - moving_objects_msg->point_clouds[i].orientation) > 0.2)      // Not moving parallel to the car (sideways with the crosswalk)
				 )//||
				 //pedestrian_about_to_enter_crosswalk(moving_objects_msg->point_clouds[i], displaced_crosswalk_pose, radius))
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

	carmen_vector_2D_t displaced_crosswalk_pose = get_displaced_annotation_position(pedestrian_track_annotation);

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
			//printf("Stopping_Busy_Crosswalk %lf %lf\n", current_globalpos_msg->v, DIST2D(current_globalpos_msg->globalpos, displaced_crosswalk_pose));
			if (!pedestrian_in_crosswalk(moving_objects_msg, pedestrian_track_annotation))
			{
				crosswalk_state = Free_Crosswalk;
				return (false);
			}
			else if (current_globalpos_msg->v < 0.15 && DIST2D(current_globalpos_msg->globalpos, displaced_crosswalk_pose) < 20.0) // || dist stop point < 2.0
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
			//printf("Leaving_Crosswalk %lf\n", DIST2D(current_globalpos_msg->globalpos, displaced_crosswalk_pose));
			if (pedestrian_crossing(moving_objects_msg, pedestrian_track_annotation))
			{
				printf("pedestrian_crossing \n");
				crosswalk_state = Stopped_Busy_Crosswalk;
				return (true);
			}
			else if (DIST2D(current_globalpos_msg->globalpos, displaced_crosswalk_pose) < 2.0)
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
				annotation_i.annotation.annotation_code = RDDF_ANNOTATION_CODE_PEDESTRIAN_TRACK_BUSY;
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
//			printf("---STOP\n");
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
carmen_rddf_play_set_annotations(carmen_point_t robot_pose)
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
