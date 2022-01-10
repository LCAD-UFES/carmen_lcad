#include <vector>
#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>
#include <carmen/path_planner_messages.h>
#include <carmen/grid_mapping.h>
#include <carmen/udatmo.h>
#include <carmen/udatmo_messages.h>
#include <prob_map.h>
#include <carmen/map.h>
#include <carmen/map_server_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/voice_interface_messages.h>
#include <carmen/voice_interface_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/obstacle_distance_mapper_datmo.h>
#include <carmen/task_manager_messages.h>
#include <carmen/task_manager_interface.h>

#include "behavior_selector.h"
#include "behavior_selector_messages.h"

//
// O obstacle_distance_mapper (ODM) recebe (a) um mapa de ocupação compactado do mapper (apenas as células com probabilidade
// de ocupação maior que mapper_min_occupied_prob (algo próximo de 0.5) e (b) o road map do route_planner. A partir destes dados,
// o ODM computa: (i) o mapa de distâncias para células ocupadas com objetos não móveis (ou com movimento lento - SODM); e
// (ii) os objetos móveis no road map (MO). Estes dois itens são enviados via mensagem existentes para os demais módulos.
// Alternativamente, estes dois itens podem ser agrupados em uma mensagem, por exemplo, carmen_obstacle_distance_mapper_output_message,
// e publicados de forma síncrona. Os objetos móveis devem ser codificados nesta mensagem de modo a poder ser acomodados em
// uma mensagem do tipo carmen_moving_objects_point_clouds_message.
//
// O behavior_selector (BS) recebe estes itens via mensagem(s) do ODM e outros dados e: (i) computa um conjunto de paths usando
// Frenet Frames; (ii) escolhe um goal que possa ser alcançado sem colisão em cada path; (iii) determina a velocidade em cada um
// destes goals; (iv) e escolhe o melhor conjunto path/goal/velocidade do goal.
//
// A escolha do melhor conjunto é feita pontuando: a distância para o goal (quanto maior melhor), a velocidade no goal (quanto
// maior melhor), e a distância para o path central que leva ao RDDF (quanto menor melhor).
//
// O itens (ii) e (iii) imediatamente acima são feitos em conjunto. Isto é, o BS pode, para cada path, fazer busca binária
// de qual velocidade pode ser atingida sem colisão, respeitando as acelerações máximas e o horizonte de tempo de planejamento.
// Ele tenta a velociade mais alta possível primeiro, V. Se houver colisão no horizonte de tempo de planejamento,
// ele tenta a 1/2 de V. Se na metada de V não houver colisão, ele tenta 3/4 de V, caso contrário, 1/4 de V e assim por diante até
// um nível mínimo de discretização de V em que consiga estabelecer um goal e uma velocidade.
//
// Na busca de um par velocidade / goal em um path, o BS examina cada ponto do path em busca de colisões com o SODM (colisão com objetos
// estáticos) e colisões com os MOs. Além disso, o BS checa anotações e se o goal vai ficar fora do mapa (neste
// caso, coloca o goal uma posição antes). Ou seja, dá para usar o infra de código existente para escolher goals e velocidade
// de goals! Basta mandar o path no lugar do RDDF para cada função existente do BS. Mas como escolher o melhor conjunto
// path/goal/velocidade do goal? A ideia é atribuir custos para cada path (e goal neste path e velocidade neste path) e escolher
// o de menor custo.
//
// Nas mensagens de MO, observar o timestamp dos MOs.
// Tratar MO no obstacle_avoider.
// Tratar MO no BS conforme acima.
// Por que entra no Low Level State: Stoping_at_Atop Sign e não sai mais?
//

// Comment or uncomment this definition to control whether simulated moving obstacles are created.
//#define SIMULATE_MOVING_OBSTACLE
//#define SIMULATE_LATERAL_MOVING_OBSTACLE

// Comment or uncomment this definition to control whether moving obstacles are displayed.
#define DISPLAY_MOVING_OBSTACLES

using namespace std;

vector<path_collision_info_t> set_optimum_path(carmen_frenet_path_planner_set_of_paths *current_set_of_paths,
		carmen_moving_objects_point_clouds_message *current_moving_objects,
		carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		int who_set_the_goal_v, carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp);


static int necessary_maps_available = 0;
static bool obstacle_avoider_active_recently = false;
static int activate_tracking = 0;
bool keep_speed_limit = false;
double last_speed_limit;

int behavior_selector_use_symotha = 0;
double obstacle_probability_threshold 	= 0.5;
double min_moving_object_velocity 		= 0.3;
double max_moving_object_velocity 		= 150.0 / 3.6; // 150 km/h
double moving_object_merge_distance		= 1.0; // m

double param_distance_between_waypoints;
double param_change_goal_distance;
double param_distance_interval;

double map_width;

carmen_obstacle_avoider_robot_will_hit_obstacle_message last_obstacle_avoider_robot_hit_obstacle_message;
carmen_rddf_annotation_message last_rddf_annotation_message;
bool last_rddf_annotation_message_valid = false;
//carmen_behavior_selector_goal_source_t last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
//carmen_behavior_selector_goal_source_t goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static int param_goal_source_onoff = 0;

int param_rddf_num_poses_ahead_limited_by_map;
int param_rddf_num_poses_ahead_min;
int param_rddf_num_poses_by_car_velocity = 1;
int carmen_rddf_num_poses_ahead = 100;

//carmen_behavior_selector_path_goals_and_annotations_message path_goals_and_annotations_message;
double robot_max_centripetal_acceleration = 1.5;

int use_truepos = 0;
extern carmen_mapper_virtual_laser_message virtual_laser_message;

carmen_rddf_road_profile_message *last_rddf_message = NULL;
static carmen_rddf_road_profile_message *last_rddf_message_copy = NULL;

bool autonomous = false;
bool wait_start_moving = false;
static double last_not_autonomous_timestamp = 0.0;

carmen_behavior_selector_state_message behavior_selector_state_message;

carmen_obstacle_distance_mapper_map_message distance_map;
static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
static carmen_obstacle_distance_mapper_compact_map_message *compact_lane_contents = NULL;

static carmen_mapper_compact_map_message *carmen_mapper_compact_map_msg = NULL;
static carmen_compact_map_t *compact_occupancy_map = NULL;

carmen_prob_models_distance_map 			distance_map2;
carmen_obstacle_distance_mapper_map_message distance_map_free_of_moving_objects;

double original_behaviour_selector_central_lane_obstacles_safe_distance;
double original_model_predictive_planner_obstacles_safe_distance;
double behaviour_selector_annotation_safe_distance_multiplier;

static double carmen_ini_max_velocity;

static bool soft_stop_on = false;

carmen_frenet_path_planner_set_of_paths *current_set_of_paths = NULL;
carmen_moving_objects_point_clouds_message *current_moving_objects = NULL;

extern int frenet_path_planner_num_paths;
extern double frenet_path_planner_paths_displacement;
double frenet_path_planner_time_horizon;
double frenet_path_planner_delta_t;

int use_frenet_path_planner = 0;
int use_unity_simulator = 0;
double in_lane_longitudinal_safety_margin;
double in_lane_longitudinal_safety_margin_with_v_multiplier;
double in_path_longitudinal_safety_margin;
double in_path_longitudinal_safety_margin_with_v_multiplier;

double distance_to_moving_object_with_v_multiplier;
double distance_between_waypoints_with_v_multiplier;

int behavior_selector_performs_path_planning;

carmen_route_planner_road_network_message *road_network_message = NULL;
extern int selected_path_id;
extern double localize_ackerman_initialize_message_timestamp;

int behavior_selector_reverse_driving = 0;
double robot_max_velocity_reverse = 0.0;

double parking_speed_limit;
double move_to_engage_pose_speed_limit;

carmen_map_t occupancy_map;
carmen_map_server_offline_map_message *offline_map = NULL;

carmen_simulator_ackerman_objects_message *carmen_simulator_ackerman_simulated_objects = NULL;

double distance_car_pose_car_front;

carmen_semi_trailer_config_t   semi_trailer_config;
static int argc_global;
static char **argv_global;

double annotation_velocity_bump;
double annotation_velocity_pedestrian_track_stop;
double annotation_velocity_yield;
double annotation_velocity_barrier;

carmen_moving_objects_point_clouds_message *pedestrians_tracked = NULL;
int behavior_selector_check_pedestrian_near_path = 0;
double behavior_pedestrian_near_path_min_lateral_distance = 4.0;
double behavior_selector_pedestrian_near_path_min_longitudinal_distance = 5.0;


int
compute_max_rddf_num_poses_ahead(carmen_robot_and_trailer_traj_point_t current_pose)
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

	last_rddf_message->poses = (carmen_robot_and_trailer_traj_point_t *) realloc(last_rddf_message->poses, sizeof(carmen_robot_and_trailer_traj_point_t) * carmen_rddf_num_poses_ahead);
	last_rddf_message->poses_back = (carmen_robot_and_trailer_traj_point_t *) realloc(last_rddf_message->poses_back, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses_back);
	last_rddf_message->annotations = (int *) realloc(last_rddf_message->annotations, sizeof(int) * carmen_rddf_num_poses_ahead);
	last_rddf_message->annotations_codes = (int *) realloc(last_rddf_message->annotations_codes, sizeof(int) * carmen_rddf_num_poses_ahead);

	memcpy(last_rddf_message->poses, rddf_msg->poses, sizeof(carmen_robot_and_trailer_traj_point_t) * carmen_rddf_num_poses_ahead);
	memcpy(last_rddf_message->poses_back, rddf_msg->poses_back, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses_back);
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

	last_rddf_message->poses = (carmen_robot_and_trailer_traj_point_t *) realloc(last_rddf_message->poses, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses);
	last_rddf_message->poses_back = (carmen_robot_and_trailer_traj_point_t *) realloc(last_rddf_message->poses_back, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses_back);
	last_rddf_message->annotations = (int *) realloc(last_rddf_message->annotations, sizeof(int) * last_rddf_message->number_of_poses);
	last_rddf_message->annotations_codes = (int *) realloc(last_rddf_message->annotations_codes, sizeof(int) * last_rddf_message->number_of_poses);

	memcpy(last_rddf_message->poses, rddf_msg->poses, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses);
	memcpy(last_rddf_message->poses_back, rddf_msg->poses_back, sizeof(carmen_robot_and_trailer_traj_point_t) * last_rddf_message->number_of_poses_back);
	memcpy(last_rddf_message->annotations, rddf_msg->annotations, sizeof(int) * last_rddf_message->number_of_poses);
	memcpy(last_rddf_message->annotations_codes, rddf_msg->annotations_codes, sizeof(int) * last_rddf_message->number_of_poses);

	return (last_rddf_message);
}


carmen_robot_and_trailer_traj_point_t *
compute_simulated_objects(double timestamp)
{
	if (!necessary_maps_available || !current_set_of_paths)
		return (NULL);

	carmen_robot_and_trailer_traj_point_t *poses = current_set_of_paths->rddf_poses_ahead;
	int number_of_poses = current_set_of_paths->number_of_poses;

	static carmen_robot_and_trailer_traj_point_t previous_pose = {0, 0, 0, 0, 0, 0};
	static double previous_timestamp = 0.0;
	static double initial_time = 0.0; // Simulation start time.

	if (initial_time == 0.0)
	{
		previous_pose = poses[number_of_poses / 5];
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

	carmen_robot_and_trailer_traj_point_t pose_ahead;
	pose_ahead.x = previous_pose.x + dx;
	pose_ahead.y = previous_pose.y + dy;

	static carmen_robot_and_trailer_traj_point_t next_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for (int i = 0, n = number_of_poses - 1; i < n; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, poses[i], poses[i + 1], pose_ahead, 0.1);
		if (status == POINT_WITHIN_SEGMENT)
			break;
	}

	previous_pose = next_pose;
	previous_timestamp = timestamp;
	return &next_pose;
}


carmen_robot_and_trailer_traj_point_t *
compute_simulated_lateral_objects(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (!necessary_maps_available || !current_set_of_paths || (current_set_of_paths->number_of_nearby_lanes == 0))
		return (NULL);

	carmen_robot_and_trailer_traj_point_t *poses = current_set_of_paths->rddf_poses_ahead;
	int number_of_poses = current_set_of_paths->number_of_poses;
	if (poses == NULL)
		return (NULL);

	static carmen_robot_and_trailer_traj_point_t previous_pose = {0, 0, 0, 0, 0, 0};
	static carmen_robot_and_trailer_traj_point_t returned_pose = {0, 0, 0, 0, 0, 0};
	static double previous_timestamp = 0.0;
	static double initial_time = 0.0; // Simulation start time.
	static double disp = 2.5;

	if (initial_time == 0.0)
	{
		returned_pose = previous_pose = poses[0];
		returned_pose.x = previous_pose.x + disp * cos(previous_pose.theta + M_PI / 2.0);
		returned_pose.y = previous_pose.y + disp * sin(previous_pose.theta + M_PI / 2.0);

		previous_timestamp = timestamp;
		initial_time = timestamp;

		return (&returned_pose);
	}

	static double stop_t0 = 15.0;
	static double stop_t1 = 15.0;
	static double stop_t2 = 25.0;

	static double v;
	double t = timestamp - initial_time;
	if (stop_t0 <= t && disp > 0.0)
		disp -= 0.03;
	if (t < stop_t1)
//		v = current_robot_pose_v_and_phi.v + 0.9;
		v = current_robot_pose_v_and_phi.v + 0.5; // Motos!
//		v = current_robot_pose_v_and_phi.v + 0.1; // Motos radicais!

	if (t > stop_t2)
	{
		v = current_robot_pose_v_and_phi.v - 1.5;
		if (v < 5.0)
			v = 5.0;
	}

	double dt = timestamp - previous_timestamp;
	double dx = v * dt * cos(previous_pose.theta);
	double dy = v * dt * sin(previous_pose.theta);

	carmen_robot_and_trailer_traj_point_t pose_ahead;
	pose_ahead.x = previous_pose.x + dx;
	pose_ahead.y = previous_pose.y + dy;

	static carmen_robot_and_trailer_traj_point_t next_pose = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < number_of_poses - 1; i++)
	{
		int status;
		next_pose = carmen_get_point_nearest_to_trajectory(&status, poses[i], poses[i + 1], pose_ahead, 0.1);
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
add_simulated_object(carmen_robot_and_trailer_traj_point_t *object_pose)
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


void
add_larger_simulated_object(carmen_robot_and_trailer_traj_point_t *object_pose)
{
	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x;
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y;
	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
	virtual_laser_message.num_positions++;

	double lateral_disp = 0.3;
	double logitudinal_disp;
	for (logitudinal_disp = -10.0; logitudinal_disp < -1.0; logitudinal_disp = logitudinal_disp + 0.2)
	{
		virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x +
				lateral_disp * cos(object_pose->theta + M_PI / 2.0) +
				logitudinal_disp * cos(object_pose->theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y +
				lateral_disp * sin(object_pose->theta + M_PI / 2.0) +
				logitudinal_disp * sin(object_pose->theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x +
				lateral_disp * cos(object_pose->theta - M_PI / 2.0) +
				logitudinal_disp * cos(object_pose->theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y +
				lateral_disp * sin(object_pose->theta - M_PI / 2.0) +
				logitudinal_disp * sin(object_pose->theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;
	}

	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x +
			logitudinal_disp * cos(object_pose->theta);
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y +
			logitudinal_disp * sin(object_pose->theta);
	virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
	virtual_laser_message.num_positions++;
}


void
add_object_to_map(double width, double length, double x, double y, double theta)
{
	double logitudinal_disp = -length / 2.0;
	double lateral_disp = -width / 2.0;
	for ( ; lateral_disp < width / 2.0; lateral_disp = lateral_disp + 0.2)
	{
		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta + M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta + M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta - M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta - M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;
	}

	lateral_disp = width / 2.0;
	for ( ; logitudinal_disp < length / 2.0; logitudinal_disp = logitudinal_disp + 0.2)
	{
		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta + M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta + M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta - M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta - M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;
	}

	for (lateral_disp = -width / 2.0; lateral_disp < width / 2.0; lateral_disp = lateral_disp + 0.2)
	{
		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta + M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta + M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = x +
				lateral_disp * cos(theta - M_PI / 2.0) +
				logitudinal_disp * cos(theta);
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = y +
				lateral_disp * sin(theta - M_PI / 2.0) +
				logitudinal_disp * sin(theta);
		virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_PURPLE;
		virtual_laser_message.num_positions++;
	}
}


void
add_simulator_ackerman_objects_to_map(carmen_simulator_ackerman_objects_message *msg,
		carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi)
{
	if (!msg)
		return;

	for (int index = 0; index < msg->num_objects; index++)
	{
		double width = 0.1;
		double length = 0.1;
		switch (msg->objects[index].type)
		{
		case CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT:
		case CARMEN_SIMULATOR_ACKERMAN_LINE_FOLLOWER:
		case CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT:
			break;

		case CARMEN_SIMULATOR_ACKERMAN_PERSON:
			width = 0.5;
			length = 0.5;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_BIKE:
			width = 0.5;
			length = 1.5;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_CAR:
			width = 1.7;
			length = 4.1;
			break;

		case CARMEN_SIMULATOR_ACKERMAN_TRUCK:
			width = 2.5;
			length = 15.0;
			break;
		}

		if (DIST2D(current_robot_pose_v_and_phi, msg->objects[index]) < 1000.0)
			add_object_to_map(width, length, msg->objects[index].x, msg->objects[index].y, msg->objects[index].theta);
	}
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
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_path_goals_and_annotations_message(carmen_rddf_road_profile_message *path_and_annotations,
		carmen_robot_and_trailer_traj_point_t *goal_list,
		int goal_list_size, double timestamp)
{
	carmen_behavior_selector_path_goals_and_annotations_message path_goals_and_annotations_message;
	path_goals_and_annotations_message.number_of_poses = path_and_annotations->number_of_poses;
	path_goals_and_annotations_message.number_of_poses_back = path_and_annotations->number_of_poses_back;
	path_goals_and_annotations_message.poses = path_and_annotations->poses;
	path_goals_and_annotations_message.poses_back = path_and_annotations->poses_back;
	path_goals_and_annotations_message.annotations = path_and_annotations->annotations;
	path_goals_and_annotations_message.annotations_codes = path_and_annotations->annotations_codes;

	path_goals_and_annotations_message.goal_list = goal_list;
	path_goals_and_annotations_message.goal_list_size = goal_list_size;

	path_goals_and_annotations_message.timestamp = timestamp;
	path_goals_and_annotations_message.host = carmen_get_host();

	carmen_behavior_selector_publish_path_goals_and_annotations_message(&path_goals_and_annotations_message);
}


void
publish_set_of_paths_message(carmen_frenet_path_planner_set_of_paths *set_of_paths)
{
	carmen_frenet_path_planner_publish_set_of_paths_message(set_of_paths);
}


void
publish_updated_lane_contents(double timestamp)
{
	carmen_obstacle_distance_mapper_compact_map_message compact_lane_contents_cpy;
	carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(&compact_lane_contents_cpy, compact_lane_contents);
	clear_moving_obstacles_from_compact_lane_map(&compact_lane_contents_cpy);

	carmen_behaviour_selector_publish_compact_lane_contents_message(&compact_lane_contents_cpy, timestamp);

	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_lane_contents_cpy);
}


void
publish_current_state(carmen_behavior_selector_state_message *msg)
{
	IPC_RETURN_TYPE err;
	carmen_behavior_selector_task_t current_task;
	carmen_behavior_selector_algorithm_t following_lane_planner;
	carmen_behavior_selector_algorithm_t parking_planner;

	behavior_selector_get_full_state(&current_task, &following_lane_planner, &parking_planner);

	msg->task = current_task;
	msg->algorithm = get_current_algorithm();

	if (road_network_message)
	{
		msg->route_planner_state = road_network_message->route_planner_state;
		msg->offroad_planner_request = road_network_message->offroad_planner_request;
	}
	else
	{
		msg->route_planner_state = IDLE;
		msg->offroad_planner_request = NO_REQUEST;
	}

	msg->timestamp = carmen_get_time();
	msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
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


void
publish_new_best_path(int best_path, double timestamp)
{
	if (!behavior_selector_performs_path_planning)
	{
		carmen_frenet_path_planner_selected_path selected_path_message;

		selected_path_message.selected_path = best_path;
		selected_path_message.timestamp = timestamp;
		selected_path_message.host = carmen_get_host();

		carmen_frenet_path_planner_publish_selected_path_message(&selected_path_message);
	}
	else
		selected_path_id = best_path;
}
///////////////////////////////////////////////////////////////////////////////////////////////


carmen_robot_and_trailer_traj_point_t *
check_soft_stop(carmen_robot_and_trailer_traj_point_t *first_goal, carmen_robot_and_trailer_traj_point_t *goal_list, int &goal_type)
{
	static bool soft_stop_in_progress = false;
	static carmen_robot_and_trailer_traj_point_t soft_stop_goal;
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
set_behaviours_parameters(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	if (behavior_selector_state_message.low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE)
	{
		get_robot_config()->model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance * behaviour_selector_annotation_safe_distance_multiplier;
		get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance =
				original_behaviour_selector_central_lane_obstacles_safe_distance -
				original_model_predictive_planner_obstacles_safe_distance +
				original_model_predictive_planner_obstacles_safe_distance * behaviour_selector_annotation_safe_distance_multiplier;
		if (behavior_selector_use_symotha)
		{
			udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance);
			udatmo_set_model_predictive_planner_obstacles_safe_distance(get_robot_config()->model_predictive_planner_obstacles_safe_distance);
		}
	}
	else
	{
		get_robot_config()->behaviour_selector_central_lane_obstacles_safe_distance = original_behaviour_selector_central_lane_obstacles_safe_distance;
		get_robot_config()->model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance;
		if (behavior_selector_use_symotha)
		{
			udatmo_set_behaviour_selector_central_lane_obstacles_safe_distance(original_behaviour_selector_central_lane_obstacles_safe_distance);
			udatmo_set_model_predictive_planner_obstacles_safe_distance(original_model_predictive_planner_obstacles_safe_distance);
		}
	}

	double distance_between_waypoints = param_distance_between_waypoints;
	double change_goal_distance = param_change_goal_distance;
	if (fabs(current_robot_pose_v_and_phi.v) > param_distance_interval)
	{
		if ((distance_between_waypoints_with_v_multiplier * fabs(current_robot_pose_v_and_phi.v)) > distance_between_waypoints)
			distance_between_waypoints = distance_between_waypoints_with_v_multiplier * fabs(current_robot_pose_v_and_phi.v);
		if ((distance_between_waypoints_with_v_multiplier * fabs(current_robot_pose_v_and_phi.v)) > change_goal_distance)
			change_goal_distance = distance_between_waypoints_with_v_multiplier * fabs(current_robot_pose_v_and_phi.v);
	}

	carmen_behavior_selector_task_t current_task = behavior_selector_get_task();
	if ((current_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
		(current_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER))
	{
		distance_between_waypoints /= 5.0;
		change_goal_distance /= 5.0;
	}
	change_distance_between_waypoints_and_goals(distance_between_waypoints, change_goal_distance);

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
		selected_path_id = frenet_path_planner_num_paths / 2;
		localize_ackerman_initialize_message_timestamp = carmen_get_time(); // isso forcca o frenet a reiniciar os paths na pose do robot
	}
	else if (carmen_get_time() - last_not_autonomous_timestamp < 3.0)
		wait_start_moving = true;
	else if (wait_start_moving && fabs(current_robot_pose_v_and_phi.v) > 0.15)
		wait_start_moving = false;
}


void
remove_moving_objects_from_distance_map_old(carmen_route_planner_road_network_message *road_network_message)
{
	if (!road_network_message)
		return;

	virtual_laser_message.num_positions = 0;
	for (int i = 0; i < road_network_message->number_of_nearby_lanes; i++)
	{
		carmen_robot_and_trailer_traj_point_t *lane = &(road_network_message->nearby_lanes[road_network_message->nearby_lanes_indexes[i]]);
		int *traffic_restrictions = &(road_network_message->traffic_restrictions[road_network_message->nearby_lanes_indexes[i]]);
		int lane_size = road_network_message->nearby_lanes_sizes[i];
		for (int j = 0; j < lane_size - 1; j++)
		{
			double lane_left_width = ROUTE_PLANNER_GET_LANE_LEFT_WIDTH(traffic_restrictions[j]);
			double lane_right_width = ROUTE_PLANNER_GET_LANE_RIGHT_WIDTH(traffic_restrictions[j]);
			for (double s = 0.0; s < DIST2D(lane[j], lane[j + 1]); s += distance_map.config.resolution * 0.5)
			{
				double lane_x = lane[j].x + s * cos(lane[j].theta);
				double lane_y = lane[j].y + s * sin(lane[j].theta);
				for (double d = -lane_right_width; d < lane_left_width; d += distance_map.config.resolution * 0.5)
				{
					double x = lane_x + d * cos(lane[j].theta + M_PI / 2.0);
					double y = lane_y + d * sin(lane[j].theta + M_PI / 2.0);

					// Move global path point coordinates to map coordinates
					int x_map_cell = (int) round((x - distance_map.config.x_origin) / distance_map.config.resolution);
					int y_map_cell = (int) round((y - distance_map.config.y_origin) / distance_map.config.resolution);
					if ((x_map_cell < 0 || x_map_cell >= distance_map.config.x_size) || (y_map_cell < 0 || y_map_cell >= distance_map.config.y_size))
						continue;

					virtual_laser_message.positions[virtual_laser_message.num_positions].x = x;
					virtual_laser_message.positions[virtual_laser_message.num_positions].y = y;
					virtual_laser_message.colors[virtual_laser_message.num_positions] = CARMEN_BLUE;
					virtual_laser_message.num_positions++;

					int index = y_map_cell + distance_map.config.y_size * x_map_cell;
					distance_map.complete_x_offset[index] = DISTANCE_MAP_HUGE_DISTANCE;
					distance_map.complete_y_offset[index] = DISTANCE_MAP_HUGE_DISTANCE;
				}
			}
		}
	}
}


void
create_carmen_obstacle_distance_mapper_map_message(carmen_obstacle_distance_mapper_map_message &distance_map_free_of_moving_objects,
		carmen_prob_models_distance_map distance_map2, double timestamp)
{
	distance_map_free_of_moving_objects.config = distance_map2.config;
	distance_map_free_of_moving_objects.size = distance_map2.config.x_size * distance_map2.config.y_size;
	distance_map_free_of_moving_objects.complete_x_offset = distance_map2.complete_x_offset;
	distance_map_free_of_moving_objects.complete_y_offset = distance_map2.complete_y_offset;
	distance_map_free_of_moving_objects.host = carmen_get_host();
	distance_map_free_of_moving_objects.timestamp = timestamp;
}


void
set_path_using_symotha(const carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	static carmen_frenet_path_planner_set_of_paths set_of_paths;

	if (behavior_selector_performs_path_planning)
	{
		if ((road_network_message->number_of_poses != 0) && (road_network_message->poses != NULL))
		{
			set_of_paths = frenet_path_planner_build_frenet_path_plan(road_network_message->poses, road_network_message->poses_back,
				road_network_message->number_of_poses, road_network_message->number_of_poses_back, current_robot_pose_v_and_phi,
				road_network_message->annotations, road_network_message->annotations_codes, road_network_message,
				&behavior_selector_state_message, timestamp);
			current_set_of_paths = &set_of_paths;
		}
		else
			current_set_of_paths = NULL;
	}

	compact_occupancy_map = obstacle_distance_mapper_uncompress_occupancy_map(&occupancy_map, compact_occupancy_map, carmen_mapper_compact_map_msg);
//	current_moving_objects = obstacle_distance_mapper_datmo(road_network_message, occupancy_map, offline_map, carmen_mapper_compact_map_msg->timestamp);
	if (current_moving_objects)
	{
		carmen_moving_objects_point_clouds_publish_message(current_moving_objects);
		obstacle_distance_mapper_remove_moving_objects_from_occupancy_map(&occupancy_map, current_moving_objects);
	}
	if (distance_map2.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&distance_map2, distance_map.config);
	carmen_prob_models_create_distance_map(&distance_map2, &occupancy_map, obstacle_probability_threshold);
	create_carmen_obstacle_distance_mapper_map_message(distance_map_free_of_moving_objects, distance_map2, timestamp);
//	obstacle_distance_mapper_restore_moving_objects_to_occupancy_map(&occupancy_map, current_moving_objects);

	if (use_frenet_path_planner)
		set_optimum_path(current_set_of_paths, current_moving_objects, current_robot_pose_v_and_phi, 0,
				behavior_selector_state_message, timestamp);
//		set_optimum_path(current_set_of_paths, current_robot_pose_v_and_phi, who_set_the_goal_v, timestamp);
	obstacle_distance_mapper_free_moving_objects_message(current_moving_objects);

	if (behavior_selector_performs_path_planning)
	{
		set_of_paths.selected_path = selected_path_id;
		publish_set_of_paths_message(&set_of_paths);

		static carmen_rddf_road_profile_message rddf_msg;
		rddf_msg.poses = &(set_of_paths.set_of_paths[set_of_paths.selected_path * set_of_paths.number_of_poses]);
		rddf_msg.poses_back = set_of_paths.poses_back;
		rddf_msg.number_of_poses = set_of_paths.number_of_poses;
		rddf_msg.number_of_poses_back = set_of_paths.number_of_poses_back;
		rddf_msg.annotations = road_network_message->annotations;
		rddf_msg.annotations_codes = road_network_message->annotations_codes;
		rddf_msg.timestamp = road_network_message->timestamp;
		rddf_msg.host = carmen_get_host();

		if (param_rddf_num_poses_by_car_velocity)
			last_rddf_message = copy_rddf_message_considering_velocity(last_rddf_message, &rddf_msg);
		else
			last_rddf_message = copy_rddf_message(last_rddf_message, &rddf_msg);

//		carmen_rddf_publish_road_profile_message(rddf_msg.poses, rddf_msg.poses_back, rddf_msg.number_of_poses, rddf_msg.number_of_poses_back,
//				rddf_msg.annotations, rddf_msg.annotations_codes);
//
//		// Copia rddf_msg para a global last_rddf_message
//		behavior_selector_motion_planner_publish_path_message(&rddf_msg, param_rddf_num_poses_by_car_velocity);
//
//		last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
//		if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
//			publish_behavior_selector_road_profile_message(&rddf_msg);

		free(set_of_paths.set_of_paths);
	}
}


int
select_behaviour_using_symotha(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static carmen_robot_and_trailer_traj_point_t last_valid_goal;
	static carmen_robot_and_trailer_traj_point_t *last_valid_goal_p = NULL;

	carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, compact_lane_contents);

	set_behaviours_parameters(current_robot_pose_v_and_phi, timestamp);

	set_path_using_symotha(current_robot_pose_v_and_phi, behavior_selector_state_message, timestamp);

	// Esta funcao altera a mensagem de rddf e funcoes abaixo dela precisam da original
	last_rddf_message_copy = copy_rddf_message(last_rddf_message_copy, last_rddf_message);

	int error = run_decision_making_state_machine(&behavior_selector_state_message, current_robot_pose_v_and_phi,
			path_collision_info_t {}, last_valid_goal_p, timestamp);
	if (error != 0)
		carmen_die("Behaviour Selector state machine error. State machine error code %d\n", error);

	int goal_list_size;
	carmen_robot_and_trailer_traj_point_t *first_goal;
	int goal_type;
	carmen_robot_and_trailer_traj_point_t *goal_list = set_goal_list(goal_list_size, first_goal, goal_type, last_rddf_message_copy,
			path_collision_info_t {}, current_moving_objects, behavior_selector_state_message, timestamp);

	first_goal = check_soft_stop(first_goal, goal_list, goal_type);

	int who_set_the_goal_v = NONE;
	if (goal_list_size > 0)
	{
		who_set_the_goal_v = set_goal_velocity(first_goal, &current_robot_pose_v_and_phi, goal_type, last_rddf_message_copy,
				path_collision_info_t {}, behavior_selector_state_message, timestamp);
		publish_path_goals_and_annotations_message(last_rddf_message_copy, goal_list, goal_list_size, timestamp);

		last_valid_goal = *first_goal;
		last_valid_goal_p = &last_valid_goal;
	}
	else if (last_valid_goal_p != NULL)
	{	// Garante parada suave ao fim do rddf
		last_valid_goal_p->v = 0.0;
		publish_path_goals_and_annotations_message(last_rddf_message_copy, last_valid_goal_p, 1, timestamp);
	}

	publish_updated_lane_contents(timestamp);
	publish_current_state(&behavior_selector_state_message);

// Control whether simulated moving obstacles are created by (un)commenting the
// definition of the macro below at the top of this file.
#ifdef SIMULATE_MOVING_OBSTACLE
	carmen_robot_and_trailer_traj_point_t *simulated_object_pose = compute_simulated_objects(timestamp);
	if (simulated_object_pose)
		add_simulated_object(simulated_object_pose);
#endif
#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
	carmen_robot_and_trailer_traj_point_t *simulated_object_pose2 = compute_simulated_lateral_objects(current_robot_pose_v_and_phi, timestamp);
	if (simulated_object_pose2)
		add_larger_simulated_object(simulated_object_pose2);
//		add_simulated_object(simulated_object_pose2);
#endif

	add_simulator_ackerman_objects_to_map(carmen_simulator_ackerman_simulated_objects, current_robot_pose_v_and_phi);

	if (virtual_laser_message.num_positions >= 0)
		publish_simulated_objects();

	return (who_set_the_goal_v);
}


void
print_road_network_behavior_selector(carmen_route_planner_road_network_message *road_network)
{
	printf("\n");

	for (int i = 0; i < road_network->number_of_nearby_lanes; i++)
	{
		printf("lane_id %d, size %d\n", road_network->nearby_lanes_ids[i], road_network->nearby_lanes_sizes[i]);

		carmen_route_planner_junction_t *lane_merges = &(road_network->nearby_lanes_merges[road_network->nearby_lanes_merges_indexes[i]]);
		for (int j = 0; j < road_network->nearby_lanes_merges_sizes[i]; j++)
			printf("lane_id %d, merge %d: target_lane_id %d, node_id %d, index_of_node_in_current_lane %d, target_node_index_in_nearby_lane %d\n",
					road_network->nearby_lanes_ids[i], j,
					lane_merges[j].target_lane_id, lane_merges[j].node_id,
					lane_merges[j].index_of_node_in_current_lane, lane_merges[j].target_node_index_in_nearby_lane);

		carmen_route_planner_junction_t *lane_forks = &(road_network->nearby_lanes_forks[road_network->nearby_lanes_forks_indexes[i]]);
		for (int j = 0; j < road_network->nearby_lanes_forks_sizes[i]; j++)
			printf("lane_id %d, fork %d: target_lane_id %d, node_id %d, index_of_node_in_current_lane %d, target_node_index_in_nearby_lane %d\n",
					road_network->nearby_lanes_ids[i], j,
					lane_forks[j].target_lane_id, lane_forks[j].node_id,
					lane_forks[j].index_of_node_in_current_lane, lane_forks[j].target_node_index_in_nearby_lane);
	}

	printf("\n");
}


//static void
//print_poses(carmen_robot_and_trailer_traj_point_t *poses, int number_of_poses, char *filename)
//{
//	FILE *arq = fopen(filename, "w");
//	for (int i = 0; i < 10000 && i < number_of_poses; i++)
//		fprintf(arq, "%lf %lf %lf %lf %lf\n",
//				poses[i].x, poses[i].y, poses[i].theta, poses[i].phi, poses[i].v);
//	fclose(arq);
//}


path_collision_info_t
set_path(const carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi,
		carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	static carmen_frenet_path_planner_set_of_paths set_of_paths;

	if (behavior_selector_performs_path_planning)
	{
		if ((road_network_message->number_of_poses != 0) && (road_network_message->poses != NULL))
		{
			set_of_paths = frenet_path_planner_build_frenet_path_plan(road_network_message->poses, road_network_message->poses_back,
				road_network_message->number_of_poses, road_network_message->number_of_poses_back, current_robot_pose_v_and_phi,
				road_network_message->annotations, road_network_message->annotations_codes, road_network_message,
				&behavior_selector_state_message, timestamp);
			current_set_of_paths = &set_of_paths;
		}
		else
			current_set_of_paths = NULL;
	}

	distance_map_free_of_moving_objects = distance_map; // O distance_map vem sem objetos móveis do obstacle_distance_mapper

//	print_poses(&(set_of_paths.set_of_paths[set_of_paths.selected_path * set_of_paths.number_of_poses]), rddf_msg.number_of_poses, (char *) "cacox0.txt");

//	print_road_network_behavior_selector(road_network_message);
	vector<path_collision_info_t> paths_collision_info;
	if (use_frenet_path_planner)
		paths_collision_info = set_optimum_path(current_set_of_paths, current_moving_objects, current_robot_pose_v_and_phi,
				0, behavior_selector_state_message, timestamp);
//		set_optimum_path(current_set_of_paths, current_robot_pose_v_and_phi, who_set_the_goal_v, timestamp);

	if (behavior_selector_performs_path_planning && current_set_of_paths)
	{
		set_of_paths.selected_path = selected_path_id;
		publish_set_of_paths_message(&set_of_paths);

		static carmen_rddf_road_profile_message rddf_msg;
		rddf_msg.poses = &(set_of_paths.set_of_paths[set_of_paths.selected_path * set_of_paths.number_of_poses]);
		rddf_msg.poses_back = set_of_paths.poses_back;
		rddf_msg.number_of_poses = set_of_paths.number_of_poses;
		rddf_msg.number_of_poses_back = set_of_paths.number_of_poses_back;

		if ((behavior_selector_get_task() == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) && (selected_path_id == frenet_path_planner_num_paths / 2))
			for (int i = 0; i < rddf_msg.number_of_poses; i++)
				rddf_msg.poses[i].beta = road_network_message->poses[i].beta;

		if (!road_network_message->annotations && (set_of_paths.number_of_poses != 0))
		{
			rddf_msg.annotations = (int *) malloc(set_of_paths.number_of_poses * sizeof(int));
			rddf_msg.annotations_codes = (int *) malloc(set_of_paths.number_of_poses * sizeof(int));
		}
		else
		{
			rddf_msg.annotations = road_network_message->annotations;
			rddf_msg.annotations_codes = road_network_message->annotations_codes;
		}
		rddf_msg.timestamp = road_network_message->timestamp;
		rddf_msg.host = carmen_get_host();

		if (param_rddf_num_poses_by_car_velocity)
			last_rddf_message = copy_rddf_message_considering_velocity(last_rddf_message, &rddf_msg);
		else
			last_rddf_message = copy_rddf_message(last_rddf_message, &rddf_msg);

		if (!road_network_message->annotations && (set_of_paths.number_of_poses != 0))
		{
			free(rddf_msg.annotations);
			free(rddf_msg.annotations_codes);
		}
//		carmen_rddf_publish_road_profile_message(rddf_msg.poses, rddf_msg.poses_back, rddf_msg.number_of_poses, rddf_msg.number_of_poses_back,
//				rddf_msg.annotations, rddf_msg.annotations_codes);
//
//		// Copia rddf_msg para a global last_rddf_message
//		behavior_selector_motion_planner_publish_path_message(&rddf_msg, param_rddf_num_poses_by_car_velocity);
//
//		last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
//		if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
//			publish_behavior_selector_road_profile_message(&rddf_msg);

		free(set_of_paths.set_of_paths);
	}

	if ((int) paths_collision_info.size() > selected_path_id)
		return (paths_collision_info[selected_path_id]);
	else
		return (path_collision_info_t {});
}


int
select_behaviour(carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi, double timestamp)
{
	static carmen_robot_and_trailer_traj_point_t last_valid_goal;
	static carmen_robot_and_trailer_traj_point_t *last_valid_goal_p = NULL;

	set_behaviours_parameters(current_robot_pose_v_and_phi, timestamp);

//	double t2 = carmen_get_time();

	path_collision_info_t path_collision_info = set_path(current_robot_pose_v_and_phi, behavior_selector_state_message, timestamp);
//	set_path(current_robot_pose_v_and_phi, timestamp); path_collision_info_t path_collision_info = {};

//	printf("delta_t funcao %0.3lf\n", carmen_get_time() - t2);

//	print_poses(last_rddf_message->poses, last_rddf_message->number_of_poses, (char *) "cacox0.txt");

	if (!last_rddf_message)
	{
		publish_current_state(&behavior_selector_state_message);
		return (NONE);
	}
	// Esta funcao altera a mensagem de rddf e funcoes abaixo dela precisam da original
	last_rddf_message_copy = copy_rddf_message(last_rddf_message_copy, last_rddf_message);

	int error = run_decision_making_state_machine(&behavior_selector_state_message, current_robot_pose_v_and_phi,
			path_collision_info, last_valid_goal_p, timestamp);
	if (error != 0)
		carmen_die("Behaviour Selector state machine error. State machine error code %d\n", error);

	int goal_list_size;
	carmen_robot_and_trailer_traj_point_t *first_goal;
	int goal_type;
	carmen_robot_and_trailer_traj_point_t *goal_list = set_goal_list(goal_list_size, first_goal, goal_type, last_rddf_message_copy,
			path_collision_info, current_moving_objects, behavior_selector_state_message, timestamp);

	first_goal = check_soft_stop(first_goal, goal_list, goal_type);

	int who_set_the_goal_v = NONE;
	if (goal_list_size > 0)
	{
		who_set_the_goal_v = set_goal_velocity(first_goal, &current_robot_pose_v_and_phi, goal_type, last_rddf_message_copy,
				path_collision_info, behavior_selector_state_message, timestamp);

		publish_path_goals_and_annotations_message(last_rddf_message_copy, goal_list, goal_list_size, timestamp);

		last_valid_goal = *first_goal;
		last_valid_goal_p = &last_valid_goal;
	}
	else if (last_valid_goal_p != NULL)
	{	// Garante parada suave ao fim do rddf
		last_valid_goal_p->v = 0.0;

		publish_path_goals_and_annotations_message(last_rddf_message_copy, last_valid_goal_p, 1, timestamp);
	}

	publish_current_state(&behavior_selector_state_message);

// Control whether simulated moving obstacles are created by (un)commenting the
// definition of the macro below at the top of this file.
#ifdef SIMULATE_MOVING_OBSTACLE
	carmen_robot_and_trailer_traj_point_t *simulated_object_pose = compute_simulated_objects(timestamp);
	if (simulated_object_pose)
		add_simulated_object(simulated_object_pose);
#endif
#ifdef SIMULATE_LATERAL_MOVING_OBSTACLE
	carmen_robot_and_trailer_traj_point_t *simulated_object_pose2 = compute_simulated_lateral_objects(current_robot_pose_v_and_phi, timestamp);
	if (simulated_object_pose2)
		add_larger_simulated_object(simulated_object_pose2);
//		add_simulated_object(simulated_object_pose2);
#endif

	add_simulator_ackerman_objects_to_map(carmen_simulator_ackerman_simulated_objects, current_robot_pose_v_and_phi);

	if (virtual_laser_message.num_positions >= 0)
		publish_simulated_objects();

	return (who_set_the_goal_v);
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
//	static double t = carmen_get_time();

	if (!necessary_maps_available || (!behavior_selector_performs_path_planning && !last_rddf_message))
		return;

	if (behavior_selector_performs_path_planning && !road_network_message)
		return;

	carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->globalpos.x;
	current_robot_pose_v_and_phi.y = msg->globalpos.y;
	current_robot_pose_v_and_phi.theta = msg->globalpos.theta;
	current_robot_pose_v_and_phi.beta = msg->beta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

//	double t2 = carmen_get_time();
	if (behavior_selector_use_symotha)
		select_behaviour_using_symotha(current_robot_pose_v_and_phi, msg->timestamp);
	else
		select_behaviour(current_robot_pose_v_and_phi, msg->timestamp);

//	printf("delta_t handler total %0.3lf, delta_t funcao %0.3lf\n",
//			carmen_get_time() - t, carmen_get_time() - t2);
//
//	t = carmen_get_time();

	if (msg->semi_trailer_type != semi_trailer_config.type)
	{
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc_global, argv_global, msg->semi_trailer_type);
		carmen_collision_detection_set_semi_trailer_type(semi_trailer_config.type);
	}
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	if (!necessary_maps_available || (!behavior_selector_performs_path_planning && !last_rddf_message))
		return;

	if (behavior_selector_performs_path_planning && !road_network_message)
		return;

	carmen_robot_and_trailer_traj_point_t current_robot_pose_v_and_phi;

	current_robot_pose_v_and_phi.x = msg->truepose.x;
	current_robot_pose_v_and_phi.y = msg->truepose.y;
	current_robot_pose_v_and_phi.theta = msg->truepose.theta;
	current_robot_pose_v_and_phi.beta = msg->beta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	if (behavior_selector_use_symotha)
		select_behaviour_using_symotha(current_robot_pose_v_and_phi, msg->timestamp);
	else
		select_behaviour(current_robot_pose_v_and_phi, msg->timestamp);
}


static void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *msg)
{
	road_network_message = msg;
	for (int m = 0; m < road_network_message->number_of_nearby_lanes; m++)
	{
		int lane = road_network_message->nearby_lanes_ids[m];
		for (int n = m + 1; n < road_network_message->number_of_nearby_lanes; n++)
		{
			if (road_network_message->nearby_lanes_ids[n] == lane)
			{
				for (int i = n; i < road_network_message->number_of_nearby_lanes - 1; i++)
				{
					road_network_message->nearby_lanes_ids[i] = road_network_message->nearby_lanes_ids[i + 1];
					road_network_message->nearby_lanes_sizes[i] = road_network_message->nearby_lanes_sizes[i + 1];
					road_network_message->nearby_lanes_indexes[i] = road_network_message->nearby_lanes_indexes[i + 1];
				}
				road_network_message->number_of_nearby_lanes--;
			}
		}
	}
}


static void
rddf_road_profile_message_handler(carmen_rddf_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	if (param_rddf_num_poses_by_car_velocity)
		last_rddf_message = copy_rddf_message_considering_velocity(last_rddf_message, rddf_msg);
	else
		last_rddf_message = copy_rddf_message(last_rddf_message, rddf_msg);

//	behavior_selector_motion_planner_publish_path_message(rddf_msg, param_rddf_num_poses_by_car_velocity);
//	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
//
//	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
//		publish_behavior_selector_road_profile_message(rddf_msg);
}


static void
frenet_path_planner_set_of_paths_message_handler(carmen_frenet_path_planner_set_of_paths *set_of_paths_message)
{
	current_set_of_paths = set_of_paths_message;
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_message)
{
	current_moving_objects = moving_objects_message;
}


static void
carmen_mapper_compact_map_message_handler(carmen_mapper_compact_map_message *message)
{
	carmen_mapper_compact_map_msg = message;
}


static void
carmen_map_server_offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	offline_map = msg;
}


static void
carmen_simulator_ackerman_objects_message_handler(carmen_simulator_ackerman_objects_message *msg)
{
	carmen_simulator_ackerman_simulated_objects = msg;
}


//static void
//path_planner_road_profile_handler(carmen_path_planner_road_profile_message *rddf_msg)
//{
//	if (!necessary_maps_available)
//		return;
//
//	behavior_selector_motion_planner_publish_path_message((carmen_rddf_road_profile_message *) rddf_msg, 0);
//	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;
//
//	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL)
//	{
//		carmen_behavior_selector_road_profile_message msg;
//		msg.annotations = rddf_msg->annotations;
//		msg.number_of_poses = rddf_msg->number_of_poses;
//		msg.number_of_poses_back = rddf_msg->number_of_poses_back;
//		msg.poses = rddf_msg->poses;
//		msg.poses_back = rddf_msg->poses_back;
//		msg.timestamp = carmen_get_time();
//		msg.host = carmen_get_host();
//
//		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &msg);
//		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);
//	}
//}


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
	if (behavior_selector_use_symotha && (compact_lane_contents == NULL))
		return;

	if (compact_distance_map == NULL)
	{
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		if (!distance_map.complete_x_offset)
			carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
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
		compact_lane_contents = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		if (!distance_map.complete_x_offset)
			carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
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
	behavior_selector_set_algorithm(msg->algorithm, msg->task);
}


//static void
//set_goal_source_handler(carmen_behavior_selector_set_goal_source_message *msg)
//{
//	behavior_selector_set_goal_source(msg->goal_source);
//}


static void
set_task_handler(carmen_behavior_selector_set_task_message *msg)
{
	behavior_selector_set_task(msg->task);
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
		else
		{
			set_max_v(strtod(message->command, NULL));
			soft_stop_on = false;
		}

		printf("New speed set by voice command: %lf\n", get_max_v());
		fflush(stdout);
	}
}


static void
carmen_localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	localize_ackerman_initialize_message_timestamp = initialize_msg->timestamp;
}


static void
moving_objects_point_clouds_message_handler_0(carmen_moving_objects_point_clouds_message *msg)
{
	pedestrians_tracked = msg;
}


static void
task_manager_set_collision_geometry_message_handler(carmen_task_manager_set_collision_geometry_message *msg)
{
	if (msg->geometry == ENGAGE_GEOMETRY)
		behavior_selector_state_message.low_level_state_flags |= CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY;
	else if (msg->geometry == DEFAULT_GEOMETRY)
		behavior_selector_state_message.low_level_state_flags &= ~CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY;

	carmen_collision_detection_set_robot_collision_config(msg->geometry);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("behaviour_selector: disconnected.\n");

		exit(0);
	}
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
	carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(NULL, (carmen_handler_t) obstacle_avoider_robot_hit_obstacle_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (!behavior_selector_performs_path_planning)
		carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_road_profile_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_frenet_path_planner_subscribe_set_of_paths_message(NULL, (carmen_handler_t) frenet_path_planner_set_of_paths_message_handler, CARMEN_SUBSCRIBE_LATEST);

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

//    carmen_subscribe_message((char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_NAME,
//			(char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_FMT,
//			NULL, sizeof (carmen_path_planner_road_profile_message),
//			(carmen_handler_t)path_planner_road_profile_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT,
			NULL, sizeof(carmen_behavior_selector_set_algorithm_message),
			(carmen_handler_t)set_algorith_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME,
//			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_FMT,
//			NULL, sizeof(carmen_behavior_selector_set_goal_source_message),
//			(carmen_handler_t)set_goal_source_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_TASK_FMT,
			NULL, sizeof(carmen_behavior_selector_set_task_message),
			(carmen_handler_t)set_task_handler, CARMEN_SUBSCRIBE_LATEST);

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

	if (behavior_selector_performs_path_planning)
	{
		carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) carmen_localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	if (behavior_selector_use_symotha)
		carmen_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) carmen_map_server_offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_objects_message(NULL, (carmen_handler_t) (carmen_simulator_ackerman_objects_message_handler), CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message_generic(0, NULL, (carmen_handler_t) moving_objects_point_clouds_message_handler_0, CARMEN_SUBSCRIBE_LATEST);

	carmen_task_manager_subscribe_set_collision_geometry_message(NULL, (carmen_handler_t) task_manager_set_collision_geometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);

//	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, IPC_VARIABLE_LENGTH,
//			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT);
//	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
//
//	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME, IPC_VARIABLE_LENGTH,
//			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_FMT);
//	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME);

    err = IPC_defineMsg(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
    		CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);

    err = IPC_defineMsg(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
    		CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME);

    carmen_frenet_path_planner_define_messages();

    carmen_moving_objects_point_clouds_define_messages();
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

		{(char *) "robot", (char *) "desired_decelaration_forward",	 CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_forward,					1, NULL},
		{(char *) "robot", (char *) "desired_decelaration_reverse",	 CARMEN_PARAM_DOUBLE, &robot_config.desired_decelaration_reverse,					1, NULL},
		{(char *) "robot", (char *) "desired_acceleration",			 CARMEN_PARAM_DOUBLE, &robot_config.desired_acceleration,							1, NULL},
		{(char *) "robot", (char *) "desired_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.desired_steering_command_rate,					1, NULL},
		{(char *) "robot", (char *) "understeer_coeficient",		 CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL},
		{(char *) "robot", (char *) "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 					1, NULL},

		{(char *) "robot", (char *) "annotation_velocity_bump",					 CARMEN_PARAM_DOUBLE, &annotation_velocity_bump,				  0, NULL},
		{(char *) "robot", (char *) "annotation_velocity_pedestrian_track_stop", CARMEN_PARAM_DOUBLE, &annotation_velocity_pedestrian_track_stop, 0, NULL},
		{(char *) "robot", (char *) "annotation_velocity_yield",				 CARMEN_PARAM_DOUBLE, &annotation_velocity_yield,				  0, NULL},
		{(char *) "robot", (char *) "annotation_velocity_barrier",				 CARMEN_PARAM_DOUBLE, &annotation_velocity_barrier,				  0, NULL},

		{(char *) "robot", (char *) "parking_speed_limit", CARMEN_PARAM_DOUBLE, &parking_speed_limit, 1, NULL},
		{(char *) "robot", (char *) "move_to_engage_pose_speed_limit", CARMEN_PARAM_DOUBLE, &move_to_engage_pose_speed_limit, 1, NULL},
		{(char *) "robot", (char *) "max_velocity_reverse", CARMEN_PARAM_DOUBLE, &robot_max_velocity_reverse, 1, NULL},
		{(char *) "semi_trailer",	   (char *) "initial_type", CARMEN_PARAM_INT,	 &semi_trailer_config.type,								 0, NULL},
		{(char *) "behavior_selector", (char *) "use_symotha", CARMEN_PARAM_ONOFF, &behavior_selector_use_symotha, 1, NULL},
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
		{(char *) "behavior_selector", (char *) "annotation_safe_distance_multiplier", CARMEN_PARAM_DOUBLE, &behaviour_selector_annotation_safe_distance_multiplier, 0, NULL},
		{(char *) "behavior_selector", (char *) "goal_velocity_tuning_factor", CARMEN_PARAM_DOUBLE, &robot_config.behaviour_selector_goal_velocity_tuning_factor, 0, NULL},
		{(char *) "behavior_selector", (char *) "in_lane_longitudinal_safety_margin", CARMEN_PARAM_DOUBLE, &in_lane_longitudinal_safety_margin, 0, NULL},
		{(char *) "behavior_selector", (char *) "in_lane_longitudinal_safety_margin_with_v_multiplier", CARMEN_PARAM_DOUBLE, &in_lane_longitudinal_safety_margin_with_v_multiplier, 0, NULL},
		{(char *) "behavior_selector", (char *) "in_path_longitudinal_safety_margin", CARMEN_PARAM_DOUBLE, &in_path_longitudinal_safety_margin, 0, NULL},
		{(char *) "behavior_selector", (char *) "in_path_longitudinal_safety_margin_with_v_multiplier", CARMEN_PARAM_DOUBLE, &in_path_longitudinal_safety_margin_with_v_multiplier, 0, NULL},
		{(char *) "behavior_selector", (char *) "distance_to_moving_object_with_v_multiplier", CARMEN_PARAM_DOUBLE, &distance_to_moving_object_with_v_multiplier, 0, NULL},
		{(char *) "behavior_selector", (char *) "distance_between_waypoints_with_v_multiplier", CARMEN_PARAM_DOUBLE, &distance_between_waypoints_with_v_multiplier, 0, NULL},
		{(char *) "behavior_selector", (char *) "performs_path_planning", CARMEN_PARAM_ONOFF, &behavior_selector_performs_path_planning, 0, NULL},
		{(char *) "behavior_selector", (char *) "reverse_driving", CARMEN_PARAM_ONOFF, &behavior_selector_reverse_driving, 0, NULL},
		{(char *) "behavior_selector", (char *) "check_pedestrian_near_path", CARMEN_PARAM_ONOFF, &behavior_selector_check_pedestrian_near_path, 1, NULL},
		{(char *) "behavior_selector", (char *) "pedestrian_near_path_min_lateral_distance", CARMEN_PARAM_DOUBLE, &behavior_pedestrian_near_path_min_lateral_distance, 1, NULL},
		{(char *) "behavior_selector", (char *) "pedestrian_near_path_min_longitudinal_distance", CARMEN_PARAM_DOUBLE, &behavior_selector_pedestrian_near_path_min_longitudinal_distance, 1, NULL},

		{(char *) "rrt",   			   (char *) "distance_interval", CARMEN_PARAM_DOUBLE, &param_distance_interval, 1, NULL},
		{(char *) "rrt",						(char *) "obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL},
		{(char *) "obstacle_avoider", 		  (char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.obstacle_avoider_obstacles_safe_distance, 	1, NULL},
		{(char *) "model_predictive_planner", (char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &robot_config.model_predictive_planner_obstacles_safe_distance, 	1, NULL},
		{(char *) "grid_mapping",      	 (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 	1, NULL},
		{(char *) "frenet_path_planner", (char *) "use_frenet_path_planner", CARMEN_PARAM_ONOFF, &use_frenet_path_planner, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "use_unity_simulator", CARMEN_PARAM_ONOFF, &use_unity_simulator, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "num_paths", CARMEN_PARAM_INT, &frenet_path_planner_num_paths, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "paths_displacement", CARMEN_PARAM_DOUBLE, &frenet_path_planner_paths_displacement, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "time_horizon", CARMEN_PARAM_DOUBLE, &frenet_path_planner_time_horizon, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "delta_t", CARMEN_PARAM_DOUBLE, &frenet_path_planner_delta_t, 0, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "min_moving_object_velocity",		CARMEN_PARAM_DOUBLE,	&min_moving_object_velocity,		1, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "max_moving_object_velocity",		CARMEN_PARAM_DOUBLE,	&max_moving_object_velocity,		1, NULL},
		{(char *) "obstacle_distance_mapper",	(char *) "moving_object_merge_distance",	CARMEN_PARAM_DOUBLE,	&moving_object_merge_distance,		1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));


	carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_param_list[] =
	{
		{(char *) "commandline",       (char *) "activate_tracking", CARMEN_PARAM_ONOFF, &activate_tracking, 0, NULL},
		{(char *) "behavior_selector", (char *) "keep_speed_limit", CARMEN_PARAM_ONOFF, &keep_speed_limit, 0, NULL},
	};
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	param_distance_between_waypoints = distance_between_waypoints;
	param_change_goal_distance = change_goal_distance;

	//TODO It look likes only voice interface uses carmen_ini_max_velocity, check what to do with max_reverse
	carmen_ini_max_velocity = last_speed_limit = robot_config.max_v;
	behavior_selector_initialize(robot_config, distance_between_waypoints, change_goal_distance, following_lane_planner, parking_planner, robot_max_velocity_reverse);

//	if (param_goal_source_onoff)
//		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;
//	else
//		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;

	original_behaviour_selector_central_lane_obstacles_safe_distance = robot_config.behaviour_selector_central_lane_obstacles_safe_distance;
	original_model_predictive_planner_obstacles_safe_distance = robot_config.model_predictive_planner_obstacles_safe_distance;

	distance_car_pose_car_front = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;

	if (semi_trailer_config.type > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.type);
}


int
main(int argc, char **argv)
{
	argc_global = argc;
	argv_global = argv;

	sleep(2);	// Para aguardar o param_deamon carregar
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	memset(&last_rddf_annotation_message, 0, sizeof(last_rddf_annotation_message));
//	memset(&road_profile_message, 0, sizeof(carmen_behavior_selector_road_profile_message));
	memset(&behavior_selector_state_message, 0, sizeof(carmen_behavior_selector_state_message));
	memset(&distance_map, 0, sizeof(carmen_obstacle_distance_mapper_map_message));
	memset(&distance_map2, 0, sizeof(carmen_prob_models_distance_map));

	read_parameters(argc, argv);
	if (behavior_selector_performs_path_planning)
	{
		carmen_rddf_play_get_parameters(argc, argv);
		frenet_path_planner_get_parameters(argc, argv);
		selected_path_id = frenet_path_planner_num_paths / 2;
	}

	define_messages();
	register_handlers();
	signal(SIGINT, shutdown_module);

	carmen_ipc_dispatch();

	return (0);
}
