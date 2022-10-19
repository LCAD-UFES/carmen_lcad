/*
 * util.cpp
 *
 *  Created on: 01/02/2012
 *      Author: rradaelli
 */

#include "util.h"
#include <carmen/carmen.h>
#include <sys/time.h>
#include "model/global_state.h"
#include "publisher_util.h"
#include "math.h"


bool Util::is_valid_position(const int &x, const int &y, const carmen_map_config_t &map_config)
{
	return x >= 0 && x < map_config.x_size && y >= 0 && y < map_config.y_size;
}

bool Util::is_valid_position(const int &x, const int &y)
{
	return is_valid_position(x, y, GlobalState::cost_map.config);
}

bool Util::is_valid_position(const Pose &pose, const carmen_map_config_t &map_config)
{
	return is_valid_position(round(pose.x), round(pose.y), map_config);
}

bool Util::is_valid_position(const Pose &pose)
{
	return is_valid_position(round(pose.x), round(pose.y), GlobalState::cost_map.config);
}

double Util::heading(const Pose &pose, const Pose &target_pose)
{
	double target_theta = fabs(carmen_normalize_theta(atan2(pose.y - target_pose.y, pose.x - target_pose.x) - M_PI - pose.theta));

	return target_theta / M_PI;
}

bool Util::is_behind(const Pose &p1, const Pose &p2)
{
	return fabs(carmen_normalize_theta(atan2(p1.y - p2.y, p1.x - p2.x) - M_PI - p1.theta)) > M_PI_2;
}

double Util::heading2(const Pose &pose, const Pose &target_pose)
{
	double alpha, target_theta, theta_diff;
	int	   is_goal_behind, is_opposite_side;

	alpha = carmen_normalize_theta(atan2(pose.y - target_pose.y, pose.x - target_pose.x) - M_PI);
	target_theta	 = fabs(carmen_normalize_theta(alpha - pose.theta));
	is_goal_behind	 = target_theta > M_PI_2;
	theta_diff		 = fabs(carmen_normalize_theta(pose.theta - target_pose.theta));
	is_opposite_side = theta_diff > M_PI_2;

	return fabs(is_opposite_side - fabs(is_goal_behind - (target_theta / M_PI)));
}

int Util::signalz(double num)
{
	if (num > 0)
	{
		return 1;
	}
	else if (num < 0)
	{
		return -1;
	}

	return 0;
}

int Util::signal(double num)
{
	return num >= 0 ? 1 : -1;
}

/**
 * Random double between 0 and 1
 */
double Util::normalized_random()
{
	return ((double)rand()) / ((double)RAND_MAX);
}

Pose Util::random_pose()
{
	Pose p;

	p.x		= rand() % GlobalState::cost_map.config.x_size;
	p.y		= rand() % GlobalState::cost_map.config.y_size;
	p.theta = carmen_normalize_theta(carmen_degrees_to_radians((rand() % 360) - 180));

	p = Util::to_global_pose(p);

	return p;
}

double Util::get_time()
{
	struct timeval t;

	gettimeofday(&t, NULL);
	return (t.tv_sec * 1000000 + t.tv_usec) * 0.000001;
}

Pose Util::to_map_pose(carmen_point_t world_pose)
{
	Pose p;

	p.theta = world_pose.theta;
	p.x		= (world_pose.x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
	p.y		= (world_pose.y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;

	return p;
}

Pose Util::to_map_pose(const Pose &world_pose, const carmen_map_config_t &map_config)
{
	Pose p;

	p.theta = world_pose.theta;
	p.x		= (world_pose.x - map_config.x_origin) / map_config.resolution;
	p.y		= (world_pose.y - map_config.y_origin) / map_config.resolution;

	return p;
}

Pose Util::to_map_pose(const Pose &world_pose)
{
	return to_map_pose(world_pose, GlobalState::cost_map.config);
}

carmen_point_t Util::to_world_pose(Pose map_pose)
{
	carmen_point_t p;

	p.theta = map_pose.theta;
	p.x		= map_pose.x * GlobalState::cost_map.config.resolution + GlobalState::cost_map.config.x_origin;
	p.y		= map_pose.y * GlobalState::cost_map.config.resolution + GlobalState::cost_map.config.y_origin;

	return p;
}

Pose Util::to_global_pose(const Pose &map_pose)
{
	return to_global_pose(map_pose, GlobalState::cost_map.config);
}

Pose Util::to_global_pose(const Pose &map_pose, const carmen_map_config_t &map_config)
{
	Pose p;

	p.theta = map_pose.theta;
	p.x		= map_pose.x * map_config.resolution + map_config.x_origin;
	p.y		= map_pose.y * map_config.resolution + map_config.y_origin;

	return p;
}

double Util::to_meter(const double &map_unit)
{
	return map_unit * GlobalState::cost_map.config.resolution;
}

double Util::to_map_unit(const double &meter)
{
	return to_map_unit(meter, GlobalState::cost_map.config);
}

double Util::to_map_unit(const double &meter, const carmen_map_config_t &map_config)
{
	return meter / map_config.resolution;
}

bool Util::can_reach(Pose robot, Pose goal)
{
	double theta_diff;
	bool   goal_reachable;

	theta_diff = goal.theta - robot.theta;

	goal_reachable = fabs(theta_diff) < carmen_degrees_to_radians(70);

	//printf("%f %f %d\n", carmen_radians_to_degrees(theta_diff), heading(robot, goal), goal_reachable);

	return goal_reachable;
}


carmen_point_t
Util::convert_to_carmen_point_t(const Pose pose)
{
	carmen_point_t p;

	p.x = pose.x;
	p.y = pose.y;
	p.theta = pose.theta;

	return p;
}


carmen_robot_and_trailers_traj_point_t
Util::convert_to_carmen_ackerman_traj_point_t(const Robot_State robot_state)
{
	carmen_robot_and_trailers_traj_point_t traj_point;

	traj_point.x = robot_state.pose.x;
	traj_point.y = robot_state.pose.y;
	traj_point.theta = robot_state.pose.theta;
	traj_point.v = robot_state.v_and_phi.v;
	traj_point.phi = robot_state.v_and_phi.phi;

	return traj_point;
}


carmen_ackerman_path_point_t
Util::convert_to_carmen_ackerman_path_point_t(const Robot_State robot_state, const double time)
{
	carmen_ackerman_path_point_t path_point;

	path_point.x = robot_state.pose.x;
	path_point.y = robot_state.pose.y;
	path_point.theta = robot_state.pose.theta;
	path_point.v = robot_state.v_and_phi.v;
	path_point.phi = robot_state.v_and_phi.phi;
	path_point.time = time;

	return path_point;
}


double
Util::distance(const Pose *pose1, const Pose *pose2)
{
	return (sqrt((pose1->x - pose2->x) * (pose1->x - pose2->x) + (pose1->y - pose2->y) * (pose1->y - pose2->y)));
}


double
Util::distance(const Pose *pose1, const carmen_ackerman_path_point_t *pose2)
{
	return (sqrt((pose1->x - pose2->x) * (pose1->x - pose2->x) + (pose1->y - pose2->y) * (pose1->y - pose2->y)));
}


static double
dist2(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


bool
Util::between(carmen_ackerman_path_point_t &p, const Pose *pose,
		carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	// Return minimum distance between line segment vw and point p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	double l2, t;

	p.x = pose->x;
	p.y = pose->y;
	p.time = 0.0;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (l2 == 0.0)
	{
		if (dist2(p, w) == 0)
			return (true);
		else
			return (false);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p before the v end of the segment
		return (false);
	if (t > 1.0)	// p beyond the w end of the segment
		return (false);

	// Projection falls on the segment
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	p.time = v.time * t;

	return (true);
}


bool
Util::between(const Pose *pose1, const carmen_ackerman_path_point_t pose2, const carmen_ackerman_path_point_t pose3)
{
	carmen_ackerman_path_point_t p;
	return (between(p, pose1, pose2, pose3));
}


//-----------Funcoes para extrair dados do Experimento------------------------
double
dist(carmen_robot_and_trailers_path_point_t v, carmen_robot_and_trailers_path_point_t w)
{
    return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}

/*
 * Encontra distancia do eixo traseiro pra lane/path
 * */
double
get_distance_between_point_to_line(carmen_robot_and_trailers_path_point_t p1,
		carmen_robot_and_trailers_path_point_t p2,
		carmen_robot_and_trailers_path_point_t robot)
{
    //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
    double d = sqrt(delta_x * delta_x + delta_y * delta_y);
    double x2y1 =  p2.x * p1.y;
    double y2x1 =  p2.y * p1.x;

    if (d < 0.0000001)
        return dist(p2, robot);

    return fabs((delta_y * robot.x) - (delta_x * robot.y) + x2y1 - y2x1) / d;

}

/*
 * Usa 3 pontos para achar os 2 pontos que formam a reta mais proxima do carro.
 *  Preenche index1, index2 e pose mais próxima do carro usando distancia euclidiana (carro sempre na origem)
 *
 */
void
get_points2(vector<carmen_robot_and_trailers_path_point_t> &detailed_goal_list, int &index_p1, int &index_p2, int &mais_proxima)
{

    double d = sqrt(pow(detailed_goal_list.at(0).x, 2) + pow(detailed_goal_list.at(0).y, 2));
    double d2 = sqrt(pow(detailed_goal_list.at(2).x, 2) + pow(detailed_goal_list.at(2).y, 2));
    double centro = sqrt(pow(detailed_goal_list.at(1).x, 2) + pow(detailed_goal_list.at(1).y, 2));
    if (d < d2)
    {
        index_p1 = 0;
        index_p2 = 1;
        mais_proxima = index_p1;
        if(centro < d)
            mais_proxima = 1;
    }
    else
    {
        index_p1 = 1;
        index_p2 = 2;
        mais_proxima = index_p2;
        if(centro < d2)
            mais_proxima = 1;
    }
}


void
get_between_points(carmen_robot_and_trailers_path_point_t robot, carmen_robot_and_trailers_path_point_t point_before, carmen_robot_and_trailers_path_point_t center, carmen_robot_and_trailers_path_point_t point_next,
		int index_center, int &index_p1, int &index_p2, int &mais_proxima)
{

    double centro = DIST2D(center, robot);
    double d = DIST2D(point_before, robot);
    double d2 = DIST2D(point_next, robot);

    if (d < d2)
    {
        index_p1 = (index_center - 1);
        index_p2 = (index_center);
        mais_proxima = (index_center - 1);
        if(centro < d)
            mais_proxima = centro;
    }
    else
    {
        index_p1 = index_center;
        index_p2 = index_center+1;
        mais_proxima = index_p2;
        if(centro < d2)
            mais_proxima = centro;
    }
}


void
save_experiment_data(carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message,
					carmen_robot_and_trailers_pose_t *localizer_pose, vector<carmen_robot_and_trailers_path_point_t> &detailed_lane,
					Command lastOdometry, double target_v)
{
	if (detailed_lane.size() > 0)
	{
		carmen_robot_and_trailers_path_point_t localize;
		localize.x = 0.0;
		localize.y = 0.0;
		//Metric evaluation
		int index1;
		int index2;
		int mais_proxima;
		get_points2(detailed_lane,index1, index2,mais_proxima);
		//      printf("%lf %lf \n", detailed_goal_list.at(index1).x, detailed_goal_list.at(index2).x);
		double distance_metric = get_distance_between_point_to_line(detailed_lane.at(index1), detailed_lane.at(index2), localize);
		//      printf("%lf \n", distance_metric);
		double x_rddf = localizer_pose->x + detailed_lane.at(mais_proxima).x * cos(localizer_pose->theta) - detailed_lane.at(mais_proxima).y * sin(localizer_pose->theta);
		double y_rddf = localizer_pose->y + detailed_lane.at(mais_proxima).x * sin(localizer_pose->theta) + detailed_lane.at(mais_proxima).y * cos(localizer_pose->theta);
		double theta_rddf = detailed_lane.at(mais_proxima).theta + localizer_pose->theta;
		double volante_rddf_theta = atan2(detailed_lane.at(index1).y - detailed_lane.at(index2).y , detailed_lane.at(index1).x - detailed_lane.at(index2).x);
		double erro_theta = fabs(volante_rddf_theta - localizer_pose->theta);
		//          1-Localise_x 2-Localise_y 3-Localise_theta 4-velocity 5-phi 6-rddf_x 7-rddf_y 8-rddf_theta 9-rddf_velocity 10-rddf_phi 11-target_v 12-lateralDist 13-volante 14-erro_theta 15-Timestamp
		fprintf(stderr, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n", localizer_pose->x, localizer_pose->y, localizer_pose->theta,
				lastOdometry.v, lastOdometry.phi, x_rddf, y_rddf, theta_rddf, detailed_lane.at(mais_proxima).v,
				detailed_lane.at(mais_proxima).phi, target_v, distance_metric, volante_rddf_theta, erro_theta, path_goals_and_annotations_message->timestamp);

	}
}

//------------------------------------------------------------

