/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include "carmen.h"

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
#include <carmen/map.h>
#endif

#include "geometry.h"
#include <assert.h>

#ifndef COMPILE_WITHOUT_MAP_SUPPORT

int carmen_geometry_x_offset[CARMEN_NUM_OFFSETS] = {0, 1, 1, 1, 0, -1, -1, -1};
int carmen_geometry_y_offset[CARMEN_NUM_OFFSETS] = {-1, -1, 0, 1, 1, 1, 0, -1};
#endif
double carmen_geometry_compute_safety_ackerman_distance(carmen_robot_ackerman_config_t *robot_ackerman_config,
		carmen_traj_point_t *robot)
{
	//TODO rever
	carmen_robot_config_t robot_config;

	robot_config.acceleration = robot_ackerman_config->maximum_acceleration_forward;
	robot_config.allow_rear_motion = robot_ackerman_config->allow_rear_motion;
	robot_config.approach_dist = robot_ackerman_config->approach_dist;
	robot_config.deceleration = robot_ackerman_config->maximum_deceleration_forward;
	robot_config.interpolate_odometry = robot_ackerman_config->interpolate_odometry;
	robot_config.length = robot_ackerman_config->length;
	robot_config.max_r_vel = robot_ackerman_config->max_phi;
	robot_config.max_t_vel = robot_ackerman_config->max_v;
	robot_config.reaction_time = robot_ackerman_config->reaction_time;
	robot_config.rectangular = robot_ackerman_config->rectangular;
	robot_config.side_dist = robot_ackerman_config->side_dist;
	robot_config.width = robot_ackerman_config->side_dist;

	return carmen_geometry_compute_safety_distance(&robot_config, robot);

}

double carmen_geometry_compute_safety_distance(carmen_robot_config_t *robot_config,
		carmen_traj_point_t *robot)
{
	return robot_config->length / 2.0 + robot_config->approach_dist +
			robot->t_vel * robot_config->reaction_time +
			robot->t_vel*robot->t_vel/(2*robot_config->acceleration) ;
}

static double compute_velocity_at_side(carmen_traj_point_t robot, 
		carmen_traj_point_t dest_pt,
		carmen_traj_point_t centre,
		double radius,
		carmen_robot_config_t *robot_config)
{
	double rotation_angle;
	double forward_safety_distance;
	double forward_distance;
	double velocity;
	double max_velocity = robot_config->max_t_vel;
	double motion_time;

	forward_safety_distance = carmen_geometry_compute_safety_distance(robot_config, &robot);

	robot.x -= centre.x;
	robot.y -= centre.y;
	dest_pt.x -= centre.x;
	dest_pt.y -= centre.y;

	rotation_angle = atan2(dest_pt.y, dest_pt.x) - atan2(robot.y, robot.x);
	rotation_angle = carmen_normalize_theta(rotation_angle);

	forward_distance = fabs(rotation_angle * radius);

	if (forward_distance < -robot_config->length/2.0)
		return max_velocity;

	// How far to the obstacle? Remove the safety distance and the distance we travel
	// while reacting

	forward_distance -= forward_safety_distance;

	if (forward_distance < 0)
		return max_velocity;

	// 0 = v_0^2 - 2ad
	// velocity = sqrt (2 * acceleration * forward_distance)

	velocity = sqrt(2 * robot_config->acceleration * forward_distance);

	// Velocity is now how fast we can go and still decelerate in time not to hit things.
	// However, the point may be off to one side. if it is,

	motion_time = velocity / robot_config->acceleration;

	if (fabs(rotation_angle) > robot.r_vel * motion_time)
		return max_velocity;

	// If the robot is going slower than the maximum allowable speed, assume we
	// accelerate over half the distance to the obstacle, and *then* start slowing down.

	if (velocity > robot.t_vel) {
		velocity =  sqrt(2 * robot_config->acceleration * forward_distance/2);
	}

	assert (!isnan(velocity));

	return velocity;
}

static double compute_forward_velocity(carmen_traj_point_t robot, 
		carmen_traj_point_t dest_pt,
		carmen_robot_config_t *robot_config)
{
	double forward_safety_distance;
	double forward_distance;
	double velocity;

	forward_distance = dest_pt.x;

	if (forward_distance < -robot_config->length/2.0)
		return robot_config->max_t_vel;

	forward_safety_distance =
			carmen_geometry_compute_safety_distance(robot_config, &robot);

	// How far to the obstacle? Remove the safety distance and the distance we travel
	// while reacting

	forward_distance -= forward_safety_distance;

	if (forward_distance < 0)
		return 0;

	// 0 = v_0^2 - 2ad
	// velocity = sqrt (2 * acceleration * forward_distance)

	velocity = sqrt(2 * robot_config->acceleration * forward_distance);

	// If the robot is going slower than the maximum allowable speed, assume we
	// accelerate over half the distance to the obstacle, and *then* start slowing down.

	if (velocity > robot.t_vel)
		velocity =  sqrt(2 * robot_config->acceleration * forward_distance/2);

	assert (!isnan(velocity));

	return velocity;
}


void 
carmen_geometry_compute_centre_and_curvature(carmen_traj_point_t start_point, 
		double theta,
		carmen_traj_point_t end_point,
		carmen_traj_point_t *centre,
		double *radius)
{
	double m_perpendicular;
	double distance;
	double x_intercept, y_intercept;
	double tan_theta_parallel, cos_theta_parallel, sin_theta_parallel;
//	double dist_sq;

	*centre = start_point;

//	dist_sq = (end_point.x-start_point.x)*(end_point.x-start_point.x) +

	distance = carmen_distance_traj(&end_point, &start_point);

	assert (!isnan(distance));

	sin_theta_parallel = (end_point.y-start_point.y) / distance;
	cos_theta_parallel = (end_point.x-start_point.x) / distance;
	tan_theta_parallel = (sin_theta_parallel/cos_theta_parallel);

	/* If the point is basically right in front of the robot heading, then
     there's no curvature. */

	if (fabs(tan_theta_parallel - tan(theta)) < .01) {
		*radius = 0;
		return;
	}

	theta += M_PI/2;

	/* m_parallel would be sin_theta_parallel/cos_theta_parallel,
     and m_perpendicular = -1.0 / m_parallel, so
     m_perpendicular = -cos_theta_parallel/sin_theta_parallel
	 */
	m_perpendicular = -cos_theta_parallel/sin_theta_parallel;

	distance = 0.5*distance;
	x_intercept = distance*cos_theta_parallel;
	y_intercept = distance*sin_theta_parallel;

	centre->x = (y_intercept - m_perpendicular*x_intercept)/
			(tan(theta) - m_perpendicular);
	centre->y = m_perpendicular*(centre->x - x_intercept) + y_intercept;

	*radius = hypot(centre->x, centre->y);

	if (*radius >= 10000)
		carmen_warn("radius %f\n", *radius);
	assert(*radius < 100000);

	centre->x += start_point.x;
	centre->y += start_point.y;
}

double
carmen_geometry_compute_ackerman_velocity(carmen_traj_point_t robot,
		carmen_traj_point_t dest_pt,
		carmen_robot_ackerman_config_t *robot_ackerman_config)
{
	//TODO rever
	carmen_robot_config_t robot_config;

	robot_config.acceleration = robot_ackerman_config->maximum_acceleration_forward;
	robot_config.allow_rear_motion = robot_ackerman_config->allow_rear_motion;
	robot_config.approach_dist = robot_ackerman_config->approach_dist;
	robot_config.deceleration = robot_ackerman_config->maximum_deceleration_forward;
	robot_config.interpolate_odometry = robot_ackerman_config->interpolate_odometry;
	robot_config.length = robot_ackerman_config->length;
	robot_config.max_r_vel = robot_ackerman_config->max_phi;
	robot_config.max_t_vel = robot_ackerman_config->max_v;
	robot_config.reaction_time = robot_ackerman_config->reaction_time;
	robot_config.rectangular = robot_ackerman_config->rectangular;
	robot_config.side_dist = robot_ackerman_config->side_dist;
	robot_config.width = robot_ackerman_config->side_dist;

	return carmen_geometry_compute_velocity(
			robot,
			dest_pt,
			&robot_config);
}
double 
carmen_geometry_compute_velocity(carmen_traj_point_t robot, 
		carmen_traj_point_t dest_pt,
		carmen_robot_config_t *robot_config)
{
	carmen_traj_point_t left_point, right_point, centre;
	double max_velocity;
	//double side_theta;
	double side_safety_distance;
	double radius;
	double temp_x;

	max_velocity = robot_config->max_t_vel;

	/* Shift everything so the robot is centred at zero */

	dest_pt.x -= robot.x;
	dest_pt.y -= robot.y;

	robot.x = 0;
	robot.y = 0;

	/* Rotate everything into reference frame of robot */

	if (fabs(robot.theta) > 0.01) {
		temp_x = dest_pt.x*cos(robot.theta)+dest_pt.y*sin(robot.theta);
		dest_pt.y = dest_pt.y*cos(robot.theta)-dest_pt.x*sin(robot.theta);
		dest_pt.x = temp_x;

		robot.theta = 0.0;
	}

	//side_theta = M_PI/2;
	side_safety_distance = robot_config->width / 2.0 + robot_config->side_dist;

	left_point = robot;
	left_point.x = robot.x;
	left_point.y = robot.y + side_safety_distance;

	right_point = robot;
	right_point.x = robot.x;
	right_point.y = robot.y - side_safety_distance;

	/* Obstacle point is behind front of robot */

	if (dest_pt.x < 0) {
		if (fabs(dest_pt.y) < side_safety_distance)
			max_velocity = compute_velocity_at_side
			(robot, dest_pt, robot, 0, robot_config);
		else
			max_velocity = robot_config->max_t_vel;
	} else {

		/* Obstacle point is in front of robot, but within the side
       safety margins */ 
		if (fabs(dest_pt.y) < side_safety_distance)  {
			max_velocity = compute_forward_velocity
					(robot, dest_pt, robot_config);
		}

		/* Obstacle point is in front of robot, and outside the side safety margins.
       Will we hit it if we start rotating? */ 

		else if (dest_pt.y > 0) {
			carmen_geometry_compute_centre_and_curvature
			(left_point, robot.theta, dest_pt, &centre, &radius);
			max_velocity = compute_velocity_at_side
					(left_point, dest_pt, centre, radius, robot_config);
		} else {
			carmen_geometry_compute_centre_and_curvature
			(right_point, robot.theta, dest_pt, &centre, &radius);
			max_velocity = compute_velocity_at_side
					(right_point, dest_pt, centre, radius, robot_config);
		}
	}

	assert(!isnan(max_velocity));

	return max_velocity;
}

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
void 
carmen_geometry_project_point(int x, int y, double theta, int *x2, int *y2, 
		carmen_map_config_t map_defn)
{
	double delta_x, delta_y, delta_theta;
	double bounding_angle;

	theta = carmen_normalize_theta(theta);

	if (theta < M_PI / 2 && theta >= -M_PI/2)
		delta_x = map_defn.x_size - 1 - x;
	else
		delta_x = -x;
	if (theta >= 0)
		delta_y = map_defn.y_size - 1 - y;
	else
		delta_y = -y;

	/* The angle to the corner to which theta is closest. */

	bounding_angle = atan2(delta_y, delta_x);

	/* This case if theta is going to run off the top of the bounding box. */

	if (theta >= 0 && ((theta < M_PI/2 && theta >= bounding_angle) ||
			(theta >= M_PI/2 && theta < bounding_angle)))
	{
		*y2 = map_defn.y_size - 1;
		delta_theta = M_PI/2 - theta;
		*x2 = carmen_round(x + tan(delta_theta)*delta_y);
	}

	/* This case if theta is going to run off the right side
     of the bounding box. */

	else if ((theta < M_PI/2 && theta >= -M_PI/2) &&
			((theta >= 0 && theta < bounding_angle) ||
					(theta < 0 && theta >= bounding_angle)))
	{
		*x2 = map_defn.x_size - 1;
		delta_theta = theta;
		*y2 = carmen_round(y + tan(delta_theta)*delta_x);
	}

	/* This case if theta is going to run off the bottom of the bounding box. */

	else if (theta < 0 && ((theta >= -M_PI/2 && theta < bounding_angle) ||
			(theta < -M_PI/2 && theta >= bounding_angle)))
	{
		*y2 = 0;
		delta_theta = -M_PI/2 - theta;
		*x2 = carmen_round(x + tan(delta_theta)*(delta_y));
	}

	/* This case if theta is going to run off the left side
     of the bounding box. */

	else
	{
		*x2 = 0;
		if (theta < 0)
			delta_theta = -M_PI - theta;
		else
			delta_theta = M_PI - theta;
		*y2 = carmen_round(y + tan(delta_theta)*fabs(delta_x));
	}
}
#endif

void 
carmen_geometry_move_pt_to_rotating_ref_frame(carmen_traj_point_p obstacle_pt,
		double tv, double rv)
{
	double radius;
	carmen_traj_point_t robot_posn, centre;
	double obstacle_radius;
	double obstacle_theta;

	robot_posn.x = 0;
	robot_posn.y = 0;

	radius = tv / rv;

	if (fabs(tv) > 0.01 && fabs(rv) > 0.001)
	{
		centre.x = 0;
		centre.y = radius;
		obstacle_radius = carmen_distance_traj(obstacle_pt, &centre);
		if (radius < 0)
			obstacle_radius = -obstacle_radius;
		obstacle_theta = carmen_angle_between(&centre, obstacle_pt) -
				carmen_angle_between(&centre, &robot_posn);

		obstacle_pt->x = radius * obstacle_theta;
		obstacle_pt->y = radius - obstacle_radius;
	}
}


double 
carmen_geometry_compute_radius_and_centre_old(carmen_traj_point_p prev, 
		carmen_traj_point_p current,
		carmen_traj_point_p next,
		carmen_traj_point_p centre,
		carmen_traj_point_p end_curve)
{

	/* The centre of the circle that lies inside the angle formed by these three
     points lies on one line, call this the centre_line. A second line runs
     through whichever point is closest to current, and is perpendicular to
     the line between that point and current, call this the perp_line. 
     The centre of the biggest circle lies at the intersection of these two
     lines, and has radius equal to the distance between the centre and the
     point closest to current. */

	double distance_from_prev, distance_from_current;
	double angle_from_prev, angle_to_prev, angle_from_current;
	double centre_slope, centre_intersect;
	double perp_slope, perp_intersect;
	double not_perp_slope, not_perp_intersect;
	double end_slope, end_intersect;
	carmen_traj_point_p perp_point;
	double radius;

	angle_from_prev = carmen_angle_between(prev, current);
	angle_to_prev = carmen_angle_between(current, prev);
	angle_from_current = carmen_angle_between(current, next);

	centre_slope =
			tan(carmen_normalize_theta(angle_to_prev/2.0 + angle_from_current/2.0));
	centre_intersect = current->y - centre_slope*current->x;

	distance_from_prev = carmen_distance_traj(prev, current);
	distance_from_current = carmen_distance_traj(current, next);
	if (distance_from_prev < distance_from_current)
	{
		perp_slope = -1/tan(angle_from_prev);
		not_perp_slope = -1/tan(angle_from_current);
		end_slope = tan(angle_from_current);
		perp_intersect = prev->y - perp_slope*prev->x;
		perp_point = prev;
	}
	else
	{
		perp_slope = -1/tan(angle_from_current);
		not_perp_slope = -1/tan(angle_from_prev);
		end_slope = tan(angle_from_prev);
		perp_intersect = next->y - perp_slope*next->x;
		perp_point = next;
	}

	centre->x = (perp_intersect - centre_intersect)/(centre_slope-perp_slope);
	centre->y = centre_slope*centre->x + centre_intersect;

	not_perp_intersect = centre->y - not_perp_slope*centre->x;
	end_intersect = current->y - end_slope*current->x;
	end_curve->x = (end_intersect - not_perp_intersect)/
			(not_perp_slope - end_slope);
	end_curve->y = not_perp_slope*end_curve->x + not_perp_intersect;

	radius = carmen_distance_traj(centre, perp_point);

	if (isnan(radius))
		carmen_warn("Is nan: centre %.2f %.2f, perp_point %.2f %.2f\n", centre->x,
				centre->y, perp_point->x, perp_point->y);


	return radius;
}

double 
carmen_geometry_compute_radius_and_centre(carmen_traj_point_p prev, 
		carmen_traj_point_p current,
		carmen_traj_point_p next,
		carmen_traj_point_p centre,
		carmen_traj_point_p end_curve)
{

	/* The centre of the circle that lies inside the angle formed by these three
     points lies on one line, call this the centre_line. A second line runs
     through whichever point is closest to current, and is perpendicular to
     the line between that point and current, call this the perp_line. 
     The centre of the biggest circle lies at the intersection of these two
     lines, and has radius equal to the distance between the centre and the
     point closest to current. */

	double distance_from_prev, distance_from_current;
	carmen_traj_point_p start;
	double angle_perp, angle_not_perp;


	double angle_from_prev, angle_to_prev, angle_from_current;
	double centre_a, centre_b, centre_c;
	double perp_a, perp_b, perp_c;
	double not_perp_a, not_perp_b, not_perp_c;
	double end_curve_a, end_curve_b, end_curve_c;

	double radius;

	angle_from_prev = carmen_angle_between(prev, current);
	angle_to_prev = carmen_angle_between(current, prev);
	angle_from_current = carmen_angle_between(current, next);

	centre_a = tan(carmen_normalize_theta
			(angle_to_prev/2.0 + angle_from_current/2.0));
	centre_b = -1;
	if (fabs(centre_a) > 1e6) {
		centre_a = 1;
		centre_b = 0;
	}
	centre_c = -(centre_a*current->x + centre_b*current->y);

	distance_from_prev = carmen_distance_traj(prev, current);
	distance_from_current = carmen_distance_traj(current, next);
	if (distance_from_prev < distance_from_current)
	{
		start = prev;
		angle_perp = angle_from_prev;
		angle_not_perp = angle_from_current;
	}
	else
	{
		start = next;
		angle_perp = angle_from_current;
		angle_not_perp = angle_from_prev;
	}

	perp_a = 1;
	perp_b = tan(angle_perp);
	if (fabs(perp_b) > 1e6)
	{
		perp_a = 0;
		perp_b = 1;
	}
	perp_c = -(perp_a*start->x + perp_b*start->y);

	if (fabs(centre_b*perp_a - perp_b*centre_a) < .01)
		return 0.0;

	centre->y = (perp_c*centre_a - centre_c*perp_a) /
			(centre_b*perp_a - perp_b*centre_a);
	if (fabs(centre_a) < 0.01)
		centre->x = current->x;
	else
		centre->x = (centre_b*centre->y + centre_c) / (-centre_a);

	not_perp_a = 1;
	not_perp_b = tan(angle_not_perp);
	if (fabs(not_perp_b) > 1e6)
	{
		not_perp_a = 0;
		not_perp_b = -1;
	}
	not_perp_c = -(not_perp_a*centre->x + not_perp_b*centre->y);

	end_curve_a = tan(angle_not_perp);
	end_curve_b = -1;
	if (fabs(end_curve_a) > 1e6)
	{
		end_curve_a = 1;
		end_curve_b = 0;
	}
	end_curve_c = -(end_curve_a*current->x+end_curve_b*current->y);

	if (fabs(end_curve_b*not_perp_a - not_perp_b*end_curve_a) < 0.01)
		return 0.0;

	end_curve->y = (not_perp_c*end_curve_a - end_curve_c*not_perp_a) /
			(end_curve_b*not_perp_a - not_perp_b*end_curve_a);
	if (fabs(end_curve_a) < 0.01)
		end_curve->x = current->x;
	else
		end_curve->x = (end_curve_b*end_curve->y + end_curve_c) / (-end_curve_a);

	radius = carmen_distance_traj(centre, start);

	return radius;
}

static void intersect(carmen_point_t *edge1, carmen_point_t *edge2,
		carmen_point_t *ray3, carmen_point_t *ray4,
		carmen_point_t *intersect_pt)
{
	double slope1, slope2, intercept1, intercept2;
	double det, a, b, x_nom, y_nom;

	if (fabs(edge2->x-edge1->x) < 1e-6 &&
			fabs(ray4->x-ray3->x) < 1e-6) {
		if (fabs(edge2->x-ray3->x) < 1e-6) {
			if (fabs(edge1->x - ray3->x) < fabs(edge2->x - ray3->x)) {
				intersect_pt->x = edge1->x;
				intersect_pt->y = edge1->y;
			} else {
				intersect_pt->x = edge2->x;
				intersect_pt->y = edge2->y;
			}
		} else {
			intersect_pt->x = ray4->x;
			intersect_pt->y = ray4->y;
		}
		return;
	}

	// Check if the lines are parallel

	slope1 = (edge2->y - edge1->y)/(edge2->x - edge1->x);
	slope2 = (ray4->y - ray3->y)/(ray4->x - ray3->x);

	if (fabs(slope1 - slope2) < 1e-6) {
		intercept1 = edge2->y - slope1*edge2->x;
		intercept2 = ray4->y - slope2*ray4->x;
		if (fabs(intercept1 - intercept2)) {
			if (hypot(edge1->x-ray3->x, edge1->y-ray3->y) < hypot(edge2->x-ray3->x, edge2->y-ray3->y)) {
				intersect_pt->x = edge1->x;
				intersect_pt->y = edge1->y;
			} else {
				intersect_pt->x = edge2->x;
				intersect_pt->y = edge2->y;
			}
		}
		else {
			intersect_pt->x = ray4->x;
			intersect_pt->y = ray4->y;
		}

		return;
	}

	det = (edge1->x-edge2->x)*(ray3->y-ray4->y)-(ray3->x-ray4->x)*(edge1->y-edge2->y);

	a = edge1->x*edge2->y - edge2->x*edge1->y;
	b = ray3->x*ray4->y-ray4->x*ray3->y;

	x_nom = a*(ray3->x-ray4->x) - b*(edge1->x-edge2->x);
	intersect_pt->x = x_nom/det;

	y_nom = a*(ray3->y-ray4->y) - b*(edge1->y-edge2->y);
	intersect_pt->y = y_nom/det;
}

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
double 
carmen_geometry_compute_expected_distance(carmen_traj_point_p traj_point, 
		double theta, carmen_map_p map)
{
	int x2, y2, px, py;
	carmen_bresenham_param_t params;
	carmen_traj_point_t obstacle;
	double distance=0;
	double resolution;
	int map_x, map_y;
	carmen_map_config_t map_defn;
	int index;

	double min_distance = MAXDOUBLE;
	//int best = -1;
	carmen_point_t ray3, ray4;
	carmen_point_t edge1, edge2;
	carmen_point_t intersect_pt;

	int x_offset[8] = {1, 1, -1, 1, -1, -1, -1, 1};
	int y_offset[8] = {-1, 1, 1, 1, -1, 1, -1, -1};

	double distance_to_obstacle, min_distance_to_obstacle = MAXDOUBLE;
	//int best_obstacle;
	carmen_point_t best_intersect_obstacle_pt = {0.0, 0.0, 0.0};

	map_defn = map->config;

	resolution = map->config.resolution;

	/* Find centre of closest occupied grid cell in map */

	map_x = carmen_round((traj_point->x - map->config.x_origin) / resolution);
	map_y = carmen_round((traj_point->y - map->config.y_origin) / resolution);

	carmen_geometry_project_point(map_x, map_y, theta, &x2, &y2, map_defn);

	carmen_get_bresenham_parameters(map_x, map_y, x2, y2, &params);
	do
	{
		carmen_get_current_point(&params, &map_x, &map_y);
		if(map_x < 0 || map_x >= map->config.x_size ||
				map_y < 0 || map_y >= map->config.y_size)
			break;
		if (map->map[map_x][map_y] > 0.5)
			break;
	}
	while (carmen_get_next_point(&params));

	ray3.x = traj_point->x;
	ray3.y = traj_point->y;
	ray4.x = traj_point->x + (distance + 1) * cos(theta);
	ray4.y = traj_point->y + (distance + 1) * sin(theta);

	for (index = 0; index < 4; index++) {
		edge1.x = (map_x * resolution + map->config.x_origin) + x_offset[2*index] * resolution * 0.5;
		edge1.y = (map_y * resolution + map->config.y_origin) + y_offset[2*index] * resolution * 0.5;

		edge2.x = (map_x * resolution + map->config.x_origin) + x_offset[2 * index + 1] * resolution * 0.5;
		edge2.y = (map_y * resolution + map->config.y_origin) + y_offset[2 * index + 1] * resolution * 0.5;

		intersect(&edge1, &edge2, &ray3, &ray4, &intersect_pt);

		distance = hypot(intersect_pt.x - traj_point->x, intersect_pt.y - traj_point->y);
		distance_to_obstacle = hypot(intersect_pt.x - (map_x * resolution + map->config.x_origin),
				intersect_pt.y - (map_y * resolution) + map->config.y_origin);

		if (distance_to_obstacle < min_distance_to_obstacle)
		{
			min_distance_to_obstacle = distance_to_obstacle;
			best_intersect_obstacle_pt = intersect_pt;
		//	best_obstacle = index;
		}

		if (index == 0 || index == 2)
		{
			if (intersect_pt.y >= edge1.y && intersect_pt.y <= edge2.y && distance < min_distance ) {
				min_distance = distance;
			//	best = index;
			}
		} else
		{
			if (intersect_pt.x >= edge1.x && intersect_pt.x <= edge2.x && distance < min_distance ) {
				min_distance = distance;
			//	best = index;
			}
		}
	}

	if (min_distance > MAXDOUBLE/2)
		min_distance = hypot(best_intersect_obstacle_pt.x-traj_point->x,
				best_intersect_obstacle_pt.y-traj_point->y);

	return min_distance;

	/* Project map up to cm level co-ordinates, and walk backwards to find the
     first **unoccupied** cm grid cell, to find out where the obstacle actually
     begins. Knowing the centre of obstacles isn't always useful. */

	px = carmen_round(traj_point->x * 100.0);
	py = carmen_round(traj_point->y * 100.0);

	x2 = carmen_round(px + distance * cos(theta) * 100);
	y2 = carmen_round(py + distance * sin(theta) * 100);

	carmen_warn("x2: %d %d px: %d %d d: %f ", x2, y2, px, py, distance);

	resolution *= 100.0;

	carmen_get_bresenham_parameters(x2, y2, px, py, &params);
	obstacle.x = x2;
	obstacle.y = y2;
	do
	{
		carmen_get_current_point(&params, &px, &py);
		map_x = carmen_round((px - map->config.x_origin) / resolution);
		map_y = carmen_round((py - map->config.y_origin) / resolution);
		if(map_x < 0 || map_x >= map->config.x_size ||
				map_y < 0 || map_y >= map->config.y_size)
			break;
		if (map->map[map_x][map_y] < 0.5)
			break;
		obstacle.x = px / 100;
		obstacle.y = py / 100;

	}
	while (carmen_get_next_point(&params));

	carmen_warn("now x2: %d %d px: %d %d d: %f\n", x2, y2, px, py, distance);

	distance = carmen_distance_traj(&obstacle, traj_point);

	return distance;

}

void
carmen_geometry_generate_sonar_data(double *sonar_data, 
		carmen_traj_point_p center,
		carmen_point_p sonar_offsets,
		int num_sonars, carmen_map_p map)
{
	int i;
	double r, t;
	carmen_traj_point_t sonar_traj;

	sonar_traj.t_vel=center->t_vel;
	sonar_traj.r_vel=center->r_vel;

	for(i=0; i<num_sonars; i++)
	{
		r = hypot(sonar_offsets[i].x, sonar_offsets[i].y);
		t = carmen_normalize_theta(atan2(sonar_offsets[i].y, sonar_offsets[i].x)
				+ center->theta);
		sonar_traj.theta =carmen_normalize_theta
				(sonar_offsets[i].theta + center->theta);
		sonar_traj.x = center->x + r*cos(t);
		sonar_traj.y = center->y + r*sin(t);
		sonar_data[i]=carmen_geometry_compute_expected_distance
				(&sonar_traj, sonar_traj.theta, map);
	}
}

void 
carmen_geometry_generate_laser_data(double *laser_data,
		carmen_traj_point_p traj_point,
		double start_theta, double end_theta,
		int num_points, carmen_map_p map)
{
	int index;
	double *laser_data_ptr;
	double theta;
	double separation;
	double dist;

	start_theta = carmen_normalize_theta(start_theta);
	end_theta = carmen_normalize_theta(end_theta);
	theta = carmen_normalize_theta(start_theta + traj_point->theta);

	if (end_theta <= start_theta)
		separation = (2*M_PI + (end_theta-start_theta)) / num_points;
	else
		separation = (end_theta - start_theta)/num_points;

	laser_data_ptr = laser_data;
	for (index = 0; index < num_points; index++)
	{
		dist = carmen_geometry_compute_expected_distance(traj_point, theta, map);
		*(laser_data_ptr++) = dist;
		theta = carmen_normalize_theta(theta+separation);
	}
}

/*
 * Development code below here : do not use
 *
 */

static int num_points_to_generate=-1;
static double ***cached_data = NULL;  
static carmen_map_p expected_map = NULL;
static int cache_misses = 0;
static int cache_hits = 0;

void 
carmen_geometry_fast_generate_laser_data
(double *laser_data, carmen_traj_point_p traj_point,
		double start_theta,
		double end_theta __attribute__ ((unused)),
		int num_points, carmen_map_p map)
{
	int index;
	double *laser_data_ptr;
	double separation;
	double dist;
	double *dist_ptr;
	double laser_theta;
	int x, y;
	int offset;

	if (num_points_to_generate < 0)
		num_points_to_generate = num_points;

	if (num_points_to_generate != num_points)
		carmen_die("You can't (yet!) call generate_laser_data with requests "
				"for different numbers \nof points. After the first time you "
				"call it, you have to use the same number of\n"
				"points always.\n");

	if (expected_map == NULL)
		expected_map = map;


	if (expected_map != map)
		carmen_die("You can't (yet!) call generate_laser_data different maps. "
				"After the first time\nyou call it, you have to always re-use "
				"the same map.\n");

	if (cached_data == NULL)
	{
		cached_data = (double ***)
			calloc(map->config.x_size*map->config.resolution, sizeof(double **));
		carmen_test_alloc(cached_data);
		memset(cached_data, 0, map->config.x_size*
				map->config.resolution*sizeof(double **));
	}

	x = carmen_round(traj_point->x);
	y = carmen_round(traj_point->y);

	if (cached_data[x] == NULL)
	{
		cached_data[x] = (double **)
			calloc(map->config.y_size*map->config.resolution, sizeof(double *));
		carmen_test_alloc(cached_data[x]);
		memset(cached_data[x], 0, map->config.y_size*
				map->config.resolution*sizeof(double *));
	}

	if (cached_data[x][y] == NULL)
	{
		cache_misses++;
		cached_data[x][y] = (double *)
			calloc(num_points_to_generate, sizeof(double));
		carmen_test_alloc(cached_data[x][y]);

		dist_ptr = cached_data[x][y];
		laser_theta = 0;
		separation = 2*M_PI/num_points_to_generate;
		for (index = 0; index < num_points_to_generate; index++)
		{
			dist = carmen_geometry_compute_expected_distance
					(traj_point, laser_theta, map);
			*(dist_ptr++) = dist;
			laser_theta = carmen_normalize_theta(laser_theta+separation);
		}
	}
	else
		cache_hits++;

	laser_data_ptr = laser_data;
	offset = carmen_round(carmen_radians_to_degrees
			(carmen_normalize_theta(start_theta)));
	if (offset < 0)
		offset += 360;
	offset = carmen_round(offset/360.0*num_points_to_generate);
	dist_ptr = cached_data[x][y] + offset;
	for (index = 0; index < num_points; index++)
	{
		*(laser_data_ptr++) = *(dist_ptr++);
		offset++;
		if (offset >= num_points)
		{
			offset = 0;
			dist_ptr = cached_data[x][y];
		}
	}
}

void 
carmen_geometry_cache_stats(int *hits, int *misses) 
{
	*hits = cache_hits;
	*misses = cache_misses;
}

void 
carmen_geometry_map_to_cspace(carmen_map_p map, 
		carmen_robot_config_t *robot_conf)
{
	int x_index, y_index, index;
	double *map_ptr;
	int x, y;
	double value = 0.0;
	double resolution = map->config.resolution;
	double downgrade;

	// This line makes downgrade equal to the number of cells that half the
	// robot width consumes
	downgrade = robot_conf->width/2/resolution;

	// This line makes downgrade equal to the amount to take off so that
	// half the width of the robot away from a cell of value 1, the cell
	// has value 0.5

	downgrade = 0.5 / downgrade;

	/* Loop through map, starting at top left, updating value of cell
     as max of itself and most expensive neighbour less the cost 
     downgrade. */

	map_ptr = map->complete_map;
	for (x_index = 0; x_index < map->config.x_size; x_index++)
		for (y_index = 0; y_index < map->config.y_size; y_index++) {
			value = *map_ptr;
			if (value >= 0 && value < 0.1)
				value = 0.1;
			else
				value = 1.0;
			*(map_ptr++) = value;
		}

	for (x_index = 1; x_index < map->config.x_size-1; x_index++)
	{
		map_ptr = map->map[x_index];
		for (y_index = 1; y_index < map->config.y_size-1; y_index++, map_ptr++)
		{
			for (index = 0; index < CARMEN_NUM_OFFSETS; index++)
			{
				x = x_index + carmen_geometry_x_offset[index];
				y = y_index + carmen_geometry_y_offset[index];

				value = map->map[x][y] - downgrade;
				if (value > *map_ptr)
					*map_ptr = value;
			}
		}
	}

	/* Loop through map again, starting at bottom right, updating value of
     cell as max of itself and most expensive neighbour less the cost 
     downgrade. */

	for (x_index = map->config.x_size-2; x_index >= 1; x_index--)
	{
		map_ptr = map->map[x_index]+map->config.y_size-1;
		for (y_index = map->config.y_size-2; y_index >= 1; y_index--, map_ptr--)
		{
			for (index = 0; index < CARMEN_NUM_OFFSETS; index++)
			{
				x = x_index + carmen_geometry_x_offset[index];
				y = y_index + carmen_geometry_y_offset[index];

				value = map->map[x][y] - downgrade;
				if (value > *map_ptr)
					*map_ptr = value;
			}
		}
	}

	/* Clamp any cell that's higher than 0.5 (close than half the width
     of the robot as impassable. Also, fix map so that the clear space
     is 0.0 */

	for (x_index = 1; x_index < map->config.x_size-1; x_index++)
	{
		map_ptr = map->map[x_index];
		for (y_index = 1; y_index < map->config.y_size-1; y_index++)
		{
			value = *map_ptr;
			if (value >= 0 && value < 0.5)
				value = 0.0;
			else if (value >= 0.5)
				value = 1.0;
			*(map_ptr++) = value;
		}
	}
}

#endif
