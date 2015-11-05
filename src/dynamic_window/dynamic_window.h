/*
 * dynamic_window.h
 *
 *  Created on: 17/10/2011
 *      Author: rradaelli
 */

#ifndef DYNAMIC_WINDOW_H_
#define DYNAMIC_WINDOW_H_

#include <carmen/carmen.h>

/*ROBOT STATE (pose, velocity)*/
carmen_point_t pose;
carmen_base_ackerman_odometry_message current_state;

/*TARGET INFORMATION*/
carmen_point_t target_pose;
int is_target_set;

/*ROBOT CONFIGURATION*/
double distance_between_rear_wheels;
double distance_between_front_and_rear_axles;
double acceleration;
double deceleration;
double phi_acceleration;
double max_phi;
double max_vel;
double width;
double height;

/*DYNAMIC WINDOW CONFIGURATIONS*/
double interval_time;
double max_distance;//max distance that dist function will check
//weights
double alfa;
double beta;
double gama;

/*MAP*/
carmen_map_t map;

/**
 * Measures the alignment of the robot with the target direction
 */
double heading(carmen_base_ackerman_velocity_message command);

/**
 * Return the distance to the closest obstacle that intersects with
the curvature
 * otherwise return a big constant
 */
double dist(carmen_base_ackerman_velocity_message command);

/**
 * Evaluate the progress of the robot on the corresponding trajectory.
 */
double velocity(carmen_base_ackerman_velocity_message command);

double objective_function(carmen_base_ackerman_velocity_message command);

/**
 * Execute one cycle of the dynamic window
 */
void execute();

/**
 * Predict the next pose given a command
 */
carmen_point_t predict_next_pose(carmen_point_t pose,
carmen_base_ackerman_velocity_message command, double interval_time);

/**
 * Return true if the robot is on an obstacle
 * otherwise return false
 */
int is_obstacle(carmen_point_t pose);

/**
 * Return true if the point is an obstacle
 * otherwise return false
 */
int is_obstacle_point(int x, int y);

/**
 * Return true if x and y are a valid map position
 * otherwise return false
 */
int is_valid_position(int x, int y);

/**
 * Print a map squared with width w
 * centered in the robot position
 */
void print_map(int w);

/**
 * Command the robot to
 * drive at velocity v and steering angle phi
 */
void send_command(double v, double phi);


#endif /* DYNAMIC_WINDOW_H_ */
