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

#include "robot_ackerman_central.h"
#include "robot_ackerman_main.h"
#include "robot_ackerman_laser.h"
#include "robot_ackerman_messages.h"
#include <carmen/carmen.h>

static double frontlaser_offset;
static double rearlaser_offset;
static double frontlaser_side_offset=0.;
static double rearlaser_side_offset=0.;
static double frontlaser_angular_offset=0.;
static double rearlaser_angular_offset=0.;

static int frontlaser_id = 1;
static int rearlaser_id = 2;
static int frontlaser_use = 1;
static int rearlaser_use = 1;

static carmen_running_average_t frontlaser_average;
static carmen_running_average_t rearlaser_average;

static double carmen_robot_ackerman_laser_bearing_skip_rate = 0.33;

static carmen_laser_laser_message front_laser, rear_laser;
static int front_laser_count = 0, rear_laser_count = 0;
static int front_laser_ready = 0, rear_laser_ready = 0;
static carmen_robot_ackerman_laser_message robot_front_laser, robot_rear_laser;

static double max_front_velocity = 0;
static double min_rear_velocity = -0;

double carmen_robot_ackerman_interpolate_heading(double head1, double head2, double fraction);

static void 
publish_frontlaser_message(carmen_robot_ackerman_laser_message laser_msg)
{
  IPC_RETURN_TYPE err;
  err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, &laser_msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);
  if(robot_front_laser.host!=NULL){
    free(robot_front_laser.host);
  }
}

static void 
publish_rearlaser_message(carmen_robot_ackerman_laser_message laser_msg)
{
  IPC_RETURN_TYPE err;

  err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, &laser_msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME);
}

/** returns 0 if time estimate is not ready, otherwise 1 **/
static int
construct_laser_message(carmen_robot_ackerman_laser_message *msg, int rear, double timestamp)
{
  int low;
  int high;
  double skew;
  double fraction;
  int laser_ready;

  if (!rear) {
    laser_ready = carmen_robot_ackerman_get_skew(front_laser_count, &skew,
					&frontlaser_average, msg->host);
    if (!laser_ready) {
      carmen_warn("Waiting for front laser data to accumulate in order to estimate the time skew.\n");
      return 0;
    }

  } else {
    laser_ready = carmen_robot_ackerman_get_skew(rear_laser_count, &skew,
					&rearlaser_average, msg->host);
    if (!laser_ready) {
      carmen_warn("Waiting for rear laser data to accumulate in order to estimate the time skew.\n");
      return 0;
    }
  }

  fraction = carmen_robot_ackerman_get_fraction(timestamp, skew, &low, &high);

  if (!carmen_robot_ackerman_config.interpolate_odometry) {
    // take the latest odometry measurement
    fraction=0;
    high = CARMEN_ROBOT_ACKERMAN_MAX_READINGS-1;
    low = CARMEN_ROBOT_ACKERMAN_MAX_READINGS-1;
  }


  msg->robot_pose.x=carmen_robot_ackerman_odometry[low].x + fraction *
    (carmen_robot_ackerman_odometry[high].x - carmen_robot_ackerman_odometry[low].x);
  msg->robot_pose.y= carmen_robot_ackerman_odometry[low].y + fraction *
    (carmen_robot_ackerman_odometry[high].y - carmen_robot_ackerman_odometry[low].y);
  msg->robot_pose.theta=carmen_robot_ackerman_interpolate_heading
    (carmen_robot_ackerman_odometry[low].theta, 
     carmen_robot_ackerman_odometry[high].theta, fraction);
  msg->robot_pose.theta=carmen_normalize_theta(msg->robot_pose.theta);

  msg->v = carmen_robot_ackerman_odometry[low].v +
    fraction*(carmen_robot_ackerman_odometry[high].v -
	      carmen_robot_ackerman_odometry[low].v);
  msg->phi = carmen_robot_ackerman_odometry[low].phi +
    fraction*(carmen_robot_ackerman_odometry[high].phi -
	      carmen_robot_ackerman_odometry[low].phi);

  if (! rear){
    double s=sin(msg->robot_pose.theta), c=cos(msg->robot_pose.theta);
    msg->laser_pose.x=msg->robot_pose.x+c*frontlaser_offset-s*frontlaser_side_offset;
    msg->laser_pose.y=msg->robot_pose.y+s*frontlaser_offset+c*frontlaser_side_offset;
    msg->laser_pose.theta=msg->robot_pose.theta + frontlaser_angular_offset;
  } else {
    double s=sin(msg->robot_pose.theta), c=cos(msg->robot_pose.theta);
    msg->laser_pose.x=msg->robot_pose.x+c*rearlaser_offset-s*rearlaser_side_offset;
    msg->laser_pose.y=msg->robot_pose.y+s*rearlaser_offset+c*rearlaser_side_offset;
    msg->laser_pose.theta=msg->robot_pose.theta + rearlaser_angular_offset;
  }
  msg->laser_pose.theta = 
    carmen_normalize_theta(msg->laser_pose.theta);
  return 1;
}

void 
carmen_robot_ackerman_correct_laser_and_publish(void) 
{
  if(!front_laser_ready && !rear_laser_ready) 
    return;

  if (front_laser_ready) {
    if (construct_laser_message(&robot_front_laser, 0, front_laser.timestamp)) {
      //fprintf(stderr, "f");
      publish_frontlaser_message(robot_front_laser);
    }
    front_laser_ready = 0;   
  }

  if (rear_laser_ready) {
    if (construct_laser_message(&robot_rear_laser, 1, rear_laser.timestamp) ) {
      fprintf(stderr, "r");
      publish_rearlaser_message(robot_rear_laser);
    }
    rear_laser_ready = 0;
  }
  
}

static void 
check_message_data_chunk_sizes(carmen_laser_laser_message *laser_ptr)
{
  static int first_front = 1, first_rear = 1;
  int first;
  carmen_robot_ackerman_laser_message robot_laser;
  carmen_laser_laser_message laser;

  if (laser_ptr == &front_laser) {
    first = first_front;
    robot_laser = robot_front_laser;
  } else {
    first = first_rear;
    robot_laser = robot_rear_laser;
  }

  laser = *laser_ptr;

  if(first) {
    robot_laser.num_readings = laser.num_readings;
    robot_laser.range = 
      (double *)calloc(robot_laser.num_readings, sizeof(double));
    carmen_test_alloc(robot_laser.range);
    robot_laser.tooclose = 
      (char *)calloc(robot_laser.num_readings, sizeof(char));
    carmen_test_alloc(robot_laser.tooclose);   

    robot_laser.num_remissions = laser.num_remissions;
    
    if (robot_laser.num_remissions>0) {
      robot_laser.remission = 
	(double *)calloc(robot_laser.num_remissions, sizeof(double));
      carmen_test_alloc(robot_laser.remission);
    }
    else 
      robot_laser.remission = NULL;
    
    first = 0;
  } else if(robot_laser.num_readings != laser.num_readings) {
    robot_laser.num_readings = laser.num_readings;
    if (robot_laser.num_readings <= 0) {
      free(robot_laser.range);
      robot_laser.range = NULL;
      free(robot_laser.tooclose);
      robot_laser.tooclose = NULL;
      robot_laser.num_readings = 0;
    } 
    else {
      
      robot_laser.range = 
	(double *)realloc(robot_laser.range,
			 sizeof(double) * robot_laser.num_readings);
      carmen_test_alloc(robot_laser.range);
      robot_laser.tooclose = (char *)realloc
	(robot_laser.tooclose, sizeof(char) * robot_laser.num_readings);
      carmen_test_alloc(robot_laser.tooclose);
    }

    robot_laser.num_remissions = laser.num_remissions;
    if (robot_laser.num_remissions>0) {
      robot_laser.remission = 
	(double *)realloc(robot_laser.remission,
			 sizeof(double) * robot_laser.num_remissions);
      carmen_test_alloc(robot_laser.remission);
    }
    else {
      robot_laser.num_remissions = 0;
      robot_laser.remission = NULL;
    }

  }

  if (laser_ptr == &front_laser) {
    first_front = first;
    robot_front_laser = robot_laser;
  } else {
    first_rear = first;
    robot_rear_laser = robot_laser;
  }
}

static void 
laser_frontlaser_handler(void)
{
  int i;
  double safety_distance;
  double theta, delta_theta;
  carmen_traj_point_t robot_posn, obstacle_pt;
  double max_velocity, velocity;

/*  int tooclose = -1;
  double tooclose_theta = 0;

  int min_index;*/
  //  double min_theta = 0;
  double min_dist;
  

  static double time_since_last_process = 0;
  double skip_sum = 0;



  /* We just got a new laser message. It may be that the new message contains
     a different number of laser readings than we were expecting, maybe
     because the laser server was restarted. Here, we check to make sure that
     our outgoing messages can handle the same number of readings as the
     newly-arrived incoming message. */

  check_message_data_chunk_sizes(&front_laser);

  carmen_robot_ackerman_update_skew(&frontlaser_average, &front_laser_count, 
			   front_laser.timestamp, front_laser.host);

  memcpy(robot_front_laser.range, front_laser.range, 
	 robot_front_laser.num_readings * sizeof(double));
  memset(robot_front_laser.tooclose, 0, robot_front_laser.num_readings);
  if (robot_front_laser.num_remissions>0)
    memcpy(robot_front_laser.remission, front_laser.remission, 
	   robot_front_laser.num_remissions * sizeof(double));

  robot_front_laser.config = front_laser.config;

  carmen_robot_ackerman_sensor_time_of_last_update = carmen_get_time();

  if (carmen_robot_ackerman_sensor_time_of_last_update - time_since_last_process < 
      1.0/carmen_robot_ackerman_collision_avoidance_frequency)
    return;


  robot_posn.x = 0;
  robot_posn.y = 0;
  robot_posn.theta = 0;
  robot_posn.t_vel = carmen_robot_ackerman_latest_odometry.v;
  robot_posn.r_vel = carmen_robot_ackerman_latest_odometry.phi;

  time_since_last_process = carmen_robot_ackerman_sensor_time_of_last_update;

  safety_distance = carmen_geometry_compute_safety_ackerman_distance(&carmen_robot_ackerman_config, &robot_posn);

  robot_front_laser.forward_safety_dist = safety_distance;
  robot_front_laser.side_safety_dist = 
    0.5 * carmen_robot_ackerman_config.width + carmen_robot_ackerman_config.side_dist;
  robot_front_laser.turn_axis = 1e6;
  robot_front_laser.timestamp = front_laser.timestamp;


  robot_front_laser.host = carmen_new_string(front_laser.host);

  max_velocity = carmen_robot_ackerman_config.max_v;

  carmen_carp_set_verbose(0);
  
  theta = front_laser.config.start_angle + frontlaser_angular_offset;
  delta_theta = front_laser.config.angular_resolution;

  skip_sum = 0.0;
  for(i = 0; i < robot_front_laser.num_readings; i++, theta += delta_theta) {
    skip_sum += carmen_robot_ackerman_laser_bearing_skip_rate;
    if (skip_sum > 0.95) {
      skip_sum = 0.0;
      robot_front_laser.tooclose[i] = -1;
      continue;
    }
    
    obstacle_pt.x = frontlaser_offset     +  robot_front_laser.range[i] * cos(theta);
    obstacle_pt.y = frontlaser_side_offset + robot_front_laser.range[i] * sin(theta);
    carmen_geometry_move_pt_to_rotating_ref_frame
      (&obstacle_pt, carmen_robot_ackerman_latest_odometry.v,
       carmen_robot_ackerman_latest_odometry.phi);    
    velocity = carmen_geometry_compute_ackerman_velocity
      (robot_posn, obstacle_pt, &carmen_robot_ackerman_config);

    if (velocity < carmen_robot_ackerman_config.max_v) {
      if (velocity < max_velocity) {
	/*tooclose = i;
	tooclose_theta = theta;*/
	max_velocity = velocity;
      }
      robot_front_laser.tooclose[i] = 1;
    }
  } /* End of for(i = 0; i < robot_front_laser.num_readings; i++) */

//  min_index = 0;
  min_dist = robot_front_laser.range[0];
  for(i = 1; i < robot_front_laser.num_readings; i++) { 
    if (robot_front_laser.range[i] < min_dist) {
      min_dist = robot_front_laser.range[i];
//      min_index = i;
    }
  }

  front_laser_ready = 1;
  
  if (max_velocity >= 0 && max_velocity < CARMEN_ROBOT_ACKERMAN_MIN_ALLOWED_VELOCITY)
    max_velocity = 0.0;
  
  if(max_velocity <= 0.0 && carmen_robot_ackerman_latest_odometry.v > 0.0)    {
    fprintf(stderr, "S");
    carmen_robot_ackerman_stop_robot(CARMEN_ROBOT_ACKERMAN_ALLOW_ROTATE);
  }
  
  max_front_velocity = max_velocity;
}

double 
carmen_robot_ackerman_laser_max_front_velocity(void) 
{
  return max_front_velocity;
}

double 
carmen_robot_ackerman_laser_min_rear_velocity(void) 
{
  return min_rear_velocity;
}

static void 
laser_rearlaser_handler(void) 
{
  int i;
  double safety_distance;
  double theta, delta_theta;
  carmen_traj_point_t robot_posn, obstacle_pt;
  double min_velocity, velocity;

/*  int tooclose = -1;
  double tooclose_theta = 0;*/

  static double time_since_last_process = 0;
  double skip_sum = 0;

  /* We just got a new laser message. It may be that the new message contains
     a different number of laser readings than we were expecting, maybe
     because the laser server was restarted. Here, we check to make sure that
     our outgoing messages can handle the same number of readings as the
     newly-arrived incoming message. */

  check_message_data_chunk_sizes(&rear_laser);

  carmen_robot_ackerman_update_skew(&rearlaser_average, &rear_laser_count, 
			   rear_laser.timestamp, rear_laser.host);

  memcpy(robot_rear_laser.range, rear_laser.range, 
	 robot_rear_laser.num_readings * sizeof(double));
  memset(robot_rear_laser.tooclose, 0, robot_rear_laser.num_readings);
  if (robot_rear_laser.num_remissions>0)
    memcpy(robot_rear_laser.remission, rear_laser.remission, 
	   robot_rear_laser.num_remissions * sizeof(double));

  robot_rear_laser.config = rear_laser.config;

  carmen_robot_ackerman_sensor_time_of_last_update = carmen_get_time();

  if (carmen_robot_ackerman_sensor_time_of_last_update - time_since_last_process < 
      1.0/carmen_robot_ackerman_collision_avoidance_frequency)
    return;

  time_since_last_process = carmen_robot_ackerman_sensor_time_of_last_update;

  safety_distance = carmen_geometry_compute_safety_ackerman_distance(&carmen_robot_ackerman_config, &robot_posn);

  robot_rear_laser.forward_safety_dist = safety_distance;
  robot_rear_laser.side_safety_dist = 
    carmen_robot_ackerman_config.width / 2.0 + carmen_robot_ackerman_config.side_dist;
  robot_rear_laser.turn_axis = 1e6;
  robot_rear_laser.timestamp = rear_laser.timestamp;
  robot_rear_laser.host = carmen_new_string(rear_laser.host);

  min_velocity = -carmen_robot_ackerman_config.max_v;

  robot_posn.x = 0;
  robot_posn.y = 0;
  robot_posn.theta = 0;
  robot_posn.t_vel = carmen_robot_ackerman_latest_odometry.v;
  robot_posn.r_vel = carmen_robot_ackerman_latest_odometry.phi;

/*   theta = -M_PI/2;  */
/*   delta_theta = M_PI/(robot_rear_laser.num_readings-1); */

  theta = rearlaser_angular_offset + rear_laser.config.start_angle;
  delta_theta = rear_laser.config.angular_resolution;

  skip_sum = 0.0;
  for(i = 0; i < robot_rear_laser.num_readings; i++, theta += delta_theta) {
    skip_sum += carmen_robot_ackerman_laser_bearing_skip_rate;
    if (skip_sum > 0.95) {
      skip_sum = 0.0;
      robot_rear_laser.tooclose[i] = -1;
      continue;
    }
    
    obstacle_pt.x = rearlaser_offset + robot_rear_laser.range[i] * cos(theta);
    obstacle_pt.y = rearlaser_side_offset + robot_rear_laser.range[i] * sin(theta);
    carmen_geometry_move_pt_to_rotating_ref_frame
      (&obstacle_pt, carmen_robot_ackerman_latest_odometry.v,
       carmen_robot_ackerman_latest_odometry.phi);    
    velocity = carmen_geometry_compute_ackerman_velocity
      (robot_posn, obstacle_pt, &carmen_robot_ackerman_config);    
    velocity = -velocity;

    if (velocity > -carmen_robot_ackerman_config.max_v) {
      if (velocity > min_velocity) {
	/*tooclose = i;
	tooclose_theta = theta;*/
	min_velocity = velocity;
      }
      robot_rear_laser.tooclose[i] = 1;
    }
  } /* End of for(i = 0; i < robot_rear_laser.num_readings; i++) */

  rear_laser_ready = 1;

  if (min_velocity <= 0 && min_velocity > -.03)
    min_velocity = 0.0;

  if(min_velocity >= 0.0 && carmen_robot_ackerman_latest_odometry.v < 0.0)
    {
      fprintf(stderr, "S");
      carmen_robot_ackerman_stop_robot(CARMEN_ROBOT_ACKERMAN_ALLOW_ROTATE);
    }
  
  min_rear_velocity = min_velocity;
}

void 
carmen_robot_ackerman_add_laser_handlers(void) 
{
  IPC_RETURN_TYPE err;

  /* define messages created by this module */
  err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

  err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME,
                      IPC_VARIABLE_LENGTH,
                      CARMEN_ROBOT_ACKERMAN_REARLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME);


  if (frontlaser_use) {
    fprintf(stderr, "Subscribing front laser to RAWLASER%d\n", frontlaser_id);
    carmen_laser_subscribe_laser_message(frontlaser_id, &front_laser,
        (carmen_handler_t)laser_frontlaser_handler,
        CARMEN_SUBSCRIBE_LATEST);
  }
  if (rearlaser_use) {
    fprintf(stderr, "Subscribing rear laser to RAWLASER%d\n", rearlaser_id);
    carmen_laser_subscribe_laser_message(rearlaser_id, &rear_laser,
        (carmen_handler_t)laser_rearlaser_handler,
        CARMEN_SUBSCRIBE_LATEST);
  }

  carmen_running_average_clear(&frontlaser_average);
  carmen_running_average_clear(&rearlaser_average);
}

void 
carmen_robot_ackerman_add_laser_parameters(int argc, char** argv) 
{
  int num_items;

  carmen_param_t param_list[] = {
    {"robot", "frontlaser_offset",       CARMEN_PARAM_DOUBLE, &frontlaser_offset, 1, NULL},
    {"robot", "frontlaser_side_offset",  CARMEN_PARAM_DOUBLE, &frontlaser_side_offset, 1, NULL},
    {"robot", "frontlaser_angular_offset",  CARMEN_PARAM_DOUBLE, &frontlaser_angular_offset, 1, NULL},
    {"robot", "frontlaser_use",          CARMEN_PARAM_ONOFF, &frontlaser_use, 0, NULL},
    {"robot", "frontlaser_id",           CARMEN_PARAM_INT, &frontlaser_id, 0, NULL},
    {"robot", "rearlaser_offset",        CARMEN_PARAM_DOUBLE, &rearlaser_offset, 1, NULL},
    {"robot", "rearlaser_side_offset",  CARMEN_PARAM_DOUBLE, &rearlaser_side_offset, 1, NULL},
    {"robot", "rearlaser_angular_offset",  CARMEN_PARAM_DOUBLE, &rearlaser_angular_offset, 1, NULL},
    {"robot", "rearlaser_use",           CARMEN_PARAM_ONOFF, &rearlaser_use, 0, NULL},
    {"robot", "rearlaser_id",            CARMEN_PARAM_INT, &rearlaser_id, 0, NULL},
    //{"robot", "laser_bearing_skip_rate", CARMEN_PARAM_DOUBLE, &carmen_robot_laser_bearing_skip_rate, 1, NULL},
  };
  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);
}
