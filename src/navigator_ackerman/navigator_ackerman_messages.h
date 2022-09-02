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

/** @addtogroup navigator **/
// @{

/** \file navigator_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_NAVIGATOR_ACKERMAN_MESSAGES_H
#define CARMEN_NAVIGATOR_ACKERMAN_MESSAGES_H

#include <carmen/global.h>
#include <carmen/map.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_NAVIGATOR_ACKERMAN_STATUS_QUERY_NAME     "carmen_navigator_ackerman_status_query"
typedef carmen_default_message carmen_navigator_ackerman_status_query_message;
#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_QUERY_NAME       "carmen_navigator_ackerman_plan_query"
typedef carmen_default_message carmen_navigator_ackerman_plan_query_message;
#define CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME             "carmen_navigator_ackerman_stop"
typedef carmen_default_message carmen_navigator_ackerman_stop_message;
#define CARMEN_NAVIGATOR_ACKERMAN_GO_NAME               "carmen_navigator_ackerman_go"
typedef carmen_default_message carmen_navigator_ackerman_go_message;

/** This message is published by the navigator, and describes its current
      status. 
 */

typedef struct {
	int autonomous;                /**< Is the navigator actively moving the robot to the goal?
                                      This field should change to 0 whenever the navigator
				      receives a stop message, and change to 1 whenever
				      the navigator receives a go message. */
	int goal_set;                  /**< Is there a current goal? */
	carmen_robot_and_trailers_traj_point_t goal;           /**< Undefined if goal_set is 0 */
	carmen_robot_and_trailers_traj_point_t robot;     /**< The current position of the robot. */
	double timestamp;
	char *host;
} carmen_navigator_ackerman_status_message;

#define CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME       "carmen_navigator_ackerman_status"
#define CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT        "{int,int,{double, double, double, double, double, double},{double, double, double, double, double, double},double,string}"

/** This message is published by the navigator. The current path of the
      navigator. Should never be emitted without a goal. If the goal is
      inaccessible, then the navigator will try and move the robot as close to
      the goal as possible. The path points will include the current robot and
      goal. The path points are given in the reference frame of the current
      map. 
 */

typedef struct {
	carmen_robot_and_trailers_traj_point_t *path;
	int path_length;
	double timestamp;
	char *host;
} carmen_navigator_ackerman_plan_message;

#define      CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME       "carmen_navigator_ackerman_plan"
#define      CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT        "{<{double, double, double, double, double, double}:2>,int,double,string}"


/** This message is sent to the navigator by other programs wishing to plan
      and move towards an (x,y) position. 
 */

typedef struct {
	double x, y, theta;           /**< It is assumed that (x, y,theta) is in the reference frame of
                              the current map. */ 
	double timestamp;
	char *host;
} carmen_navigator_ackerman_set_goal_message;

#define      CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME         "carmen_navigator_ackerman_set_goal"
#define      CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT          "{double,double,double,double,string}"

/** This message is sent to the navigator by other programs wishing to plan
      and move towards an (x,y) position and (theta) orientation. 
 */

typedef struct {
	carmen_robot_and_trailers_traj_point_t goal;   /**< It is assumed that (x, y) is in the reference
			      frame of the current map. Using this function causes
                              the planner to also arrive at the goal with an orientation
                              that matches the theta field of the goal point.  */ 
	double timestamp;
	char *host;
} carmen_navigator_ackerman_set_goal_triplet_message;

#define      CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_TRIPLET_NAME         "carmen_navigator_ackerman_set_goal_triplet"
#define      CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_TRIPLET_FMT          "{{double,double,double,double,double,double},double,string}"

typedef enum{CARMEN_NAVIGATOR_ACKERMAN_GOAL_REACHED_v,
	CARMEN_NAVIGATOR_ACKERMAN_USER_STOPPED_v,
	CARMEN_NAVIGATOR_ACKERMAN_UNKNOWN_v} carmen_navigator_ackerman_reason_t;

	/** This message is published by the navigator whenever it stops trying to move
      towards the current goal. 
	 */

	typedef struct {
		carmen_navigator_ackerman_reason_t reason; /**< Reasons are: that the goal was
				       reached, a user stopped autonomous mode
				       (by publishing a stop message) or due
				       to some unknown failure. */ 
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_autonomous_stopped_message;

#define      CARMEN_NAVIGATOR_ACKERMAN_AUTONOMOUS_STOPPED_NAME "carmen_navigator_ackerman_autonomous_stopped"
#define      CARMEN_NAVIGATOR_ACKERMAN_AUTONOMOUS_STOPPED_FMT "{int,double,string}"

	typedef enum {CARMEN_NAVIGATOR_ACKERMAN_MAP_v, CARMEN_NAVIGATOR_ACKERMAN_ENTROPY_v,
		CARMEN_NAVIGATOR_ACKERMAN_COST_v, CARMEN_NAVIGATOR_ACKERMAN_UTILITY_v,
		CARMEN_LOCALIZE_ACKERMAN_LMAP_v, CARMEN_LOCALIZE_ACKERMAN_GMAP_v}
	carmen_navigator_ackerman_map_t;

	/** This message is sent to the navigator to get one of its current maps,
      including the current occupancy grid (including updates from recent
      laser data), cost maps and utility functions. 
	 */

	typedef struct {
		carmen_navigator_ackerman_map_t map_type;
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_map_request_message;

#define CARMEN_NAVIGATOR_ACKERMAN_MAP_REQUEST_NAME "carmen_navigator_ackerman_map_request"
#define CARMEN_NAVIGATOR_ACKERMAN_MAP_REQUEST_FMT "{int,double,string}"

	/** This message is returned by the navigator after a map request is
      received.
	 */

	typedef struct {
		unsigned char *data;
		int size;
		int compressed;
		carmen_map_config_t config;
		carmen_navigator_ackerman_map_t map_type;
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_map_message;

#define CARMEN_NAVIGATOR_ACKERMAN_MAP_NAME "carmen_navigator_ackerman_map"
#define CARMEN_NAVIGATOR_ACKERMAN_MAP_FMT "{<char:2>,int,int,{int,int,double,[byte:64], string, double, double},int,double,string}"

	/** This message is sent to the navigator to set a goal point that
      corresponds to some named place in the map. See
      carmen_navigator_ackerman_set_goal_place() for more information. 
	 */

	typedef struct {
		char *placename;
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_placename_message;

#define CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_PLACE_NAME "carmen_navigator_ackerman_set_goal_place"
#define CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_PLACE_FMT "{string,double,string}"

	typedef struct {
		int code;
		char *error;
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_return_code_message;

#define CARMEN_NAVIGATOR_ACKERMAN_RETURN_CODE_NAME "carmen_navigator_ackerman_return_code"
#define CARMEN_NAVIGATOR_ACKERMAN_RETURN_CODE_FMT "{int,string,double,string}"

	/**
    Attribute can be one of:
    "robot colour"
    "goal colour" 
    "path colour" 
    "track robot"
    "draw waypoints"
    "show particles"
    "show gaussians"
    "show laser"
    "show simulator"
    "show people"

    The colours are RGB in int form, i.e., R << 16 | G << 8 | B.
    A colour of -1 restores the default colour.
    The other attributes are binary. -1 is restore default, 0 is turn off,
    1 is turn on, other values are ignored.

    Setting the reset_all_to_defaults flag causes all attributes
    to be reset to defaults.

    The status_message is currently ignored. There will eventually be
    a status window that will get the contents of this field.

	 */

	typedef struct {
		char *attribute;
		int value;
		char *status_message;
		int reset_all_to_defaults;
		double timestamp;
		char *host;
	} carmen_navigator_ackerman_display_config_message;

#define CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME "carmen_navigator_ackerman_display_config"
#define CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_FMT "{string, int, string, int, double, string}"

/*	typedef struct {
		carmen_ackerman_traj_point_p point;
		int lenght;
	} carmen_navigator_ackerman_astar_states_message;

#define      CARMEN_NAVIGATOR_ACKERMAN_ASTAR_STATES_NAME       "carmen_navigator_ackerman_astar_states"
#define      CARMEN_NAVIGATOR_ACKERMAN_ASTAR_STATES_FMT        "{<{double, double, double, double, double}:2>,int}" */

#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_NUM_PATHS	100 // Se mecher, tem que alterar o CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT
#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE	500 // Se mecher, tem que alterar o CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT

typedef struct
{
	int	   num_edges;//int
	carmen_robot_and_trailers_traj_point_t *p1;//<{double,double,double,double,double}:1>
	carmen_robot_and_trailers_traj_point_t *p2;//<{double,double,double,double,double}:1>
	int *mask;//<int:1>

	carmen_robot_and_trailers_traj_point_t paths[CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_NUM_PATHS][CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE];//[{double,double,double,double,double}:100, 1000]
	int path_size[CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_NUM_PATHS];//[int:100]
	int num_path;//int

	double timestamp;//double
	char  *host;//string
} carmen_navigator_ackerman_plan_tree_message;

#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME "carmen_navigator_ackerman_plan_tree"
#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT "{int,<{double,double,double,double,double,double}:1>,<{double,double,double,double,double,double}:1>,<int:1>,[{double,double,double,double,double,double}:100, 500],[int:100],int,double,string}"

#define CARMEN_NAVIGATOR_ACKERMAN_GOAL_PLAN_TREE_NAME "carmen_navigator_ackerman_goal_plan_tree"
#define CARMEN_NAVIGATOR_ACKERMAN_GOAL_PLAN_TREE_FMT CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT

typedef struct
{
	int path_size;
	carmen_robot_and_trailers_traj_point_t *path;
	double timestamp;//double
	char  *host;//string
} carmen_navigator_ackerman_plan_to_draw_message;

#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME "carmen_navigator_ackerman_plan_to_draw_message"
#define CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_FMT "{int,<{double,double,double,double,double,double}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
