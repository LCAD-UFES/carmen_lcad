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

#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/map_graphics.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_interface.h>
#include <X11/Xlib.h>
#include <X11/cursorfont.h>
#include <ctype.h>

#include "navigator_panel.h"
#include "navigator_graphics.h"
#include <carmen/map_server_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/obstacle_avoider_messages.h>
#include <carmen/obstacle_avoider_interface.h>

#undef USE_DOT

#ifdef USE_DOT

#include <carmen/dot.h>
#include <carmen/dot_messages.h>
#include <carmen/dot_interface.h>

#endif

Display *gdk_x11_cursor_get_xdisplay(GdkCursor *cursor);
Cursor	 gdk_x11_cursor_get_xcursor(GdkCursor *cursor);

#define BUTTON_WIDTH 255
#define BUTTON_HEIGHT 30
#define GRADIENT_COLORS 40

#define ALWAYS_REDRAW 0

#define DEFAULT_ROBOT_COLOUR carmen_red
#define DEFAULT_GOAL_COLOUR carmen_yellow
#define DEFAULT_PATH_COLOUR carmen_blue
#define DEFAULT_TREE_COLOUR carmen_orange

#define DEFAULT_PEOPLE_COLOUR carmen_orange

#define DEFAULT_TRACK_ROBOT 1
#define DEFAULT_DRAW_PATH	 1
#define DEFAULT_DRAW_WAYPOINTS 1
#define DEFAULT_DRAW_ROBOT_WAYPOINTS 1
#define DEFAULT_SHOW_LATERAL_OFFSET 0
#define DEFAULT_SHOW_PARTICLES 0
#define DEFAULT_SHOW_FUSED_ODOMETRY 0
#define DEFAULT_SHOW_GAUSSIANS 0
#define DEFAULT_SHOW_LASER 0
#define DEFAULT_SHOW_SIMULATOR 0

#define DEFAULT_SHOW_TRACKED_OBJECTS 1

#define DEFAULT_SHOW_COMMAND_PATH 0  //@@@ colision check
#define DEFAULT_SHOW_MOTION_PATH 0
#define DEFAULT_SHOW_DYNAMIC_OBJECTS 0

typedef void *(*void_func)(void *);

typedef enum
{
	NO_PLACEMENT,
	PLACING_ROBOT,
	ORIENTING_ROBOT,
	PLACING_GOAL,
	ORIENTING_GOAL,
	PLACING_PERSON,
	ORIENTING_PERSON,
	PLACING_SIMULATOR,
	ORIENTING_SIMULATOR,
	SELECTING_FINAL_REGION,
	PLACING_FINAL_GOAL,
	ORIENTING_FINAL_GOAL,
	SELECTING_NEAR_WAYPOINT
} placement_t;

#define MAX_PATH_SIZE 10000

fpointers *queuePoints = NULL;

static char *map_path;

static carmen_navigator_map_t display;
static carmen_map_placelist_p placelist = NULL;
static carmen_list_t *place_action_uids = NULL;
static carmen_list_t *goal_actions	= NULL;
static carmen_list_t *start_actions = NULL;

static double time_of_last_redraw	 = 0;
static int	  display_needs_updating = 0;
static int	  num_plan_tree_points;

static carmen_world_point_t path[MAX_PATH_SIZE];
static int	  num_path_points;

static carmen_world_point_t obstacle_avoider_path[MAX_PATH_SIZE];
static int obstacle_avoider_path_size = 0;

static carmen_world_point_t motion_path[MAX_PATH_SIZE];
static int motion_path_size = 0;

static carmen_world_point_t *plan_tree_p1 = NULL;
static carmen_world_point_t *plan_tree_p2 = NULL;


#define PATH_VECTOR_SIZE 20
static carmen_world_point_t *path_vector[PATH_VECTOR_SIZE];
static int path_vector_size[PATH_VECTOR_SIZE];
static GdkColor *path_vector_color[PATH_VECTOR_SIZE];


static carmen_world_point_t	 goal;
static carmen_world_point_t	 goal_temp;
static carmen_world_point_t	 final_goal;
static carmen_world_point_t	 robot, robot_temp;
static carmen_traj_point_t	 robot_traj;
static carmen_world_point_t	 last_robot;
static carmen_world_point_t	 new_person;
static carmen_world_point_t	 new_simulator;
static carmen_world_point_t *navigator_goal_list = NULL;
static carmen_world_point_t *navigator_waypoint_list = NULL;
static int goal_list_size = 0;
static int waypoint_list_size = 0;

static GdkColor robot_colour, goal_colour, tree_colour, people_colour, path_colour;

static int	 is_filming = 0;
static guint filming_timeout = 0;

static placement_t placement_status = 0;
static carmen_world_point_t cursor_pos;
static double last_navigator_update = 0;
static double last_simulator_update = 0;

static carmen_localize_ackerman_globalpos_message *globalpos;
static carmen_localize_ackerman_particle_message	  particle_msg;
static carmen_localize_ackerman_sensor_message	  sensor_msg;
static carmen_dynamic_object_detector_clustered_objects_message* dynamic_object_detector_clustered_objects_msg = NULL;
static int update_local_map = 1;

carmen_robot_ackerman_road_velocity_control_message road_velocity_control;

GdkColor RedBlueGradient[GRADIENT_COLORS];


static int ignore_click;

static GtkUIManager	  *ui_manager;
static GtkActionGroup *goal_action_group;
static GtkActionGroup *start_action_group;

static GtkMapViewer *map_view;

static GtkWidget *window;
static GtkWidget *autonomous_button;
static GtkWidget *place_robot_button;
static GtkWidget *place_goal_button;
static GtkWidget *decrement_point_button;
static GtkWidget *clear_all_goals_button;
static GtkWidget *robot_status_label;
static GtkWidget *fused_odometry_status_label;
static GtkWidget *robot_speed_label;
static GtkWidget *goal_status_label;
static GtkWidget *cursor_status_label;
static GtkWidget *value_label;
static GtkWidget *map_status_label;
static GtkWidget *map_origin_label;
static GtkWidget *simulator_box;
static GtkWidget *filler_box;
static GtkWidget *place_simulator_button;
static GtkWidget *sync_mode_button;
static GtkWidget *next_tick_button;
static GtkWidget *follow_lane_algorithm_selection;
static GtkWidget *state_selection;
static GtkWidget *parking_algorithm_selection;
static GtkWidget *zoom_in_global_map_button;
static GtkWidget *zoom_out_global_map_button;
static GtkWidget *place_final_goal_button;
static GtkWidget *select_nearest_waypoint_button;
//static GtkWidget *update_map_button;

static carmen_world_point_t simulator_trueposition = {{0, 0, 0}, NULL};
static double time_of_simulator_update = 0;
static double simulator_hidden;

static carmen_world_point_t fused_odometry_position = {{0, 0, 0}, NULL};

static carmen_list_t *simulator_objects = NULL;
static carmen_list_t *people = NULL;

static carmen_robot_ackerman_config_t	 *robot_config;
static carmen_navigator_config_t *nav_config;
static carmen_navigator_panel_config_t *nav_panel_config;
static int global_view = 0;

static void switch_map_display(GtkAction *action, gpointer user_data
		__attribute__ ((unused)));
static void switch_localize_display(GtkAction *action,
		gpointer   user_data
		__attribute__ ((unused)));
static void set_location(GtkAction *action, gpointer user_data
		__attribute__ ((unused)));
static void start_filming(GtkWidget *w __attribute__ ((unused)),
		int		 arg __attribute__ ((unused)));
static gint save_image(gpointer data, guint action, GtkWidget *widget);

static void sync_mode_change_handler(char *module, char *variable, char *value);

static int behavior_selector_active = 0;

static void
initialize_path_vector()
{
	int i;

	for (i = 0; i < PATH_VECTOR_SIZE; i++)
	{
		path_vector[i] = NULL;
		path_vector_size[i] = 0;
	}

	path_vector_color[0] = &carmen_red;
	path_vector_color[1] = &carmen_blue;
	path_vector_color[2] = &carmen_green;
	path_vector_color[3] = &carmen_yellow;
	path_vector_color[4] = &carmen_light_blue;
	path_vector_color[5] = &carmen_black;
	path_vector_color[6] = &carmen_orange;
	path_vector_color[7] = &carmen_grey;
	path_vector_color[8] = &carmen_light_grey;
	path_vector_color[9] = &carmen_purple;
}

static void
do_redraw(void)
{
	if (display_needs_updating &&
			((carmen_get_time() - time_of_last_redraw > 0.025) || ALWAYS_REDRAW))
	{
		carmen_map_graphics_redraw(map_view);
		time_of_last_redraw	   = carmen_get_time();
		display_needs_updating = 0;
	}
}

static void delete_event(GtkWidget *widget, GdkEvent *event, gpointer data)
{
	widget = widget;
	event  = event;
	data   = data;

	gtk_main_quit();
}

static void label_autonomy_button(char *str)
{
	GtkWidget *label;

	label = GTK_BIN(autonomous_button)->child;
	gtk_label_set_text(GTK_LABEL(label), str);
}

static void go_autonomous(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	GtkWidget *label;

	if (!ignore_click)
	{
		if (GTK_TOGGLE_BUTTON(autonomous_button)->active)
		{
			label = GTK_BIN(autonomous_button)->child;
			gtk_label_set_text(GTK_LABEL(label), "Stop");
			navigator_start_moving();
		}
		else
		{
			label = GTK_BIN(autonomous_button)->child;
			gtk_label_set_text(GTK_LABEL(label), "Go");
			navigator_stop_moving();
		}
	}
	else
	{
		ignore_click = 0;
	}

	if(global_view)
	{
		//		get_initial_map();
		global_view = 0;
	}
}

int navigator_graphics_update_map()
{
	if (update_local_map)
		return 1;
	return 0;
}


static void next_tick(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	if (GTK_TOGGLE_BUTTON(sync_mode_button)->active)
	{
		carmen_simulator_ackerman_next_tick();
	}
}

static void sync_mode(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	carmen_param_set_module(NULL);
	carmen_param_set_onoff("simulator_sync_mode", GTK_TOGGLE_BUTTON(
			sync_mode_button)->active, NULL);
}

static void change_cursor(GdkColor *fg, GdkColor *bg)
{
	XColor	   xfg, xbg;
	Display	  *xdisplay;
	Cursor	   xcursor;
	GdkCursor *cursor = gdk_cursor_new(GDK_DOT);

	xfg.pixel = fg->pixel;
	xfg.red	  = fg->red;
	xfg.blue  = fg->blue;
	xfg.green = fg->green;

	xbg.pixel = bg->pixel;
	xbg.red	  = bg->red;
	xbg.blue  = bg->blue;
	xbg.green = bg->green;

	xdisplay = gdk_x11_cursor_get_xdisplay(cursor);
	xcursor	 = gdk_x11_cursor_get_xcursor(cursor);
	XRecolorCursor(xdisplay, xcursor, &xfg, &xbg);

	gdk_window_set_cursor(map_view->image_widget->window, cursor);
}

static void place_robot(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	change_cursor(&carmen_red, &carmen_black);
	placement_status = PLACING_ROBOT;
}

static void place_simulator(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	change_cursor(&carmen_red, &carmen_black);
	placement_status = PLACING_SIMULATOR;
}

static void place_person(GtkWidget *w __attribute__ ((unused)), int arg __attribute__ ((unused)))
{
	change_cursor(&carmen_orange, &carmen_black);
	placement_status = PLACING_PERSON;
}

static void clear_objects(GtkWidget *w __attribute__ ((unused)), int arg __attribute__ ((unused)))
{
	carmen_simulator_ackerman_clear_objects();
}

static void place_goal(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	change_cursor(&carmen_yellow, &carmen_black);
	placement_status = PLACING_GOAL;
}


static int selecting_near_waypoint_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point)
{

	if ( (placement_status == SELECTING_NEAR_WAYPOINT) )
	{
		carmen_rddf_publish_end_point_message(1, world_point->pose);

		placement_status = NO_PLACEMENT;

		return TRUE;
	}

	return FALSE;
}

static void
zoom_out_global_map_action(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	if (display != CARMEN_COMPLETE_MAP_v)
		return;

	navigator_graphics_change_map(navigator_get_complete_map_map_pointer());
}

static void
zoom_in_global_map_action(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	if (display != CARMEN_COMPLETE_MAP_v)
		return;

	gdk_window_set_cursor(map_view->image_widget->window, gdk_cursor_new(GDK_BASED_ARROW_DOWN));

	//gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(update_map_button), 0);
	//	gtk_widget_set_sensitive(place_final_goal_button, TRUE);
	gtk_widget_set_sensitive(select_nearest_waypoint_button, FALSE);
	//gtk_widget_set_sensitive(update_map_button, FALSE);

	placement_status = SELECTING_FINAL_REGION;

	//	global_view = 1;
	//	update_local_map = 0;
}

static int selecting_final_region_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point)
{

	if ( (placement_status == SELECTING_FINAL_REGION) )
	{
		carmen_map_t* map;
		carmen_point_t pose;
		pose = world_point->pose;

		map = (carmen_map_t*) malloc(sizeof(carmen_map_t));
		map->complete_map = NULL;
		map->config.map_name = NULL;

		carmen_grid_mapping_get_block_map_by_origin(map_path, 'm', pose, map);

		navigator_graphics_change_map(map);

		placement_status = NO_PLACEMENT;

		gdk_window_set_cursor(the_map_view->image_widget->window, gdk_cursor_new(GDK_LEFT_PTR));

		return TRUE;

	}

	return FALSE;
}

void execute_decrement_point()
{
	if ((queuePoints != NULL) && (queuePoints->begin != NULL))
	{
		pointers *item	  = NULL;
		pointers *itemAux = NULL;

		item = queuePoints->begin;

		while (item->next != NULL)
		{
			itemAux = item;
			item	= item->next;
		}

		if (itemAux == NULL)
		{
			navigator_stop_moving();
			queuePoints->begin = NULL;
			queuePoints->curr  = NULL;
			queuePoints->end   = NULL;
		}
		else
		{
			queuePoints->end = itemAux;
			queuePoints->end->next = NULL;
		}

		free(item);
	}
}

static void decrement_point(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	if (!behavior_selector_active)
	{
		execute_decrement_point();

		//change_cursor(&carmen_yellow, &carmen_black);
	}
	else
	{
		carmen_behavior_selector_remove_goal();
	}

	placement_status = PLACING_GOAL;
}

void execute_clear_all_goals()
{
	if (!behavior_selector_active)
	{
		if (queuePoints != NULL)
		{
			pointers *item = queuePoints->begin;
			pointers *itemAux;

			while (item != NULL)
			{
				itemAux = item;
				item	= item->next;
				free(itemAux);
			}

			queuePoints->begin = NULL;
			queuePoints->curr  = NULL;
			queuePoints->end   = NULL;

			//navigator_unset_goal(item->pt.posiX, item->pt.posiY);
			free(item);
			navigator_set_goal(-1, -1, 0);
			navigator_stop_moving();
		}
	}
	else
	{
		carmen_behavior_selector_clear_goal_list();
	}
}

void
carmen_dynamic_object_detector_handler(carmen_dynamic_object_detector_clustered_objects_message *message)
{
	int i;

	if(dynamic_object_detector_clustered_objects_msg == NULL)
	{
		dynamic_object_detector_clustered_objects_msg =
				(carmen_dynamic_object_detector_clustered_objects_message *) malloc (sizeof(carmen_dynamic_object_detector_clustered_objects_message));
		dynamic_object_detector_clustered_objects_msg->objects_map = (double *) malloc (message->map_size * sizeof(double));
	}

	for(i = 0; i < message->map_size; i++)
		dynamic_object_detector_clustered_objects_msg->objects_map[i] = message->objects_map[i];

	dynamic_object_detector_clustered_objects_msg->map_size = message->map_size;
	dynamic_object_detector_clustered_objects_msg->config = message->config;
	dynamic_object_detector_clustered_objects_msg->timestamp = message->timestamp;
	dynamic_object_detector_clustered_objects_msg->host = message->host;
}

void
road_velocity_control_handler(carmen_robot_ackerman_road_velocity_control_message * msg)
{
	road_velocity_control.left_near_obstacle = msg->left_near_obstacle;
	road_velocity_control.right_near_obstacle = msg->right_near_obstacle;
	road_velocity_control.host = msg->host;
	road_velocity_control.timestamp = msg->timestamp;
}

void
motion_path_handler(carmen_navigator_ackerman_plan_message *msg)
{
	int i;

	if (map_view->internal_map == NULL)
		return;

	if (msg->path_length > MAX_PATH_SIZE)
		carmen_die("path_length > MAX_PATH_SIZE in motion_path_handler()\n");

	motion_path_size = msg->path_length;

	for (i = 0; i < msg->path_length; i++)
	{
		motion_path[i].pose.x	   = msg->path[i].x;
		motion_path[i].pose.y	   = msg->path[i].y;
		motion_path[i].pose.theta  = msg->path[i].theta;
		motion_path[i].map 		   = map_view->internal_map;
	}
}

void
obstacle_avoider_message_handler(carmen_navigator_ackerman_plan_message *msg)
{
	int i;

	if (map_view->internal_map == NULL)
		return;

	if (msg->path_length > MAX_PATH_SIZE)
		carmen_die("path_length > MAX_PATH_SIZE in obstacle_avoider_message_handler()\n");

	obstacle_avoider_path_size = msg->path_length;

	for (i = 0; i < msg->path_length; i++)
	{
		obstacle_avoider_path[i].pose.x	   = msg->path[i].x;
		obstacle_avoider_path[i].pose.y	   = msg->path[i].y;
		obstacle_avoider_path[i].pose.theta = msg->path[i].theta;
		obstacle_avoider_path[i].map = map_view->internal_map;
	}
}

void
navigator_graphics_update_plan(carmen_ackerman_traj_point_p new_plan, int plan_length)
{
	int index;

	if (map_view->internal_map == NULL)
		return;

	if (plan_length > MAX_PATH_SIZE)
		carmen_die("plan_length > MAX_PATH_SIZE in navigator_graphics_update_plan()\n");

	num_path_points = plan_length;

	if (plan_length > 0)
	{
		carmen_verbose("Got path of length %d\n", plan_length);

		for (index = 0; index < num_path_points; index++)
		{
			path[index].pose.x	   = new_plan[index].x;
			path[index].pose.y	   = new_plan[index].y;
			path[index].pose.theta = new_plan[index].theta;
			path[index].map = map_view->internal_map;
			carmen_verbose("%.1f %.1f\n", path[index].pose.x,
					path[index].pose.y);
		}
	}
	else
	{
		num_path_points = 0;
	}

	display_needs_updating = 1;
	do_redraw();
}


static GtkActionEntry action_entries[] = {
		{ "FileMenu",			NULL,			"_File",			   NULL,		 NULL, NULL						 },
		{ "ScreenShot",			GTK_STOCK_SAVE, "_Screen Shot",		   "<control>S", NULL, G_CALLBACK(save_image)	 },
		{ "StartFilming",		NULL,			"Start Filming",	   NULL,		 NULL, G_CALLBACK(start_filming) },
		{ "StopFilming",		NULL,			"Stop Filming",		   NULL,		 NULL, G_CALLBACK(start_filming) },
		{ "Quit",				GTK_STOCK_QUIT, "_Quit",			   "<control>Q", NULL, G_CALLBACK(gtk_main_quit) },
		{ "MapMenu",			NULL,			"_Maps",			   NULL,		 NULL, NULL						 },
		{ "SuperimposedMapMenu",NULL,			"_SuperimposedMaps",   NULL,		 NULL, NULL						 },
		{ "DisplayMenu",		NULL,			"_Display",			   NULL,		 NULL, NULL						 },
		{ "SimulatorMenu",		NULL,			"_Simulator",		   NULL,		 NULL, NULL						 },
		{ "SimAddPerson",		NULL,			"Add Person",		   NULL,		 NULL, G_CALLBACK(place_person)	 },
		{ "SimClearObjects",	NULL,			"Clear Objects",	   NULL,		 NULL, G_CALLBACK(clear_objects) },
		{ "StartLocationMenu",	NULL,			"_Start Location",	   NULL,		 NULL, NULL						 },
		{ "GlobalLocalization", NULL,			"Global Localization", NULL,		 NULL, G_CALLBACK(set_location)	 },
		{ "GoalMenu",			NULL,			"_Goals",			   NULL,		 NULL, NULL						 },
		{ "HelpMenu",			NULL,			"_Help",			   NULL,		 NULL, NULL						 },
		{ "HelpAbout",			NULL,			"_About",			   NULL,		 NULL, NULL						 }
};


static GtkToggleActionEntry toggle_entries[] = {
		{"TrackRobot",				NULL, "Track Robot",			   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"DrawPath",				NULL, "Draw Path",				   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"DrawWaypoints",			NULL, "Draw Waypoints",			   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"DrawRobotWaypoints",		NULL, "Draw Robot Waypoints",	   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowLateralOffset",		NULL, "Show Lateral Offset",	   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowParticles",			NULL, "Show Particles",			   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowFusedOdometry",		NULL, "Show Fused Odometry",	   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowGaussians",			NULL, "Show Gaussians",			   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowLaserData",			NULL, "Show Laser Data",		   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowMotionPath",		    NULL, "Show Motion Path",		   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowCommandPath",		    NULL, "Show Command Path",		   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"ShowDynamicObjects",		NULL, "Show Dynamic Objects",	   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"SimShowTruePosition",		NULL, "Show True Position",		   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE},
		{"SimShowObjects",			NULL, "Show Objects",			   NULL, NULL, G_CALLBACK(switch_localize_display), FALSE}
};


static GtkRadioActionEntry radio_map_entries[] = {
		{"Map",				NULL, "_Map",				"<control>M", NULL, CARMEN_NAVIGATOR_MAP_v	  },
		{"OfflineMap",		NULL, "_Offline Map",		NULL, 		  NULL, CARMEN_OFFLINE_MAP_v	  },
		{"Utility",			NULL, "_Utility",			NULL,		  NULL, CARMEN_NAVIGATOR_UTILITY_v},
		{"Costs",			NULL, "_Costs",				NULL,		  NULL, CARMEN_NAVIGATOR_COST_v	  },
		{"Likelihood",		NULL, "_Likelihood",		NULL,		  NULL, CARMEN_LOCALIZE_LMAP_v	  },
		{"GLikelihood", 	NULL, "_Global Likelihood", NULL,		  NULL, CARMEN_LOCALIZE_GMAP_v},
		{"Lane", 			NULL, "_Lane", 				NULL,		  NULL, CARMEN_LANE_MAP_v},
		{"CompleteMap",	NULL, "_Complete Map",		NULL,		  NULL, CARMEN_COMPLETE_MAP_v}
};

static GtkRadioActionEntry radio_superimposedmap_entries[] = {
		{"NoneSI",			NULL, "_None",				NULL,		  NULL, CARMEN_NONE_v	  },
		{"MapSI",			NULL, "_Map",				NULL,		  NULL, CARMEN_NAVIGATOR_MAP_v	  },
		{"OfflineMapSI",	NULL, "_Offline Map",		NULL, 		  NULL, CARMEN_OFFLINE_MAP_v	  },
		{"UtilitySI",		NULL, "_Utility",			NULL,		  NULL, CARMEN_NAVIGATOR_UTILITY_v},
		{"CostsSI",			NULL, "_Costs",				NULL,		  NULL, CARMEN_NAVIGATOR_COST_v	  },
		{"LikelihoodSI",	NULL, "_Likelihood",		NULL,		  NULL, CARMEN_LOCALIZE_LMAP_v	  },
		{"GLikelihoodSI",	NULL, "_Global Likelihood", NULL,		  NULL, CARMEN_LOCALIZE_GMAP_v},
		{"LaneSI", 			NULL, "_Lane", 				NULL,		  NULL, CARMEN_LANE_MAP_v}
};


char *ui_description =
		"<ui>"
		"  <menubar name='MainMenu'>"
		"    <menu action='FileMenu'>"
		"      <menuitem action='ScreenShot'/>"
		"      <separator/>"
		"      <menuitem action='StartFilming'/>"
		"      <menuitem action='StopFilming'/>"
		"      <separator/>"
		"      <menuitem action='Quit'/>"
		"    </menu>"
		"    <menu action='MapMenu'>"
		"      <menuitem action='Map'/>"
		"      <menuitem action='OfflineMap'/>"
		"      <menuitem action='Utility'/>"
		"      <menuitem action='Costs'/>"
		"      <menuitem action='Likelihood'/>"
		"      <menuitem action='GLikelihood'/>"
		"      <menuitem action='Lane'/>"
		"      <menuitem action='CompleteMap'/>"
		"    </menu>"
		"    <menu action='SuperimposedMapMenu'>"
		"      <menuitem action='NoneSI'/>"
		"      <menuitem action='MapSI'/>"
		"      <menuitem action='OfflineMapSI'/>"
		"      <menuitem action='UtilitySI'/>"
		"      <menuitem action='CostsSI'/>"
		"      <menuitem action='LikelihoodSI'/>"
		"      <menuitem action='GLikelihoodSI'/>"
		"      <menuitem action='LaneSI'/>"
		"    </menu>"
		"    <menu action='DisplayMenu'>"
		"      <menuitem action='TrackRobot'/>"
		"      <menuitem action='DrawPath'/>"
		"      <menuitem action='DrawWaypoints'/>"
		"      <menuitem action='DrawRobotWaypoints'/>"
		"      <menuitem action='ShowLateralOffset'/>"
		"      <menuitem action='ShowParticles'/>"
		"      <menuitem action='ShowFusedOdometry'/>"
		"      <menuitem action='ShowGaussians'/>"
		"      <menuitem action='ShowLaserData'/>"
		"      <menuitem action='ShowMotionPath'/>"
		"      <menuitem action='ShowCommandPath'/>"
		"      <menuitem action='ShowDynamicObjects'/>"
		"    </menu>"
		"    <menu action='SimulatorMenu'>"
		"      <menuitem action='SimShowTruePosition'/>"
		"      <menuitem action='SimShowObjects'/>"
		"      <separator/>"
		"      <menuitem action='SimAddPerson'/>"
		"      <menuitem action='SimClearObjects'/>"
		"    </menu>"
		"    <menu action='StartLocationMenu'>"
		"      <menuitem action='GlobalLocalization'/>"
		"    </menu>"
		"    <menu action='GoalMenu'>"
		"    </menu>"
		"    <menu action='HelpMenu'>"
		"      <menuitem action='HelpAbout'/>"
		"    </menu>"
		"  </menubar>"
		"</ui>";


static void switch_localize_display(GtkAction *action,
		gpointer   user_data __attribute__ ((unused)))
{
	char *name;
	GtkToggleAction *toggle;

	name = (char *)gtk_action_get_name(action);

	if (strcmp(name, "TrackRobot") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->track_robot = gtk_toggle_action_get_active(toggle);

		if (robot.map && nav_panel_config->track_robot)
		{
			carmen_map_graphics_adjust_scrollbars(map_view, &robot);
		}
	}
	else if (strcmp(name, "DrawPath") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->draw_path = gtk_toggle_action_get_active(toggle);
	}
	else if (strcmp(name, "DrawWaypoints") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->draw_waypoints = gtk_toggle_action_get_active(toggle);
	}
	else if (strcmp(name, "DrawRobotWaypoints") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->draw_robot_waypoints = gtk_toggle_action_get_active(toggle);
	}
	else if (strcmp(name, "ShowParticles") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_particles = gtk_toggle_action_get_active(toggle);

		if ((nav_panel_config->show_particles == 1) &&
				!nav_panel_config->show_gaussians)
		{
			carmen_localize_ackerman_subscribe_particle_correction_message(&particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
		}
		else if (!nav_panel_config->show_particles &&
				!nav_panel_config->show_gaussians)
		{
			carmen_localize_ackerman_subscribe_particle_correction_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
		}
	}
	else if (strcmp(name, "ShowFusedOdometry") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_fused_odometry = gtk_toggle_action_get_active(toggle);

	}
	else if (strcmp(name, "ShowGaussians") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_gaussians = gtk_toggle_action_get_active(toggle);
	}
	else if (strcmp(name, "ShowLaserData") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_lasers = gtk_toggle_action_get_active(toggle);

		if (nav_panel_config->show_lasers)
		{
			carmen_localize_ackerman_subscribe_sensor_message(&sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
		}
	}
	else if (strcmp(name, "ShowCommandPath") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_command_path = gtk_toggle_action_get_active(toggle);

		if (nav_panel_config->show_command_path)
		{
			carmen_obstacle_avoider_subscribe_path_message(NULL, (carmen_handler_t)obstacle_avoider_message_handler, CARMEN_SUBSCRIBE_LATEST);
			carmen_robot_ackerman_subscribe_road_velocity_control_message(NULL, (carmen_handler_t) road_velocity_control_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_obstacle_avoider_subscribe_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
			carmen_robot_ackerman_subscribe_road_velocity_control_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
		}
	}
	else if (strcmp(name, "ShowMotionPath") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_motion_path = gtk_toggle_action_get_active(toggle);

		if (nav_panel_config->show_motion_path)
		{
			carmen_obstacle_avoider_subscribe_motion_planner_path_message(NULL, (carmen_handler_t)motion_path_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_obstacle_avoider_subscribe_motion_planner_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
		}
	}
	else if (strcmp(name, "ShowDynamicObjects") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_dynamic_objects = gtk_toggle_action_get_active(toggle);

		if (nav_panel_config->show_dynamic_objects)
		{
			carmen_dynamic_object_detector_subscribe_clustered_objects_message(NULL, (carmen_handler_t) carmen_dynamic_object_detector_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_dynamic_object_detector_unsubscribe_clustered_objects_message(NULL);
		}
	}
	else if (strcmp(name, "SimShowTruePosition") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_true_pos = gtk_toggle_action_get_active(toggle);
	}
	else if (strcmp(name, "SimShowObjects") == 0)
	{
		toggle = GTK_TOGGLE_ACTION(action);
		nav_panel_config->show_simulator_objects =
				gtk_toggle_action_get_active(toggle);
	}
}

static gint film_image(gpointer data)
{
	return save_image(data, 0, NULL);
}

static void start_filming(GtkWidget *w __attribute__ ((unused)),
		int		 arg __attribute__ ((unused)))
{
	GtkWidget *menu_item;

	if (is_filming)
	{
		menu_item =
				gtk_ui_manager_get_widget(ui_manager,
						"/ui/MainMenu/FileMenu/StopFilming");
		gtk_widget_hide(menu_item);
		menu_item =
				gtk_ui_manager_get_widget(ui_manager,
						"/ui/MainMenu/FileMenu/StartFilming");
		gtk_widget_show(menu_item);
		gtk_timeout_remove(filming_timeout);
		is_filming = 0;
	}
	else
	{
		menu_item =
				gtk_ui_manager_get_widget(ui_manager,
						"/ui/MainMenu/FileMenu/StartFilming");
		gtk_widget_hide(menu_item);
		menu_item =
				gtk_ui_manager_get_widget(ui_manager,
						"/ui/MainMenu/FileMenu/StopFilming");
		gtk_widget_show(menu_item);
		is_filming = 1;
		filming_timeout = gtk_timeout_add
				(1000, (GtkFunction)film_image, NULL);
	}
}

static void assign_colour(GdkColor *colour, int new_colour)
{
	colour->pixel = new_colour;
	colour->blue  = new_colour & 0xff;
	new_colour	>>= 8;
	colour->green = new_colour & 0xff;
	new_colour	>>= 8;
	colour->red	  = new_colour & 0xff;
	new_colour	>>= 8;
}

static void assign_variable(char *action_name, int value, int default_value)
{
	GtkAction *action;
	int state;

	if (value > 2)
	{
		return;
	}

	action = gtk_ui_manager_get_action(ui_manager, action_name);
	state  = gtk_toggle_action_get_active(GTK_TOGGLE_ACTION(action));

	if (value == -1)
	{
		value = default_value;
	}

	if (state != value)
	{
		gtk_toggle_action_set_active(GTK_TOGGLE_ACTION(action), value);
	}
}

void navigator_graphics_reset(void)
{
	robot_colour  = DEFAULT_ROBOT_COLOUR;
	goal_colour	  = DEFAULT_GOAL_COLOUR;
	path_colour	  = DEFAULT_PATH_COLOUR;
	tree_colour	  = DEFAULT_TREE_COLOUR;
	people_colour = DEFAULT_PEOPLE_COLOUR;

	assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot", -1,
			DEFAULT_TRACK_ROBOT);
	assign_variable("/ui/MainMenu/DisplayMenu/DrawPath", -1,
			DEFAULT_DRAW_PATH);
	assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints", -1,
			DEFAULT_DRAW_WAYPOINTS);
	assign_variable("/ui/MainMenu/DisplayMenu/DrawRobotWaypoints", -1,
			DEFAULT_DRAW_ROBOT_WAYPOINTS);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowLateralOffset", -1,
			DEFAULT_SHOW_LATERAL_OFFSET);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles", -1,
			DEFAULT_SHOW_PARTICLES);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowFusedOdometry", -1,
			DEFAULT_SHOW_FUSED_ODOMETRY);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians", -1,
			DEFAULT_SHOW_GAUSSIANS);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData", -1,
			DEFAULT_SHOW_LASER);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowCommandPath", -1,
			DEFAULT_SHOW_COMMAND_PATH);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowMotionPath", -1,
			DEFAULT_SHOW_MOTION_PATH);
	assign_variable("/ui/MainMenu/DisplayMenu/ShowDynamicObjects", -1,
			DEFAULT_SHOW_DYNAMIC_OBJECTS);
	assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition", -1,
			DEFAULT_SHOW_SIMULATOR);
	assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects", -1,
			DEFAULT_SHOW_TRACKED_OBJECTS);
}

void navigator_graphics_display_config
(char *attribute, int value, char *new_status_message __attribute__ ((unused)))
{
	if (strncmp(attribute, "robot colour", 12) == 0)
	{
		if (value == -1)
		{
			robot_colour = DEFAULT_ROBOT_COLOUR;
		}
		else
		{
			assign_colour(&robot_colour, value);
		}
	}
	else if (strncmp(attribute, "goal colour", 11) == 0)
	{
		if (value == -1)
		{
			goal_colour = DEFAULT_GOAL_COLOUR;
		}
		else
		{
			assign_colour(&goal_colour, value);
		}
	}
	else if (strncmp(attribute, "goal colour", 11) == 0)
	{
		if (value == -1)
		{
			tree_colour = DEFAULT_TREE_COLOUR;
		}
		else
		{
			assign_colour(&tree_colour, value);
		}
	}
	else if (strncmp(attribute, "path colour", 11) == 0)
	{
		if (value == -1)
		{
			path_colour = DEFAULT_PATH_COLOUR;
		}
		else
		{
			assign_colour(&path_colour, value);
		}
	}
	else if (strncmp(attribute, "people colour", 11) == 0)
	{
		if (value == -1)
		{
			path_colour = DEFAULT_PATH_COLOUR;
		}
		else
		{
			assign_colour(&people_colour, value);
		}
	}
	else if (strncmp(attribute, "track robot", 11) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot",
				value, DEFAULT_TRACK_ROBOT);
	}
	else if (strncmp(attribute, "draw path", 20) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/DrawPath",
				value, DEFAULT_DRAW_PATH);
	}
	else if (strncmp(attribute, "draw waypoints", 14) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints",
				value, DEFAULT_DRAW_WAYPOINTS);
	}
	else if (strncmp(attribute, "draw robot waypoints", 14) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/DrawRobotWaypoints",
				value, DEFAULT_DRAW_ROBOT_WAYPOINTS);
	}
	else if (strncmp(attribute, "show particles", 14) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles",
				value, DEFAULT_SHOW_PARTICLES);
	}
	else if (strncmp(attribute, "show fused odometry", 19) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowFusedOdometry",
				value, DEFAULT_SHOW_FUSED_ODOMETRY);
	}
	else if (strncmp(attribute, "show gaussians", 14) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians",
				value, DEFAULT_SHOW_GAUSSIANS);
	}
	else if (strncmp(attribute, "show laser", 10) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData",
				value, DEFAULT_SHOW_LASER);
	}
	else if (strncmp(attribute, "Show Command Path", 17) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowCommandPath",
				value, DEFAULT_SHOW_COMMAND_PATH);
	}
	else if (strncmp(attribute, "Show Command Path", 50) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowMotionPath",
				value, DEFAULT_SHOW_MOTION_PATH);
	}
	else if (strncmp(attribute, "Show Dynamic Objects", 20) == 0)
	{
		assign_variable("/ui/MainMenu/DisplayMenu/ShowDynamicObjects",
				value, DEFAULT_SHOW_DYNAMIC_OBJECTS);
	}
	else if (strncmp(attribute, "show simulator", 14) == 0)
	{
		assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition",
				value, DEFAULT_SHOW_SIMULATOR);
	}
	else if (strncmp(attribute, "show tracked objects", 20) == 0)
	{
		assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects",
				value, DEFAULT_SHOW_TRACKED_OBJECTS);
	}

	carmen_map_graphics_redraw(map_view);
}

void
navigator_graphics_redraw_superimposed()
{
	carmen_map_graphics_redraw_superimposed(map_view);
}

static void
switch_superimposedmap_display(GtkAction *action, gpointer user_data
		__attribute__ ((unused)))
{
	carmen_navigator_map_t new_display;

	new_display = gtk_radio_action_get_current_value(GTK_RADIO_ACTION(action));

	navigator_get_map(new_display, 1);

	carmen_map_graphics_redraw_superimposed(map_view);
}

static void
switch_map_display(GtkAction *action, gpointer user_data
		__attribute__ ((unused)))
{
	carmen_navigator_map_t new_display;

	new_display = gtk_radio_action_get_current_value(GTK_RADIO_ACTION(action));

	navigator_get_map(new_display, 0);
}

static GtkWidget *get_main_menu(void)
{
	GtkWidget *menubar;
	GtkActionGroup *action_group;
	GtkAccelGroup  *accel_group;
	GError	  *error;
	GtkAction *action;

	action_group = gtk_action_group_new("MenuActions");
	gtk_action_group_add_actions(action_group, action_entries,
			G_N_ELEMENTS(action_entries), window);
	gtk_action_group_add_toggle_actions(action_group, toggle_entries,
			G_N_ELEMENTS(toggle_entries), window);

	gtk_action_group_add_radio_actions(action_group, radio_map_entries,
			G_N_ELEMENTS(radio_map_entries),
			CARMEN_NAVIGATOR_MAP_v,
			G_CALLBACK(switch_map_display), NULL);

	gtk_action_group_add_radio_actions(action_group, radio_superimposedmap_entries,
			G_N_ELEMENTS(radio_superimposedmap_entries),
			CARMEN_NONE_v,
			G_CALLBACK(switch_superimposedmap_display), NULL);

	ui_manager = gtk_ui_manager_new();
	gtk_ui_manager_insert_action_group(ui_manager, action_group, 0);

	accel_group = gtk_ui_manager_get_accel_group(ui_manager);
	gtk_window_add_accel_group(GTK_WINDOW(window), accel_group);

	error = NULL;

	if (!gtk_ui_manager_add_ui_from_string(ui_manager, ui_description, -1,
			&error))
	{
		g_message("building menus failed: %s", error->message);
		g_error_free(error);
		exit(EXIT_FAILURE);
	}

	menubar = gtk_ui_manager_get_widget(ui_manager, "/MainMenu");

	action = gtk_action_group_get_action(action_group, "TrackRobot");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->track_robot);

	action = gtk_action_group_get_action(action_group, "DrawPath");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->draw_path);

	action = gtk_action_group_get_action(action_group, "DrawWaypoints");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->draw_waypoints);

	action = gtk_action_group_get_action(action_group, "DrawRobotWaypoints");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->draw_robot_waypoints);

	action = gtk_action_group_get_action(action_group, "ShowParticles");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_particles);

	action = gtk_action_group_get_action(action_group, "ShowFusedOdometry");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_fused_odometry);

	action = gtk_action_group_get_action(action_group, "ShowGaussians");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_gaussians);

	action = gtk_action_group_get_action(action_group, "ShowLaserData");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_lasers);

	action = gtk_action_group_get_action(action_group, "ShowCommandPath");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_command_path);

	action = gtk_action_group_get_action(action_group, "ShowMotionPath");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_motion_path);

	action = gtk_action_group_get_action(action_group, "ShowDynamicObjects");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_dynamic_objects);

	action = gtk_action_group_get_action(action_group, "SimShowTruePosition");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_true_pos);

	action = gtk_action_group_get_action(action_group, "SimShowObjects");
	gtk_toggle_action_set_active
	(GTK_TOGGLE_ACTION(action), nav_panel_config->show_simulator_objects);

	if (nav_panel_config->show_particles || nav_panel_config->show_gaussians)
	{
		carmen_localize_ackerman_subscribe_particle_correction_message(&particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}

	if (nav_panel_config->show_lasers)
	{
		carmen_localize_ackerman_subscribe_sensor_message(&sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}

	return menubar;
}

static GtkWidget *new_label(char *s, GtkWidget *box)
{
	GtkWidget *the_new_label;

	the_new_label = gtk_label_new(s);
	gtk_box_pack_start(GTK_BOX(box), the_new_label, FALSE, FALSE, 0);

	return the_new_label;
}

static GtkWidget *construct_status_frame(GtkWidget *parent)
{
	GtkWidget *status_frame;
	GtkWidget *status_box;
	char *font_type = "Verdana 10";

	status_frame = gtk_frame_new("Status");
	gtk_container_set_border_width(GTK_CONTAINER(status_frame), 2);
	gtk_container_add(GTK_CONTAINER(parent), status_frame);

	status_box = gtk_vbox_new(FALSE, 15);
	gtk_container_set_border_width(GTK_CONTAINER(status_box), 2);

	map_status_label = new_label("No map", status_box);
	gtk_widget_modify_font (map_status_label,	pango_font_description_from_string (font_type));

	map_origin_label = new_label("Origin: (0, 0)", status_box);
	gtk_widget_modify_font (map_origin_label,	pango_font_description_from_string (font_type));

	robot_status_label	= new_label("Robot position: 0 0", status_box);
	gtk_widget_modify_font (robot_status_label,	pango_font_description_from_string (font_type));

	fused_odometry_status_label = new_label("Fused Pos: 0 0", status_box);
	gtk_widget_modify_font (fused_odometry_status_label,	pango_font_description_from_string (font_type));

	robot_speed_label	= new_label("Velocity: 0 km/h (0 m/s) 0 rad/s", status_box);
	gtk_widget_modify_font (robot_speed_label,	pango_font_description_from_string (font_type));

	goal_status_label	= new_label("Goal position: 0 0", status_box);
	gtk_widget_modify_font (goal_status_label,	pango_font_description_from_string (font_type));

	cursor_status_label = new_label("Grid Cell:", status_box);
	gtk_widget_modify_font (cursor_status_label,	pango_font_description_from_string (font_type));

	value_label = new_label("Value: 0.0", status_box);
	gtk_widget_modify_font (value_label,	pango_font_description_from_string (font_type));

	gtk_container_add(GTK_CONTAINER(status_frame), status_box);

	return status_box;
}

void clear_point_checked()
{
	pointers *point = queuePoints->begin;
	queuePoints->begin = point->next;
	free(point);
}

void update_point(pointers *reached)
{
	queuePoints->curr = reached->next;
	navigator_set_goal(queuePoints->curr->point.pose.x, queuePoints->curr->point.pose.y, queuePoints->curr->point.pose.theta);
	clear_point_checked();
	navigator_start_moving();
}

static int received_robot_pose(void)
{
	if (!behavior_selector_active)
		if ((queuePoints != NULL) && (queuePoints->begin != NULL) && (GTK_TOGGLE_BUTTON(autonomous_button)->active))
		{
			pointers *reached = queuePoints->curr;

			if ((reached != NULL) && (reached->next != NULL))
			{
				if (carmen_distance(&queuePoints->curr->point.pose, &robot.pose) < 0.5) // 0.5 m // @@@ Alberto: Isso deveria ser controlado pelo navigator, nao pela interface
				{
					update_point(reached);
				}
			}
		}

	return (robot.map != NULL);
}


static void
draw_particles(GtkMapViewer *the_map_view, double pixel_size)
{
	int index;
	carmen_world_point_t particle, final_point;

	if (!nav_panel_config->show_particles)
	{
		return;
	}

	if (particle_msg.particles != NULL)
	{
		for (index = 0; index < particle_msg.num_particles; index++)
		{
			particle.pose.x = particle_msg.particles[index].x;
			particle.pose.y = particle_msg.particles[index].y;
			particle.pose.theta = particle_msg.particles[index].theta;
			particle.map	= the_map_view->internal_map;

			final_point = particle;
			final_point.pose.x = final_point.pose.x +
					cos(final_point.pose.theta) * 0.2;
			final_point.pose.y = final_point.pose.y +
					sin(final_point.pose.theta) * 0.2;

			carmen_map_graphics_draw_line(the_map_view, &carmen_black,
					&particle, &final_point);

			carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE,
					&particle, pixel_size);
		}
	}

}

static void draw_gaussians(GtkMapViewer *the_map_view)
{
	carmen_world_point_t mean;

	if (!nav_panel_config->show_gaussians || (particle_msg.particles == NULL))
	{
		return;
	}

	mean = robot;
	mean.pose.x		= globalpos->globalpos.x;
	mean.pose.y		= globalpos->globalpos.y;
	mean.pose.theta = globalpos->globalpos.theta;

	carmen_map_graphics_draw_ellipse
	(the_map_view, &carmen_black, &mean,
			carmen_square(globalpos->globalpos_std.x),
			globalpos->globalpos_xy_cov,
			carmen_square(globalpos->globalpos_std.y), 4);
}

static void draw_lasers(GtkMapViewer *the_map_view, double pixel_size)
{
	double dot_size;
	int	   index;
	carmen_world_point_t particle;
	double angle;

	dot_size = 3 * pixel_size;

	if (!nav_panel_config->show_lasers)
	{
		return;
	}

	particle = robot;

	for (index = 0; index < sensor_msg.num_readings;
			index += sensor_msg.laser_skip)
	{
		// finale: doesn't this assume a 180 fov?
		// angle = sensor_msg.pose.theta - M_PI_2 +
		//  index / (float)(sensor_msg.num_readings - 1) * M_PI;
		angle = sensor_msg.pose.theta - sensor_msg.config.fov / 2 +
				index / (double)(sensor_msg.num_readings - 1) * sensor_msg.config.fov;


		particle.pose.x = sensor_msg.pose.x + sensor_msg.range[index] *
				cos(angle);
		particle.pose.y = sensor_msg.pose.y + sensor_msg.range[index] *
				sin(angle);

		if (sensor_msg.mask[index])
		{
			carmen_map_graphics_draw_line(the_map_view, &carmen_green, &robot, &particle);
			carmen_map_graphics_draw_circle(the_map_view,
					&carmen_green, TRUE,
					&particle, dot_size);
		}
		else
		{
			carmen_map_graphics_draw_line(the_map_view, &carmen_yellow, &robot, &particle);
			carmen_map_graphics_draw_circle(the_map_view,
					&carmen_yellow, TRUE,
					&particle, dot_size);
		}
	}

#ifdef blah

	/* rear laser */
	for (index = 0; index < sensor_msg.rear_laser_scan.num_readings;
			index++)
	{
		colour = sensor_msg.rear_laser_scan.scan[index].mean_prob *
				(GRADIENT_COLORS - 1);

		particle.pose.x = sensor_msg.rear_laser_scan.scan[index].mean_x;
		particle.pose.y = sensor_msg.rear_laser_scan.scan[index].mean_y;
		carmen_map_graphics_draw_circle(the_map_view,
				&RedBlueGradient[colour], TRUE,
				&particle, dot_size);
	}

#endif
}

static double x_coord(double x, double y, carmen_world_point_t *offset)
{
	return x * cos(offset->pose.theta) - y *sin(offset->pose.theta) + offset->pose.x;
}

static double y_coord(double x, double y, carmen_world_point_t *offset)
{
	return x * sin(offset->pose.theta) + y *cos(offset->pose.theta) + offset->pose.y;
}

static void draw_differential_shape(GtkMapViewer *the_map_view,
		carmen_world_point_t *location, int filled,
		GdkColor *colour)
{
	double robot_radius;
	robot_radius = robot_config->width / 2.0;

	carmen_map_graphics_draw_circle(the_map_view, colour, filled,
			location, robot_radius);
}

static void draw_ackerman_shape(GtkMapViewer *the_map_view,
		carmen_world_point_t *location, int filled,
		GdkColor *colour)
{
	carmen_world_point_t wp[4];
	double width2, length, dist_rear_car_rear_wheels;

	dist_rear_car_rear_wheels = robot_config->distance_between_rear_car_and_rear_wheels;
	width2 = robot_config->width / 2;
	length = robot_config->length;

	wp[0].pose.x = x_coord(-dist_rear_car_rear_wheels, width2, location);
	wp[0].pose.y = y_coord(-dist_rear_car_rear_wheels, width2, location);
	wp[1].pose.x = x_coord(-dist_rear_car_rear_wheels, -width2, location);
	wp[1].pose.y = y_coord(-dist_rear_car_rear_wheels, -width2, location);
	wp[2].pose.x = x_coord(length - dist_rear_car_rear_wheels, -width2, location);
	wp[2].pose.y = y_coord(length - dist_rear_car_rear_wheels, -width2, location);
	wp[3].pose.x = x_coord(length - dist_rear_car_rear_wheels, width2, location);
	wp[3].pose.y = y_coord(length - dist_rear_car_rear_wheels, width2, location);

	wp[0].map = wp[1].map = wp[2].map = wp[3].map = location->map;

	carmen_map_graphics_draw_polygon(the_map_view, colour, wp, 4, filled);
}

static void draw_robot_shape(GtkMapViewer *the_map_view,
		carmen_world_point_t *location, int filled,
		GdkColor *colour)
{
	if (!robot_config->rectangular)
	{
		draw_differential_shape(the_map_view, location, filled, colour);
	}
	else
	{
		draw_ackerman_shape(the_map_view, location, filled, colour);
	}
}

static void draw_differential_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
{
	carmen_world_point_t initial_point, radius;
	initial_point = *robot_pose;

	radius = initial_point;
	radius.pose.x = radius.pose.x +
			cos(radius.pose.theta) * robot_config->width / 2.0;
	radius.pose.y = radius.pose.y +
			sin(radius.pose.theta) * robot_config->width / 2.0;

	carmen_map_graphics_draw_line(the_map_view, &carmen_black,
			&initial_point, &radius);
}

static void draw_ackerman_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
{
	carmen_world_point_t wp[2];

	wp[0].map = wp[1].map = robot_pose->map;

	wp[0] = *robot_pose;
	wp[0].pose.x = x_coord(robot_config->length / 2.0, 0, robot_pose);
	wp[0].pose.y = y_coord(robot_config->length / 2.0, 0, robot_pose);
	wp[1] = wp[0];
	wp[1].pose.x = wp[1].pose.x + cos(wp[1].pose.theta) * robot_config->length / 2.0;
	wp[1].pose.y = wp[1].pose.y + sin(wp[1].pose.theta) * robot_config->length / 2.0;

	carmen_map_graphics_draw_line(the_map_view, &carmen_black, &wp[0], &wp[1]);
}

static void draw_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
{
	if (!robot_config->rectangular)
	{
		draw_differential_orientation_mark(the_map_view, robot_pose);
	}
	else
	{
		draw_ackerman_orientation_mark(the_map_view, robot_pose);
	}
}

static void draw_fused_odometry_pose(GtkMapViewer *the_map_view)
{
	draw_robot_shape(the_map_view, &fused_odometry_position, TRUE, &carmen_green);
	draw_robot_shape(the_map_view, &fused_odometry_position, FALSE, &carmen_black);

	draw_orientation_mark(the_map_view, &fused_odometry_position);
}


static void draw_simulated_robot(GtkMapViewer *the_map_view)
{
	if (!nav_panel_config->show_true_pos || (simulator_trueposition.map == NULL))
	{
		return;
	}

	draw_robot_shape(the_map_view, &simulator_trueposition, TRUE, &carmen_blue);
	draw_robot_shape(the_map_view, &simulator_trueposition, FALSE, &carmen_black);

	draw_orientation_mark(the_map_view, &simulator_trueposition);
}

static void draw_simulated_objects(GtkMapViewer *the_map_view)
{
	int index;
	carmen_world_point_t particle;
	carmen_simulator_ackerman_objects_t *simulator_object;
	double circle_size;

	if (nav_panel_config->show_simulator_objects)
	{
		circle_size = robot_config->width / 2.0;

		particle.map = the_map_view->internal_map;

		if (simulator_objects)
		{
			for (index = 0; index < simulator_objects->length; index++)
			{
				simulator_object = (carmen_simulator_ackerman_objects_t *) carmen_list_get(simulator_objects, index);
				particle.pose.x	 = simulator_object->x;
				particle.pose.y	 = simulator_object->y;
				carmen_map_graphics_draw_circle(the_map_view, &carmen_orange, TRUE,
						&particle, circle_size);
				carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
						&particle, circle_size);
			}
		}
	}
}

static void draw_robot(GtkMapViewer *the_map_view)
{
	if (!nav_panel_config->show_particles && !nav_panel_config->show_gaussians)
	{
		draw_robot_shape(the_map_view, &robot, TRUE, &robot_colour);
	}

	if (!nav_panel_config->show_gaussians)
	{
		draw_robot_shape(the_map_view, &robot, FALSE, &carmen_black);
	}

	draw_orientation_mark(the_map_view, &robot);
}


static void draw_goal_list(GtkMapViewer	*the_map_view,
		carmen_world_point_t goal)
{
	carmen_world_point_t new_goal;
	int i;

	//draw current navigator goal
	if ((goal.pose.x > 0) && (goal.pose.y > 0) && (goal.map != NULL))
	{
		draw_robot_shape(the_map_view, &goal, TRUE, &goal_colour);
		draw_robot_shape(the_map_view, &goal, FALSE, &carmen_black);
		draw_orientation_mark(the_map_view, &goal);
	}

	//draw final navigator goal
	if (final_goal.map != NULL)
	{
		draw_robot_shape(the_map_view, &final_goal, TRUE, &carmen_red);
		draw_robot_shape(the_map_view, &final_goal, FALSE, &carmen_black);
		draw_orientation_mark(the_map_view, &final_goal);
	}

	//draw goal list set by the interface
	if (!behavior_selector_active)
		if ((queuePoints != NULL) && (queuePoints->begin != NULL))
		{
			pointers *lista;

			lista = queuePoints->begin;

			if (lista->next == NULL)
			{
				new_goal = goal;
				new_goal.pose.x		= lista->point.pose.x;
				new_goal.pose.y		= lista->point.pose.y;
				new_goal.pose.theta = lista->point.pose.theta;

				draw_robot_shape(the_map_view, &new_goal, TRUE, &goal_colour);
				draw_robot_shape(the_map_view, &new_goal, FALSE, &carmen_black);
				draw_orientation_mark(the_map_view, &new_goal);
			}
			else
			{
				while (lista != NULL)
				{
					new_goal = goal;
					new_goal.pose.x		= lista->point.pose.x;
					new_goal.pose.y		= lista->point.pose.y;
					new_goal.pose.theta = lista->point.pose.theta;

					draw_robot_shape(the_map_view, &new_goal, TRUE, &goal_colour);
					draw_robot_shape(the_map_view, &new_goal, FALSE, &carmen_black);
					draw_orientation_mark(the_map_view, &new_goal);


					lista = lista->next;
				}
			}
		}

	//draw navigator goal list
	for(i = 0; i < goal_list_size; i++)
	{
		draw_robot_shape(the_map_view, &navigator_goal_list[i], TRUE, &goal_colour);
		draw_robot_shape(the_map_view, &navigator_goal_list[i], FALSE, &carmen_black);
		draw_orientation_mark(the_map_view, &navigator_goal_list[i]);
	}

	//draw rddf waypoint list
	for(i = 0; i < waypoint_list_size; i++)
	{
		draw_robot_shape(the_map_view, &navigator_waypoint_list[i], TRUE, &carmen_green);
		draw_robot_shape(the_map_view, &navigator_waypoint_list[i], FALSE, &carmen_black);
		draw_orientation_mark(the_map_view, &navigator_waypoint_list[i]);
	}
}


void
print_pose(carmen_world_point_p pose, char *prefix)
{
	printf("%s %lf, %lf\n", prefix, pose->pose.x, pose->pose.y);
}

void
convert_map_point_to_world_point(carmen_map_point_p carmen_map_point,
		carmen_world_point_p world_point)
{
	double x = carmen_map_point->x * carmen_map_point->map->config.resolution;
	double y = carmen_map_point->y * carmen_map_point->map->config.resolution;

	world_point->pose.x = x;
	world_point->pose.y = y;
	world_point->pose.theta = 0;
	world_point->map = carmen_map_point->map;
}

static void
draw_dynamic_objects(GtkMapViewer *the_map_view)
{
	int i, x, y;
	//	carmen_map_point_t map_point;
	//	carmen_world_point_t world_point;

	if(dynamic_object_detector_clustered_objects_msg != NULL)
	{
		for(i = 0; i < dynamic_object_detector_clustered_objects_msg->map_size; i++)
		{
			int object_value = (int) dynamic_object_detector_clustered_objects_msg->objects_map[i];

			if(object_value == 1)
			{
				x = i % dynamic_object_detector_clustered_objects_msg->config.x_size;
				y = dynamic_object_detector_clustered_objects_msg->config.y_size - (i / dynamic_object_detector_clustered_objects_msg->config.x_size);

				gdk_gc_set_foreground(the_map_view->drawing_gc, &carmen_red);
				gdk_draw_point(the_map_view->drawing_pixmap, the_map_view->drawing_gc,
						x, y);
			}
		}
	}
}


static void
draw_road_velocity_control(GtkMapViewer *the_map_view)
{
	carmen_world_point_t start_point, end_point;

	if (the_map_view->internal_map == NULL)
		return;

	start_point.map = the_map_view->internal_map;
	end_point.map = the_map_view->internal_map;

	start_point.pose.x = road_velocity_control.left_near_obstacle.x;
	start_point.pose.y = road_velocity_control.left_near_obstacle.y;
	start_point.pose.theta = road_velocity_control.left_near_obstacle.theta;

	end_point.pose.x = road_velocity_control.right_near_obstacle.x;
	end_point.pose.y = road_velocity_control.right_near_obstacle.y;
	end_point.pose.theta = road_velocity_control.right_near_obstacle.theta;

	carmen_map_graphics_draw_line(the_map_view, &carmen_purple, &start_point, &end_point);
}

static void
draw_path_color(GtkMapViewer *the_map_view, carmen_world_point_t* path, int size, GdkColor *color)
{
	carmen_world_point_t path_x_1, path_x_2;
	int index;

	if (path == NULL || path->map == NULL)
		return;

	for (index = 1; index < size; index++)
	{
		carmen_map_graphics_draw_line(the_map_view, color, path + index - 1, path + index);

		if (nav_panel_config->draw_robot_waypoints)
		{
			draw_robot_shape(the_map_view, (path + index), FALSE, color);
			draw_orientation_mark(the_map_view, (path + index));
		}

		if (nav_panel_config->draw_waypoints)
		{
			path_x_1 = *(path + index);
			path_x_2 = *(path + index);

			path_x_1.pose.x -= path->map->config.resolution;
			path_x_1.pose.y -= path->map->config.resolution;
			path_x_2.pose.x += path->map->config.resolution;
			path_x_2.pose.y += path->map->config.resolution;
			carmen_map_graphics_draw_line(the_map_view, color, &path_x_1,
					&path_x_2);

			path_x_1.pose.y += path->map->config.resolution * 2;
			path_x_2.pose.y -= path->map->config.resolution * 2;
			carmen_map_graphics_draw_line(the_map_view, color, &path_x_1,
					&path_x_2);
		}
	}
}

static void
draw_path_vector(GtkMapViewer *the_map_view)
{
	int i;
	for (i = 0; i < PATH_VECTOR_SIZE; i++)
	{
		if (path_vector[i])
		{
			draw_path_color(the_map_view, path_vector[i], path_vector_size[i], path_vector_color[i]);
		}
	}
}

static void
draw_path(carmen_world_point_t *path, int num_path_points, GdkColor path_colour, GdkColor robot_color, GtkMapViewer *the_map_view)
{
	int index;
	carmen_world_point_t path_x_1, path_x_2;

	if (path == NULL)
		return;
	else if (path->map == NULL)
		return;


	//print_pose(path, "@@@");
	for (index = 1; index < num_path_points; index++)
	{
		//print_pose(path + index, "@");
		carmen_map_graphics_draw_line(the_map_view, &path_colour, path + index - 1, path + index);

		if (nav_panel_config->draw_robot_waypoints)
		{
			draw_robot_shape(the_map_view, (path + index), FALSE, &robot_color);
			draw_orientation_mark(the_map_view, (path + index));
		}

		if (nav_panel_config->draw_waypoints)
		{
			path_x_1 = *(path + index);
			path_x_2 = *(path + index);

			path_x_1.pose.x -= path->map->config.resolution;
			path_x_1.pose.y -= path->map->config.resolution;
			path_x_2.pose.x += path->map->config.resolution;
			path_x_2.pose.y += path->map->config.resolution;
			carmen_map_graphics_draw_line(the_map_view, &path_colour, &path_x_1,
					&path_x_2);

			path_x_1.pose.y += path->map->config.resolution * 2;
			path_x_2.pose.y -= path->map->config.resolution * 2;
			carmen_map_graphics_draw_line(the_map_view, &path_colour, &path_x_1,
					&path_x_2);
		}
	}
}


static void
draw_plan_tree(GtkMapViewer *the_map_view, double pixel_size)
{
	int index;
	//carmen_world_point_t path_x_1, path_x_2;

	for (index = 1; index < num_plan_tree_points; index++)
	{
		if (plan_tree_p1->map == NULL)
		{
			break;
		}

		if (plan_tree_p2->map == NULL)
		{
			break;
		}

		//carmen_map_graphics_draw_line(the_map_view, &tree_colour, plan_tree_p1 + index, plan_tree_p2 + index);
		//		time_of_simulator_update = carmen_get_time();

		carmen_map_graphics_draw_line(the_map_view, &carmen_yellow, &plan_tree_p1[index], &plan_tree_p2[index]);
		carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE, &plan_tree_p1[index], pixel_size);
		carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE, &plan_tree_p2[index], pixel_size);
	}
}


static void
draw_placing_animation(GtkMapViewer *the_map_view)
{
	GdkColor *colour = &carmen_black;
	carmen_world_point_t *draw_point = NULL;

	if ((placement_status != ORIENTING_ROBOT) &&
			(placement_status != ORIENTING_GOAL) &&
			(placement_status != ORIENTING_PERSON) &&
			(placement_status != ORIENTING_FINAL_GOAL) &&
			(placement_status != ORIENTING_SIMULATOR))
	{
		return;
	}

	/* Everything from here down is only used if we are orienting something.
	     We have to draw the object itself (the robot, person, whatever) since
	     in some cases the display hasn't actually published the fact that the
	     feature has changed.
	 */

	if (placement_status == ORIENTING_ROBOT)
	{
		draw_point = &robot_temp;
		colour = &carmen_red;
	}
	else if (placement_status == ORIENTING_PERSON)
	{
		draw_point = &new_person;
		colour	   = &carmen_orange;
	}
	else if (placement_status == ORIENTING_SIMULATOR)
	{
		draw_point = &new_simulator;
		colour	   = &carmen_blue;
	}
	else if (placement_status == ORIENTING_GOAL)
	{
		draw_point = &goal_temp;
		colour	   = &carmen_yellow;
	}
	else if (placement_status == ORIENTING_FINAL_GOAL)
	{
		draw_point = &final_goal;
		colour	   = &carmen_red;
	}

	draw_point->pose.theta = atan2(cursor_pos.pose.y - draw_point->pose.y, cursor_pos.pose.x - draw_point->pose.x);

	if (1) // @@@ Alberto: o teste era if (placement_status != ORIENTING_PERSON), mas as pessoas ficavam gigantes no momento da colocacao...
	{
		draw_robot_shape(the_map_view, draw_point, TRUE, colour);
		draw_robot_shape(the_map_view, draw_point, FALSE, &carmen_black);
		carmen_map_graphics_draw_line(the_map_view, colour, draw_point,
				&cursor_pos);
		draw_orientation_mark(the_map_view, draw_point);
	}
	else
	{
		carmen_map_graphics_draw_circle(the_map_view, colour, TRUE,
				draw_point, 1);
		carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE,
				draw_point, 1);
		carmen_map_graphics_draw_line(the_map_view, colour, draw_point,
				&cursor_pos);
	}
}

void draw_robot_objects(GtkMapViewer *the_map_view)
{
	//  int colour;
	double pixel_size;

	if (the_map_view->internal_map == NULL)
	{
		return;
	}

	pixel_size = 1 / map_view->rescale_size *
			map_view->internal_map->config.resolution;

	if (nav_panel_config->show_fused_odometry)
		draw_fused_odometry_pose(the_map_view);

	/*
	 * Draw robot features
	 */
	if (received_robot_pose())
	{
		if(nav_panel_config->show_particles)
			draw_particles(the_map_view, pixel_size);

		if(nav_panel_config->show_gaussians)
			draw_gaussians(the_map_view);

		if(nav_panel_config->show_lasers)
			draw_lasers(the_map_view, pixel_size);

		draw_robot(the_map_view);
	}

	draw_plan_tree(the_map_view, pixel_size);

	draw_goal_list(the_map_view, goal);

	if(nav_panel_config->show_true_pos)
		draw_simulated_robot(the_map_view);

	if(nav_panel_config->show_simulator_objects)
		draw_simulated_objects(the_map_view);

	//do some animation when the user is placing something (like robot or goal)
	draw_placing_animation(the_map_view);

	if (nav_panel_config->show_motion_path)
	{
		draw_path(motion_path, motion_path_size, carmen_green, carmen_green, the_map_view);
	}

	if (nav_panel_config->show_command_path)
	{
		draw_path(obstacle_avoider_path, obstacle_avoider_path_size, carmen_red, carmen_red, the_map_view);
		draw_road_velocity_control(the_map_view);
	}

	if (nav_panel_config->draw_path)
		draw_path(path, num_path_points, path_colour, carmen_black, the_map_view);

	if (nav_panel_config->show_dynamic_objects)
	{
		draw_dynamic_objects(the_map_view);
	}

	draw_path_vector(the_map_view);
}

void world_point_to_global_world_point(carmen_world_point_t *world_point)
{
	if (!world_point || !world_point->map)
	{
		return;
	}

	world_point->pose.x += world_point->map->config.x_origin;
	world_point->pose.y += world_point->map->config.y_origin;
}


static gint motion_handler(GtkMapViewer *the_map_view, carmen_world_point_t *world_point,
		GdkEventMotion *event __attribute__ ((unused)))
{
	char buffer[1024];
	carmen_map_point_t point;
	carmen_map_t *the_map;


	if (the_map_view == NULL || the_map_view->internal_map == NULL)
		return TRUE;

	the_map = the_map_view->internal_map;

	world_point_to_global_world_point(world_point);
	carmen_world_to_map(world_point, &point);

	if (world_point->pose.x < 10000000 && world_point->pose.y < 10000000)
	sprintf(buffer, "Grid Cell: %d, %d\n(%.1f m, %.1f m)", point.x, point.y,
			world_point->pose.x, world_point->pose.y);
	gtk_label_set_text(GTK_LABEL(cursor_status_label), buffer);

	if (the_map != NULL)
	{
		sprintf(buffer, "Value: %.2f", the_map->map[point.x][point.y]);
		gtk_label_set_text(GTK_LABEL(value_label), buffer);
	}

	if ((placement_status == ORIENTING_ROBOT) ||
			(placement_status == ORIENTING_GOAL) ||
			(placement_status == ORIENTING_SIMULATOR) ||
			(placement_status == ORIENTING_FINAL_GOAL) ||
			(placement_status == ORIENTING_PERSON))
	{
		cursor_pos = *world_point;
		display_needs_updating = 1;
		do_redraw();
	}

	return TRUE;
}


void resend_coords(GtkWidget *widget __attribute__ ((unused)),
		gpointer	  data __attribute__ ((unused)))
{
	carmen_verbose("Robot: %.0f %.0f Goal: %.0f %.0f\n", last_robot.pose.x,
			last_robot.pose.y, goal.pose.x, goal.pose.y);

	if (goal.pose.x > 0)
	{
		navigator_set_goal(goal.pose.x, goal.pose.y, goal.pose.theta);
	}

	if (last_robot.pose.x > 0)
	{
		navigator_update_robot(&last_robot);
	}
}

static int button_press_handler(GtkMapViewer		*the_map_view __attribute__ ((unused)),
		carmen_world_point_p world_point __attribute__ ((unused)),
		GdkEventButton		*event __attribute__ ((unused)))
{
	if (the_map_view->internal_map == NULL)
	{
		return TRUE;
	}

	return TRUE;
}

static int
placing_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
{
	GdkCursor *cursor;

	if ((placement_status == PLACING_ROBOT) ||
			((placement_status == NO_PLACEMENT) &&
					((event->state & GDK_CONTROL_MASK) &&
							(event->button == 3))))
	{
		//		if (GTK_TOGGLE_BUTTON(autonomous_button)->active)
		//		{
		//			placement_status = NO_PLACEMENT;
		//			return TRUE;
		//		}

		world_point->pose.theta = robot.pose.theta;
		robot_temp = *world_point;

		placement_status = ORIENTING_ROBOT;
		cursor = gdk_cursor_new(GDK_EXCHANGE);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);


		return TRUE;
	}

	return FALSE;
}

static void
add_goal_to_internal_list(carmen_world_point_t goal)
{
	pointers *new_item = (pointers *) malloc(sizeof(pointers));
	new_item->point = goal;
	new_item->next	= NULL;

	if (queuePoints == NULL)
	{
		queuePoints = (fpointers *) malloc(sizeof(fpointers));
		queuePoints->begin = NULL;
		queuePoints->curr  = NULL;
		queuePoints->end   = NULL;
	}

	if (queuePoints->end != NULL)
	{
		queuePoints->end->next = new_item;
		queuePoints->end = new_item;
	}
	else
	{
		queuePoints->begin = new_item;
		queuePoints->end   = new_item;
	}

	if ((queuePoints != NULL) && (queuePoints->begin == queuePoints->end))
	{
		queuePoints->curr = queuePoints->begin;

		navigator_set_goal(queuePoints->curr->point.pose.x, queuePoints->curr->point.pose.y, queuePoints->curr->point.pose.theta);
	}
}


static int
placing_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
{
	GdkCursor *cursor;
	if ((placement_status == PLACING_GOAL) ||
			((placement_status == NO_PLACEMENT) && (event->button == 1) && (event->state & GDK_CONTROL_MASK)))
	{
		placement_status = NO_PLACEMENT;

		goal_temp = *world_point;


		cursor = gdk_cursor_new(GDK_EXCHANGE);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		placement_status = ORIENTING_GOAL;

		return TRUE;
	}

	return FALSE;
}

static void
place_final_goal_action(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	//	gtk_widget_set_sensitive(place_final_goal_button, TRUE);
	gtk_widget_set_sensitive(select_nearest_waypoint_button, TRUE);
	//gtk_widget_set_sensitive(update_map_button, FALSE);
	change_cursor(&carmen_yellow, &carmen_black);
	placement_status = PLACING_FINAL_GOAL;
}

static int
placing_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;

	if ( (placement_status == PLACING_FINAL_GOAL) )
	{
		final_goal = *world_point;
		cursor = gdk_cursor_new(GDK_EXCHANGE);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		placement_status = ORIENTING_FINAL_GOAL;

		return TRUE;
	}

	return FALSE;
}

static int
orienting_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;
	double angle;

	if (placement_status == ORIENTING_FINAL_GOAL)
	{
		placement_status = NO_PLACEMENT;

		angle = atan2(world_point->pose.y - final_goal.pose.y,
				world_point->pose.x - final_goal.pose.x);
		final_goal.pose.theta = angle;

		carmen_rddf_publish_end_point_message(50, final_goal.pose);

		cursor = gdk_cursor_new(GDK_LEFT_PTR);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);

		placement_status = SELECTING_NEAR_WAYPOINT;
		update_local_map = 1;

		return TRUE;
	}

	return FALSE;
}

static int
placing_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;

	if (placement_status == PLACING_PERSON)
	{
		new_person = *world_point;
		cursor	   = gdk_cursor_new(GDK_EXCHANGE);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		placement_status = ORIENTING_PERSON;
		return TRUE;
	}

	return FALSE;
}


static int
placing_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;

	if (placement_status == PLACING_SIMULATOR)
	{
		new_simulator = *world_point;
		cursor = gdk_cursor_new(GDK_EXCHANGE);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		placement_status = ORIENTING_SIMULATOR;
		return TRUE;
	}

	return FALSE;
}

static int
orienting_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
{
	GdkCursor *cursor;
	double angle;

	if ((placement_status == ORIENTING_ROBOT) ||
			((placement_status == NO_PLACEMENT) &&
					(((event->button == 2) && (event->state & GDK_CONTROL_MASK)) ||
							((event->button == 3) && (event->state & GDK_CONTROL_MASK)))))
	{
		placement_status = NO_PLACEMENT;

		//		if (GTK_TOGGLE_BUTTON(autonomous_button)->active)
		//		{
		//			return TRUE;
		//		}

		angle = atan2(world_point->pose.y - robot_temp.pose.y,
				world_point->pose.x - robot_temp.pose.x);
		robot_temp.pose.theta = angle;
		navigator_update_robot(&robot_temp);

		cursor = gdk_cursor_new(GDK_LEFT_PTR);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
	}

	return FALSE;
}

static int
orienting_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;

	if (placement_status == ORIENTING_GOAL)
	{
		placement_status = NO_PLACEMENT;
		goal_temp.pose.theta = atan2(world_point->pose.y - goal_temp.pose.y,
				world_point->pose.x - goal_temp.pose.x);

		cursor = gdk_cursor_new(GDK_LEFT_PTR);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);

		if (!behavior_selector_active)
		{
			add_goal_to_internal_list(goal_temp);
		}
		else
		{
			carmen_behavior_selector_add_goal(goal_temp.pose);
		}

		update_local_map = 1;

		return TRUE;
	}
	return FALSE;

}

static int
orienting_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;
	double angle, speed;
	if (placement_status == ORIENTING_PERSON)
	{
		placement_status = NO_PLACEMENT;

		angle = atan2(world_point->pose.y - new_person.pose.y,
				world_point->pose.x - new_person.pose.x);
		speed = hypot(world_point->pose.y - new_person.pose.y,
				world_point->pose.x - new_person.pose.x);
		speed /= 10.0;
		new_person.pose.theta = angle;
		carmen_simulator_ackerman_set_object(&(new_person.pose), speed,
				CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT);
		cursor = gdk_cursor_new(GDK_LEFT_PTR);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		return TRUE;
	}
	return FALSE;

}

static int
orienting_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
{
	GdkCursor *cursor;
	double angle;
	if (placement_status == ORIENTING_SIMULATOR)
	{
		placement_status = NO_PLACEMENT;
		angle = atan2(world_point->pose.y - new_person.pose.y,
				world_point->pose.x - new_person.pose.x);
		new_simulator.pose.theta = angle;
		carmen_simulator_ackerman_set_truepose(&(new_simulator.pose));
		cursor = gdk_cursor_new(GDK_LEFT_PTR);
		gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
		return TRUE;
	}
	return FALSE;

}


static int
button_release_handler(GtkMapViewer		   *the_map_view,
		carmen_world_point_t *world_point,
		GdkEventButton	   *event __attribute__ ((unused)))
{
	int rtr;

	world_point_to_global_world_point(world_point);

	if (the_map_view->internal_map == NULL)
		return TRUE;

	rtr = placing_robot_action(the_map_view, world_point, event);
	if(rtr)
		return TRUE;

	rtr = placing_goal_action(the_map_view, world_point, event);
	if(rtr)
		return TRUE;

	rtr = placing_person_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = placing_simulator_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = orienting_robot_action(the_map_view, world_point, event);
	if(rtr)
		return TRUE;

	rtr = orienting_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = orienting_person_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = orienting_simulator_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = selecting_final_region_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = placing_final_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = orienting_final_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = selecting_near_waypoint_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	return TRUE;
}


static void set_goal(GtkAction *action, gpointer user_data)
{
	int place_index;

	// Just to make the compiler happy
	action = action;

	for (place_index = 0; place_index < placelist->num_places; place_index++)
	{
		if (strcmp(user_data, placelist->places[place_index].name) == 0)
		{
			navigator_set_goal_by_place(placelist->places + place_index);
			return;
		}
	}
}

static void set_location(GtkAction *action, gpointer user_data)
{
	carmen_world_point_t point;
	char *name;
	int	  place_index;

	name = (char *)gtk_action_get_name(action);

	if (strcmp(name, "GlobalLocalization") == 0)
	{
		carmen_verbose("Global localization\n");
		navigator_update_robot(NULL);
		return;
	}

	for (place_index = 0; place_index < placelist->num_places; place_index++)
	{
		if (strcmp(user_data, placelist->places[place_index].name) == 0)
		{
			if (placelist->places[place_index].type == CARMEN_NAMED_POSITION_TYPE)
			{
				point.pose.x	 = placelist->places[place_index].x;
				point.pose.y	 = placelist->places[place_index].y;
				point.pose.theta = robot.pose.theta;
			}
			else
			{
				point.pose.x	 = placelist->places[place_index].x;
				point.pose.y	 = placelist->places[place_index].y;
				point.pose.theta = placelist->places[place_index].theta;
			}

			navigator_update_robot(&point);
		}
	}
}

static gint save_image(gpointer	  data __attribute__ ((unused)),
		guint	  action __attribute__ ((unused)),
		GtkWidget *widget  __attribute__ ((unused)))
{
	int x_size, y_size;
	int x_start, y_start;
	static int counter = 0;
	char filename[255];

	x_start = map_view->x_scroll_adj->value;
	y_start = map_view->y_scroll_adj->value;
	x_size	= carmen_fmin(gdk_pixbuf_get_width(map_view->current_pixbuf),
			map_view->port_size_x);
	y_size = carmen_fmin(gdk_pixbuf_get_height(map_view->current_pixbuf),
			map_view->port_size_y);

	sprintf(filename, "%s%02d.png",
			carmen_extract_filename(map_view->internal_map->config.map_name),
			counter++);

	if (display == CARMEN_NAVIGATOR_ENTROPY_v)
	{
		carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
				x_start, y_start, x_size, y_size);
	}
	else if (display == CARMEN_NAVIGATOR_UTILITY_v)
	{
		carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
				x_start, y_start, x_size, y_size);
	}
	else
	{
		carmen_graphics_write_pixmap_as_png(map_view->drawing_pixmap, filename,
				0, 0, x_size, y_size);
	}

	return 1;
}


int
get_algorithm_code(char *algorithm_name)
{
	int code = -1;
	if(strcmp(algorithm_name, "Gradient") == 0)
		code =  CARMEN_BEHAVIOR_SELECTOR_GRADIENT;
	else if(strcmp(algorithm_name, "A*") == 0)
		code =  CARMEN_BEHAVIOR_SELECTOR_A_STAR;
	else if(strcmp(algorithm_name, "RRT") == 0)
		code =  CARMEN_BEHAVIOR_SELECTOR_RRT;
	else if(strcmp(algorithm_name, "RDDF") == 0)
		code =  CARMEN_BEHAVIOR_SELECTOR_RDDF;
	else if(strcmp(algorithm_name, "Frenet") == 0)
		code =  CARMEN_BEHAVIOR_SELECTOR_FRENET;

	return code;
}


static void
parking_algorithm_selection_handler(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	navigator_set_algorithm(get_algorithm_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)parking_algorithm_selection)), BEHAVIOR_SELECTOR_PARK);
}

static void
follow_lane_selection_handler(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	navigator_set_algorithm(get_algorithm_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)follow_lane_algorithm_selection)), BEHAVIOR_SELECTOR_FOLLOW_ROUTE);
}

static int
get_state_code(char* state_name)
{
	if (state_name == NULL)
		return (0);
		
	if (strcmp(state_name, "Following Lane") == 0)
	{
		return (0);
	} 
	else if(strcmp(state_name, "Parking") == 0)
	{
		return (1);
	}
	else if(strcmp(state_name, "Human Intervention") == 0)
	{
		return (2);
	}

	return (-1);
}

static void
state_selection_handler(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	carmen_behavior_selector_set_task(get_state_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)state_selection)));
}


int
get_task_code(char *task_name)
{
	if (strcmp(task_name, "Follow Lane") == 0)
		return 0;
	else if(strcmp(task_name, "Park") == 0)
		return 1;
	else if(strcmp(task_name, "Human Intervention") == 0)
		return 2;

	return -1;
}

void navigator_graphics_update_behavior_selector_state(carmen_behavior_selector_state_message msg)
{
	behavior_selector_active = 1;

	if ((int) msg.task != get_task_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)state_selection)))
		gtk_combo_box_set_active((GtkComboBox*)state_selection, msg.task);

	if (msg.task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
	{
		if ((int) msg.algorithm != get_algorithm_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)follow_lane_algorithm_selection)))
			gtk_combo_box_set_active((GtkComboBox*)follow_lane_algorithm_selection, msg.algorithm);
	}
	else if (msg.task == BEHAVIOR_SELECTOR_PARK)
	{
		if ((int) msg.algorithm != get_algorithm_code(gtk_combo_box_text_get_active_text((GtkComboBoxText*)parking_algorithm_selection)))
			gtk_combo_box_set_active((GtkComboBox*)parking_algorithm_selection, msg.algorithm);
	}

	gtk_widget_set_sensitive(state_selection, 0);
}

int navigator_graphics_init(int argc, char *argv[],
		carmen_localize_ackerman_globalpos_message *msg,
		carmen_robot_ackerman_config_t *robot_conf_param,
		carmen_navigator_config_t *nav_conf_param,
		carmen_navigator_panel_config_t *nav_panel_conf_param)
{
	/* GtkWidget is the storage type for widgets */
	GtkWidget *main_box;
	GtkWidget *content_box;
	GtkWidget *status_container;
	GtkWidget *button_container;
	GtkWidget *button_box;
	GtkWidget *label_box;
	GtkWidget *menubar;
	GtkWidget *hseparator;


	/*   GtkWidget *resend_button; */
	GtkWidget *menu_item;
	int index;
	int sync_mode_var = 0;

	gtk_init(&argc, &argv);

	carmen_graphics_setup_colors();
	robot_colour  = DEFAULT_ROBOT_COLOUR;
	goal_colour	  = DEFAULT_GOAL_COLOUR;
	path_colour	  = DEFAULT_PATH_COLOUR;
	tree_colour   = DEFAULT_TREE_COLOUR;
	people_colour = DEFAULT_PEOPLE_COLOUR;

	initialize_path_vector();

	nav_panel_config = nav_panel_conf_param;

	if ((nav_panel_config->initial_map_zoom < 1.0) ||
			(nav_panel_config->initial_map_zoom > 100.0))
	{
		nav_panel_config->initial_map_zoom = 100.0;
	}

	/* Create a new window */
	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	g_signal_connect(GTK_OBJECT(window), "destroy",
			G_CALLBACK(gtk_main_quit), "WM destroy");

	gtk_window_set_title(GTK_WINDOW(window), "CARMEN Planner");
	g_signal_connect(GTK_OBJECT(window), "delete_event",
			G_CALLBACK(delete_event), NULL);
	gtk_container_set_border_width(GTK_CONTAINER(window), 0);

	//main box
	main_box = gtk_vbox_new(FALSE, 0);
	gtk_container_border_width(GTK_CONTAINER(main_box), 0);
	gtk_container_add(GTK_CONTAINER(window), main_box);

	//menu
	menubar = get_main_menu();
	gtk_box_pack_start(GTK_BOX(main_box), menubar, FALSE, FALSE, 0);

	//content box
	content_box = gtk_hbox_new(FALSE, 0);
	gtk_container_border_width(GTK_CONTAINER(content_box), 5);
	gtk_container_add(GTK_CONTAINER(main_box), content_box);

	//left box
	{
		GtkWidget *frame;
		frame = gtk_frame_new("Buttons");
		gtk_container_border_width(GTK_CONTAINER(frame), 2);

		button_container = gtk_vbox_new(FALSE, 0);
		gtk_container_border_width(GTK_CONTAINER(button_container), 0);

		gtk_container_add(GTK_CONTAINER(frame), button_container);
		gtk_box_pack_start(GTK_BOX(content_box), frame, FALSE, FALSE, 0);
	}


	//map_view
	{
		map_view = carmen_map_graphics_new_viewer(460, 400, nav_panel_config->initial_map_zoom);

		carmen_map_graphics_add_motion_event
		(map_view, (carmen_graphics_mapview_callback_t)motion_handler);
		carmen_map_graphics_add_button_release_event
		(map_view, (carmen_graphics_mapview_callback_t)button_release_handler);
		carmen_map_graphics_add_button_press_event
		(map_view, (carmen_graphics_mapview_callback_t)button_press_handler);
		carmen_map_graphics_add_drawing_func
		(map_view, (carmen_graphics_mapview_drawing_func_t)draw_robot_objects);

		gtk_box_pack_start(GTK_BOX(content_box), map_view->map_box, TRUE, TRUE, 0);
	}

	//right box
	{
		status_container = gtk_hbox_new(FALSE, 0);
		gtk_container_border_width(GTK_CONTAINER(status_container), 0);

		gtk_box_pack_start(GTK_BOX(content_box), status_container, FALSE, FALSE, 0);
	}


	//label box
	{
		label_box = gtk_vbox_new(FALSE, 0);

		gtk_container_border_width(GTK_CONTAINER(label_box), 0);

		construct_status_frame(label_box);

		gtk_box_pack_start(GTK_BOX(status_container), label_box, FALSE, FALSE, 0);
	}


	if (nav_panel_config->use_ackerman)
	{
		{
			button_box = gtk_hbutton_box_new();
			gtk_container_border_width(GTK_CONTAINER(button_box), 5);
			gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
			gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
			gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
			gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .4,
					BUTTON_HEIGHT);
		}

		{
			button_box = gtk_hbutton_box_new();
			gtk_container_border_width(GTK_CONTAINER(button_box), 5);
			gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
			gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
			gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
			gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .4,
					BUTTON_HEIGHT);
		}

		{
			new_label("State:", button_box);
			state_selection = gtk_combo_box_text_new();
			gtk_combo_box_text_append_text((GtkComboBoxText*) state_selection, "Following Lane");
			gtk_combo_box_text_append_text((GtkComboBoxText*)state_selection, "Parking");
			//			gtk_combo_box_text_append_text((GtkComboBoxText*)state_selection, "Human Intervention");
			gtk_combo_box_set_active((GtkComboBox*)state_selection, 0);
			g_signal_connect(GTK_OBJECT(state_selection), "changed", G_CALLBACK(state_selection_handler), NULL);
			gtk_box_pack_end(GTK_BOX(button_box), state_selection, FALSE, FALSE, 0);
		}

		{
			button_box = gtk_hbutton_box_new();
			gtk_container_border_width(GTK_CONTAINER(button_box), 5);
			gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
			gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
			gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
			gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .4,
					BUTTON_HEIGHT);
		}

		{
			new_label("Follow Lane:", button_box);
			follow_lane_algorithm_selection = gtk_combo_box_text_new();
			gtk_combo_box_text_append_text((GtkComboBoxText*) follow_lane_algorithm_selection, "Gradient");
			gtk_combo_box_text_append_text((GtkComboBoxText*)follow_lane_algorithm_selection, "A*");
			gtk_combo_box_text_append_text((GtkComboBoxText*)follow_lane_algorithm_selection, "RRT");
			gtk_combo_box_text_append_text((GtkComboBoxText*)follow_lane_algorithm_selection, "RDDF");
			gtk_combo_box_set_active((GtkComboBox*)follow_lane_algorithm_selection, 0);
			g_signal_connect(GTK_OBJECT(follow_lane_algorithm_selection), "changed", G_CALLBACK(follow_lane_selection_handler), NULL);
			gtk_box_pack_end(GTK_BOX(button_box), follow_lane_algorithm_selection, FALSE, FALSE, 0);
		}

		{
			button_box = gtk_hbutton_box_new();
			gtk_container_border_width(GTK_CONTAINER(button_box), 5);
			gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
			gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
			gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
			gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .4,
					BUTTON_HEIGHT);
		}

		{
			new_label("Parking:", button_box);
			parking_algorithm_selection = gtk_combo_box_text_new();
			gtk_combo_box_text_append_text((GtkComboBoxText*)parking_algorithm_selection, "");
			gtk_combo_box_text_append_text((GtkComboBoxText*)parking_algorithm_selection, "A*");
			gtk_combo_box_text_append_text((GtkComboBoxText*)parking_algorithm_selection, "RRT");
			gtk_combo_box_set_active((GtkComboBox*)parking_algorithm_selection, 0);
			g_signal_connect(GTK_OBJECT(parking_algorithm_selection), "changed", G_CALLBACK(parking_algorithm_selection_handler), NULL);
			gtk_box_pack_end(GTK_BOX(button_box), parking_algorithm_selection, FALSE, FALSE, 0);
		}

	}

	{
		button_box = gtk_hbutton_box_new();
		gtk_container_border_width(GTK_CONTAINER(button_box), 5);
		gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
		gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
		gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
		gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .47,
				BUTTON_HEIGHT);
	}

	{
		place_robot_button = gtk_button_new_with_label("Place Robot");
		g_signal_connect(GTK_OBJECT(place_robot_button), "clicked",
				G_CALLBACK(place_robot), NULL);
		gtk_box_pack_start(GTK_BOX(button_box), place_robot_button, FALSE, FALSE, 0);
	}

	{
		place_goal_button = gtk_button_new_with_label("Place Goal");
		g_signal_connect(GTK_OBJECT(place_goal_button), "clicked",
				G_CALLBACK(place_goal), NULL);
		gtk_box_pack_end(GTK_BOX(button_box), place_goal_button, FALSE, FALSE, 0);
	}

	{
		button_box = gtk_hbutton_box_new();
		gtk_container_border_width(GTK_CONTAINER(button_box), 5);
		gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
		gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
		gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
		gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .4,
				BUTTON_HEIGHT);
	}

	{
		decrement_point_button = gtk_button_new_with_label("Remove Goal");
		g_signal_connect(GTK_OBJECT(decrement_point_button), "clicked", G_CALLBACK(
				decrement_point), NULL);
		gtk_box_pack_start(GTK_BOX(button_box), decrement_point_button, TRUE, FALSE, 0);
	}

	{
		clear_all_goals_button = gtk_button_new_with_label("Clear Goals");
		g_signal_connect(GTK_OBJECT(clear_all_goals_button), "clicked", G_CALLBACK(
				execute_clear_all_goals), NULL);
		gtk_box_pack_end(GTK_BOX(button_box), clear_all_goals_button, TRUE, FALSE, 0);
	}


	{
		button_box = gtk_hbutton_box_new();
		gtk_container_border_width(GTK_CONTAINER(button_box), 5);
		gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
		gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
		gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
		gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * 0.98,
				BUTTON_HEIGHT);
	}

	{
		autonomous_button = gtk_toggle_button_new_with_label("Go");
		g_signal_connect(GTK_OBJECT(autonomous_button), "clicked",
				G_CALLBACK(go_autonomous), (gpointer)"Autonomous");
		gtk_box_pack_start(GTK_BOX(button_box), autonomous_button, FALSE, FALSE, 0);
	}

	if (nav_panel_config->use_ackerman)
	{
		hseparator = gtk_hseparator_new();
		gtk_box_pack_start(GTK_BOX(button_container), hseparator, FALSE, FALSE, 0);

		{
			button_box = gtk_vbutton_box_new();
			gtk_container_border_width(GTK_CONTAINER(button_box), 5);
			gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
			gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
			gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 2);
			gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * 0.98 	,
					BUTTON_HEIGHT);
		}

		{
			{
				button_box = gtk_hbutton_box_new();
				gtk_container_border_width(GTK_CONTAINER(button_box), 5);
				gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
				gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
				gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 10);
				gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * .47,
						BUTTON_HEIGHT);
			}
			{
				zoom_in_global_map_button = gtk_button_new_with_label("Zoom in");
				g_signal_connect(GTK_OBJECT(zoom_in_global_map_button), "clicked",
						G_CALLBACK(zoom_in_global_map_action), NULL);
				gtk_box_pack_start(GTK_BOX(button_box), zoom_in_global_map_button, FALSE, FALSE, 0);
			}

			{
				zoom_out_global_map_button = gtk_button_new_with_label("Zoom out");
				g_signal_connect(GTK_OBJECT(zoom_out_global_map_button), "clicked",
						G_CALLBACK(zoom_out_global_map_action), NULL);
				gtk_box_pack_start(GTK_BOX(button_box), zoom_out_global_map_button, FALSE, FALSE, 0);
			}
			{
				button_box = gtk_vbutton_box_new();
				gtk_container_border_width(GTK_CONTAINER(button_box), 5);
				gtk_box_pack_start(GTK_BOX(button_container), button_box, FALSE, FALSE, 0);
				gtk_button_box_set_layout(GTK_BUTTON_BOX(button_box), GTK_BUTTONBOX_SPREAD);
				gtk_button_box_set_spacing(GTK_BUTTON_BOX(button_box), 2);
				gtk_button_box_set_child_size(GTK_BUTTON_BOX(button_box), BUTTON_WIDTH * 0.98 	,
						BUTTON_HEIGHT);
			}
		}


		{
			place_final_goal_button = gtk_button_new_with_label("Place Final Goal");
			g_signal_connect(GTK_OBJECT(place_final_goal_button), "clicked",
					G_CALLBACK(place_final_goal_action), NULL);
			gtk_box_pack_end(GTK_BOX(button_box), place_final_goal_button, FALSE, FALSE, 0);
		}

	}

	hseparator = gtk_hseparator_new();
	gtk_box_pack_start(GTK_BOX(button_container), hseparator, FALSE, FALSE, 0);

	//  gtk_container_add(GTK_CONTAINER(label_box), hseparator);
	gtk_widget_set_usize(hseparator, BUTTON_WIDTH, 5);

	simulator_box = gtk_vbutton_box_new();
	gtk_container_border_width(GTK_CONTAINER(simulator_box), 5);

	//  gtk_container_add (GTK_CONTAINER (label_box), simulator_box);
	gtk_box_pack_start(GTK_BOX(button_container), simulator_box, FALSE, FALSE, 0);

	gtk_button_box_set_layout(GTK_BUTTON_BOX(simulator_box),
			GTK_BUTTONBOX_START);
	gtk_button_box_set_spacing(GTK_BUTTON_BOX(simulator_box), 10);
	gtk_button_box_set_child_size(GTK_BUTTON_BOX(simulator_box), BUTTON_WIDTH,
			BUTTON_HEIGHT);

	place_simulator_button = gtk_button_new_with_label("Place Simulator");
	g_signal_connect(GTK_OBJECT(place_simulator_button), "clicked",
			G_CALLBACK(place_simulator), NULL);
	gtk_box_pack_start(GTK_BOX(simulator_box), place_simulator_button,
			FALSE, FALSE, 0);
	next_tick_button = gtk_button_new_with_label("Next Tick");
	g_signal_connect(GTK_OBJECT(next_tick_button), "clicked",
			G_CALLBACK(next_tick), NULL);
	gtk_box_pack_start(GTK_BOX(simulator_box), next_tick_button,
			FALSE, FALSE, 0);
	sync_mode_button = gtk_toggle_button_new_with_label("Sync Mode");
	carmen_param_set_module(NULL);
	carmen_param_get_onoff("simulator_sync_mode", &sync_mode_var, NULL);
	carmen_param_subscribe_onoff("simulator", "sync_mode", NULL,
			(carmen_param_change_handler_t)
			sync_mode_change_handler);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(sync_mode_button),
			sync_mode_var);
	g_signal_connect(GTK_OBJECT(sync_mode_button), "clicked",
			G_CALLBACK(sync_mode), (gpointer)"Autonomous");
	gtk_box_pack_start(GTK_BOX(simulator_box), sync_mode_button, FALSE,
			FALSE, 0);

	filler_box = gtk_vbox_new(FALSE, 0);
	gtk_widget_set_usize(filler_box, BUTTON_WIDTH, 100);
	gtk_container_border_width(GTK_CONTAINER(filler_box), 0);

	//  gtk_container_add (GTK_CONTAINER (label_box), filler_box);
	gtk_box_pack_start(GTK_BOX(button_container), filler_box, FALSE, FALSE, 0);

	gtk_widget_show(filler_box);

	for (index = 0; index < GRADIENT_COLORS; index++)
	{
		RedBlueGradient[index] =
				carmen_graphics_add_color_rgb(255 - 255 * index / (float)GRADIENT_COLORS,
						0, 255 * index / (float)GRADIENT_COLORS);
	}

	gtk_widget_show_all(window);
	gtk_widget_hide(simulator_box);
	simulator_hidden = 1;

	gtk_widget_grab_focus(window);

	menu_item =
			gtk_ui_manager_get_widget(ui_manager, "/ui/MainMenu/FileMenu/StopFilming");
	gtk_widget_hide(menu_item);

	globalpos = msg;

	robot_config = robot_conf_param;
	nav_config	 = nav_conf_param;

	cursor_pos.map = NULL;

	return (0);
}

void navigator_graphics_add_ipc_handler(GdkInputFunction handle_ipc)
{
	carmen_graphics_update_ipc_callbacks(handle_ipc);
}

void navigator_graphics_change_map(carmen_map_p new_map)
{
	char buffer[1024];

	memset(buffer, 0, 1024);

	carmen_map_graphics_add_map(map_view, new_map, 0);

	if (people)
	{
		people->length = 0;
	}

	if (new_map->config.map_name != NULL)
	{
		sprintf(buffer, "Map: %s",
				carmen_extract_filename(new_map->config.map_name));
	}

	gtk_label_set_text(GTK_LABEL(map_status_label), buffer);


	sprintf(buffer, "Origin: (%ld, %ld)", (long int) new_map->config.x_origin, (long int) new_map->config.y_origin);
	gtk_label_set_text(GTK_LABEL(map_origin_label), buffer);
}

void navigator_graphics_display_map(carmen_map_t *new_map, carmen_navigator_map_t type)
{
	char name[100];
	int	 flags = 0;

	display = type;

	switch (type)
	{
	case CARMEN_NAVIGATOR_MAP_v:
		strcpy(name, "Map");
		break;

	case CARMEN_NAVIGATOR_ENTROPY_v:
		strcpy(name, "Entropy");
		flags = CARMEN_GRAPHICS_RESCALE;
		break;

	case CARMEN_NAVIGATOR_COST_v:
		strcpy(name, "Costs");
		flags = CARMEN_GRAPHICS_RESCALE;
		break;

	case CARMEN_NAVIGATOR_UTILITY_v:
		strcpy(name, "Utility");
		flags = CARMEN_GRAPHICS_RESCALE | CARMEN_GRAPHICS_INVERT;
		break;

	case CARMEN_LOCALIZE_LMAP_v:
		strcpy(name, "Likelihood Map");
		flags = CARMEN_GRAPHICS_RESCALE;
		break;

	case CARMEN_LOCALIZE_GMAP_v:
		strcpy(name, "Global Likelihood Map");
		flags = CARMEN_GRAPHICS_RESCALE;
		break;

	case CARMEN_LANE_MAP_v:
		strcpy(name, "Lane");
		break;

	case CARMEN_OFFLINE_MAP_v:
		strcpy(name, "Offline Map");
		break;

	case CARMEN_COMPLETE_MAP_v:
		strcpy(name, "Complete Map");
		break;

	default:
		return;
	}

	carmen_map_graphics_add_map(map_view, new_map, flags);
}

void navigator_graphics_add_placelist(carmen_map_placelist_p new_placelist)
{
	char  name[1024], label[1024], menu_path[1024];
	int	  index;
	int	 *merge_uid, new_merge_uid;
	char *underscore;
	GtkAction *action;

	carmen_verbose("Received %d places\n", new_placelist->num_places);

	if (place_action_uids != NULL)
	{
		for (index = 0; index < place_action_uids->length; index++)
		{
			merge_uid = (int *)carmen_list_get(place_action_uids, index);
			gtk_ui_manager_remove_ui(ui_manager, *merge_uid);

			action = carmen_list_get(goal_actions, index);
			gtk_action_group_remove_action(goal_action_group, action);
			g_object_unref(action);

			action = carmen_list_get(start_actions, index);
			gtk_action_group_remove_action(start_action_group, action);
			g_object_unref(action);
		}

		free(placelist->places);
	}
	else
	{
		place_action_uids = carmen_list_create
				(sizeof(int), new_placelist->num_places);
		goal_actions = carmen_list_create
				(sizeof(GtkAction *), new_placelist->num_places);
		start_actions = carmen_list_create
				(sizeof(GtkAction *), new_placelist->num_places);
		placelist = (carmen_map_placelist_p) calloc(1, sizeof(carmen_map_placelist_t));
		carmen_test_alloc(placelist);
		goal_action_group  = gtk_action_group_new("Goals");
		start_action_group = gtk_action_group_new("StartLocations");
		gtk_ui_manager_insert_action_group(ui_manager, goal_action_group, 1);
		gtk_ui_manager_insert_action_group(ui_manager, start_action_group, 2);
	}

	placelist->num_places	  = new_placelist->num_places;
	place_action_uids->length = 0;

	if (placelist->num_places == 0)
	{
		return;
	}

	placelist->places = (carmen_place_p)calloc(placelist->num_places,
			sizeof(carmen_place_t));
	carmen_test_alloc(placelist->places);
	memcpy(placelist->places, new_placelist->places,
			sizeof(carmen_place_t) * placelist->num_places);

	for (index = 0; index < placelist->num_places; index++)
	{
		strcpy(label, placelist->places[index].name);
		label[0] = toupper(label[0]);

		do
		{
			underscore = strchr(label, '_');

			if (underscore)
			{
				*underscore = ' ';
			}
		} while (underscore);

		sprintf(name, "Goal%s", placelist->places[index].name);
		action = gtk_action_new(name, label, NULL, NULL);
		gtk_action_group_add_action(goal_action_group, action);
		carmen_list_add(goal_actions, &action);
		g_signal_connect(action, "activate", G_CALLBACK(set_goal),
				placelist->places[index].name);

		sprintf(menu_path, "/ui/MainMenu/GoalMenu/");
		new_merge_uid = gtk_ui_manager_new_merge_id(ui_manager);
		gtk_ui_manager_add_ui(ui_manager, new_merge_uid, menu_path,
				name, name, GTK_UI_MANAGER_MENUITEM, FALSE);
		carmen_list_add(place_action_uids, &new_merge_uid);

		sprintf(name, "Start%s", placelist->places[index].name);
		action = gtk_action_new(name, label, NULL, NULL);
		gtk_action_group_add_action(start_action_group, action);
		carmen_list_add(start_actions, &action);
		g_signal_connect(action, "activate", G_CALLBACK(set_location),
				placelist->places[index].name);

		sprintf(menu_path, "/ui/MainMenu/StartLocationMenu/");
		new_merge_uid = gtk_ui_manager_new_merge_id(ui_manager);
		gtk_ui_manager_add_ui(ui_manager, new_merge_uid, menu_path,
				name, name, GTK_UI_MANAGER_MENUITEM, FALSE);
		carmen_list_add(place_action_uids, &new_merge_uid);
	}
}

void navigator_graphics_update_dynamics(void)
{
	display_needs_updating = 1;
	do_redraw();
}

void navigator_graphics_initialize_dynamics(carmen_list_t *new_people)
{
	people = new_people;
}

void navigator_graphics_update_display(carmen_traj_point_p	new_robot,
		carmen_world_point_p new_goal,
		int					autonomous)
{
	char   buffer[255];
	double robot_distance = 0.0, goal_distance = 0.0;
	carmen_world_point_t new_robot_w;
	static int previous_width = 0, previous_height = 0;
	double	   delta_angle;
	int	   autonomous_change = 0;
	int	   update_map_change = 0;
	double adjust_distance;

	if (!autonomous &&
			gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autonomous_button)))
	{
		ignore_click = 1;
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autonomous_button), 0);
		label_autonomy_button("Go");
		autonomous_change = 1;
	}

	if (autonomous &&
			!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autonomous_button)))
	{
		ignore_click = 1;
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autonomous_button), 1);
		label_autonomy_button("Stop");
		autonomous_change = 1;
	}

	if (!map_view->internal_map)
	{
		return;
	}

	//	if (time_of_simulator_update - carmen_get_time() > 30)
	//	{
	//		gtk_widget_hide(simulator_box);
	//		gtk_widget_show(filler_box);
	//		simulator_hidden = 1;
	//	}

	adjust_distance = carmen_fmax
			(map_view->internal_map->config.x_size / (double)map_view->port_size_x,
					map_view->internal_map->config.y_size / (double)map_view->port_size_y);

	adjust_distance *= map_view->internal_map->config.resolution *
			(map_view->zoom / 100.0);
	adjust_distance *= 10;


	if (new_robot)
	{
		robot_traj = *new_robot;
	}

	new_robot_w.pose.x	   = robot_traj.x;
	new_robot_w.pose.y	   = robot_traj.y;
	new_robot_w.pose.theta = robot_traj.theta;
	new_robot_w.map = map_view->internal_map;

	robot_distance	= carmen_distance_world(&new_robot_w, &(map_view->centre));

	if (nav_panel_config->track_robot &&
			((robot_distance > adjust_distance) ||
					(previous_width != map_view->image_widget->allocation.width) ||
					(previous_height != map_view->image_widget->allocation.height)))
	{
		carmen_map_graphics_adjust_scrollbars(map_view, &robot);
	}

	robot_distance = carmen_distance_world(&new_robot_w, &robot);
	delta_angle	   =
			carmen_normalize_theta(new_robot_w.pose.theta - robot.pose.theta);

	robot = new_robot_w;

	if (new_goal)
	{
		goal_distance = carmen_distance_world(new_goal, &goal);
		goal = *new_goal;
	}
	else
	{
		goal_distance = 0.0;
	}

	previous_width	= map_view->image_widget->allocation.width;
	previous_height = map_view->image_widget->allocation.height;

	if (autonomous_change || update_map_change || (robot_distance > 1.0) || (goal_distance > 1.0) ||
			(fabs(delta_angle) > carmen_degrees_to_radians(0.01)))
	{
		display_needs_updating = 1;
	}

	sprintf(buffer, "Robot: %5.1f m, %5.1f m, %6.2f", robot.pose.x,
			robot.pose.y, carmen_radians_to_degrees(robot.pose.theta));
	gtk_label_set_text(GTK_LABEL(robot_status_label), buffer);

	sprintf(buffer, "Velocity: %5.1f km/h (%5.1f m/s), %5.1f %s", robot_traj.t_vel * 3.6, robot_traj.t_vel,
			carmen_radians_to_degrees(robot_traj.r_vel), (nav_panel_config->use_ackerman ? "deg" : "deg/s"));
	gtk_label_set_text(GTK_LABEL(robot_speed_label), buffer);

	sprintf(buffer, "Goal: %.1f m, %.1f m", goal.pose.x, goal.pose.y);
	gtk_label_set_text(GTK_LABEL(goal_status_label), buffer);

	last_navigator_update = carmen_get_time();

	do_redraw();
}

void navigator_graphics_update_path(carmen_ackerman_traj_point_t* new_path, int path_length, int path_id)
{
	int i, path_index;

	if (map_view->internal_map == NULL)
		return;

	path_index = path_id % PATH_VECTOR_SIZE;

	if (path_vector[path_index])
	{
		free(path_vector[path_index]);
		path_vector[path_index] = NULL;
	}

	path_vector_size[path_index] = path_length;

	if (path_length > 0)
	{
		path_vector[path_index] = (carmen_world_point_t *)calloc(path_vector_size[path_index], sizeof(carmen_world_point_t));
		carmen_test_alloc(path_vector[path_index]);

		for (i = 0; i < path_vector_size[path_index]; i++)
		{
			path_vector[path_index][i].pose.x	   = new_path[i].x;
			path_vector[path_index][i].pose.y	   = new_path[i].y;
			path_vector[path_index][i].pose.theta = new_path[i].theta;
			path_vector[path_index][i].map = map_view->internal_map;
		}
	}


	do_redraw();
}

void navigator_graphics_update_plan_tree(carmen_ackerman_traj_point_p p1, carmen_ackerman_traj_point_p p2, int plan_tree_length)
{
	int index;

	if (map_view->internal_map == NULL)
	{
		return;
	}

	if (plan_tree_p1 != NULL)
	{
		free(plan_tree_p1);
		plan_tree_p1 = NULL;
	}

	if (plan_tree_p2 != NULL)
	{
		free(plan_tree_p2);
		plan_tree_p2 = NULL;
	}

	num_plan_tree_points = plan_tree_length;

	if (plan_tree_length > 0)
	{
		plan_tree_p1 = (carmen_world_point_t *)calloc(plan_tree_length, sizeof(carmen_world_point_t));
		carmen_test_alloc(plan_tree_p1);
		plan_tree_p2 = (carmen_world_point_t *)calloc(plan_tree_length, sizeof(carmen_world_point_t));
		carmen_test_alloc(plan_tree_p2);
		carmen_verbose("Got path of length %d\n", plan_tree_length);

		for (index = 0; index < num_plan_tree_points; index++)
		{
			plan_tree_p1[index].pose.x	   = p1[index].x;
			plan_tree_p1[index].pose.y	   = p1[index].y;
			plan_tree_p1[index].pose.theta = p1[index].theta;
			plan_tree_p1[index].map = map_view->internal_map;
			carmen_verbose("%.1f %.1f\n", plan_tree_p1[index].pose.x,
					plan_tree_p1[index].pose.y);

		}

		for (index = 0; index < num_plan_tree_points; index++)
		{
			plan_tree_p2[index].pose.x	   = p2[index].x;
			plan_tree_p2[index].pose.y	   = p2[index].y;
			plan_tree_p2[index].pose.theta = p2[index].theta;
			plan_tree_p2[index].map = map_view->internal_map;
			carmen_verbose("%.1f %.1f\n", plan_tree_p2[index].pose.x,
					plan_tree_p2[index].pose.y);
		}
	}
	else
	{
		num_plan_tree_points = 0;
	}

	display_needs_updating = 1;
	do_redraw();
}

carmen_world_point_p navigator_graphics_get_current_path(void)
{
	return path;
}


void navigator_graphics_update_fused_odometry(carmen_point_t fused_odometry_pose)
{
	char buffer[256];

	fused_odometry_position.map = map_view->internal_map;
	fused_odometry_position.pose = fused_odometry_pose;

	sprintf(buffer, "Fused pos: %5.1f m, %5.1f m, %6.2f", fused_odometry_pose.x,
			fused_odometry_pose.y, carmen_radians_to_degrees(fused_odometry_pose.theta));
	gtk_label_set_text(GTK_LABEL(fused_odometry_status_label), buffer);

	do_redraw();
}


void navigator_graphics_update_simulator_truepos(carmen_point_t truepose)
{
	time_of_simulator_update = carmen_get_time();

	if (simulator_hidden)
	{
		gtk_widget_show_all(simulator_box);
		gtk_widget_hide(filler_box);
		simulator_hidden = 0;

		if (!GTK_TOGGLE_BUTTON(sync_mode_button)->active)
		{
			gtk_widget_hide(next_tick_button);
		}
	}

	simulator_trueposition.pose = truepose;
	simulator_trueposition.map	= map_view->internal_map;
	last_simulator_update  = carmen_get_time();
	display_needs_updating = 1;
	do_redraw();
}

void navigator_graphics_update_goal_list(carmen_ackerman_traj_point_t* goal_list, int size)
{
	int i;
	if(navigator_goal_list)
		free(navigator_goal_list);

	goal_list_size = size;

	navigator_goal_list = (carmen_world_point_t*) malloc(sizeof(carmen_world_point_t) * goal_list_size);

	for(i = 0; i < size; i++)
	{
		navigator_goal_list[i].pose.x = goal_list[i].x;
		navigator_goal_list[i].pose.y = goal_list[i].y;
		navigator_goal_list[i].pose.theta = goal_list[i].theta;
		navigator_goal_list[i].map = map_view->internal_map;
	}

	do_redraw();
}

void navigator_graphics_update_waypoint_list(carmen_ackerman_traj_point_t* waypoint_list, int size)
{
	int i;
	if(navigator_waypoint_list)
		free(navigator_waypoint_list);

	waypoint_list_size = size;

	navigator_waypoint_list = (carmen_world_point_t*) malloc(sizeof(carmen_world_point_t) * waypoint_list_size);

	for(i = 0; i < size; i++)
	{
		navigator_waypoint_list[i].pose.x = waypoint_list[i].x;
		navigator_waypoint_list[i].pose.y = waypoint_list[i].y;
		navigator_waypoint_list[i].pose.theta = waypoint_list[i].theta;
		navigator_waypoint_list[i].map = map_view->internal_map;
	}

	do_redraw();
}

void navigator_graphics_update_simulator_objects(int num_objects, carmen_simulator_ackerman_objects_t *objects_list)
{
	int i;

	if (simulator_objects == NULL)
	{
		if (num_objects == 0)
		{
			return;
		}

		simulator_objects = carmen_list_create(sizeof(carmen_simulator_ackerman_objects_t), num_objects);
	}

	simulator_objects->length = 0;

	for (i = 0; i < num_objects; i++)
	{
		carmen_list_add(simulator_objects, objects_list + i);
	}

	display_needs_updating = 1;
	do_redraw();
}

static void sync_mode_change_handler(char *module __attribute__ ((unused)),
		char *variable __attribute__ ((unused)),
		char *value)
{
	int new_value;

	if ((strlen(value) >= 2) && (strncmp(value, "on", 2) == 0))
	{
		new_value = 1;
	}
	else if ((strlen(value) >= 3) && (strncmp(value, "off", 3) == 0))
	{
		new_value = 0;
	}
	else
	{
		return;
	}

	GTK_TOGGLE_BUTTON(sync_mode_button)->active = new_value;

	if (simulator_hidden)
	{
		return;
	}

	if (GTK_TOGGLE_BUTTON(sync_mode_button)->active)
	{
		gtk_widget_show(next_tick_button);
	}
	else
	{
		gtk_widget_hide(next_tick_button);
	}

}

void navigator_graphics_start(char *path)
{
	map_path = path;
	gtk_main();
}
