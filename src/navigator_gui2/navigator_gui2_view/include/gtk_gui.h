#ifndef MVOG_GTK_GUI_GTK_GUI_H
#define MVOG_GTK_GUI_GTK_GUI_H

#define GL_GLEXT_PROTOTYPES

#include <string>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include "draw_callbacks.h"
#include "../../navigator_gui2_main.h"

#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/map_graphics.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_interface.h>
#include <carmen/collision_detection.h>
#include <X11/Xlib.h>
#include <X11/cursorfont.h>
#include <gdk/gdkx.h>
#include <ctype.h>

#include <carmen/map_server_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/dynamic_object_detector_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/lane_detector_interface.h>
#include <carmen/model_predictive_planner_interface.h>
#include <car_panel.h>

#include <carmen/rddf_util.h>
#include <carmen/carmen_gps_wrapper.h>

#define DEFAULT_ROBOT_COLOUR 	carmen_red
#define DEFAULT_GOAL_COLOUR 	carmen_yellow
#define DEFAULT_PATH_COLOUR 	carmen_light_blue
#define DEFAULT_TREE_COLOUR 	carmen_orange
#define DEFAULT_PEOPLE_COLOUR 	carmen_orange

#define PATH_VECTOR_SIZE 20
#define ALWAYS_REDRAW 0
#define GRADIENT_COLORS 40


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
	SELECTING_NEAR_WAYPOINT,
	EDITING_NEAR_RDDF
} placement_t;

typedef struct {
	int n_points;
	double *points;
	double displacement;
} carmen_polygon_config_t;

typedef struct points
{
	carmen_world_point_t point;
	struct points *next;
} pointers;

typedef struct fpoints
{
	pointers *curr;
	pointers *begin;
	pointers *end;
	int count;
} fpointers;

namespace View
{

	class GtkGui
	{
	private:



	public:

		struct Controls
		{
			/* Main application window */
			GtkWidget *drawArea;
			GtkWidget *drawAreaCarPanel;
			GtkWidget *main_window;
			GtkHBox* box_map_2d;
			GtkMapViewer *map_view;

			GtkComboBox *comboGoalSource;
			GtkComboBox *comboState;
			GtkComboBox *comboFollowLane;
			GtkComboBox *comboParking;

			GtkLabel *labelStatusMap;

			GtkLabel *labelFusedOdometry;
			GtkLabel *labelOrigin;
			GtkLabel *labelRobot;
			GtkLabel *labelVelocity;
			GtkLabel *labelGoal;
			GtkLabel *labelGridCell;
			GtkLabel *labelValue;
			GtkLabel *labelDistTraveled;
			GtkLabel *labelGlobalPosTimeStamp;
			GtkLabel *labelLowLevelState;
			GtkLabel *labelTrafficSignState;

			GtkLabel *labelNavConTimestamp;

			GtkToggleButton* buttonSyncMode;
			GtkToggleButton* buttonNextTick;
			GtkToggleButton* buttonGo;

			GtkCheckMenuItem* menuDisplay_TrackRobot;
			GtkCheckMenuItem* menuDisplay_DrawPath;
			GtkCheckMenuItem* menuDisplay_DrawWaipoints;
			GtkCheckMenuItem* menuDisplay_DrawRobotWaipoints;
			GtkCheckMenuItem* menuDisplay_ShowLateralOffset;
			GtkCheckMenuItem* menuDisplay_ShowParticles;
			GtkCheckMenuItem* menuDisplay_ShowFusedOdometry;
			GtkCheckMenuItem* menuDisplay_ShowGaussians;
			GtkCheckMenuItem* menuDisplay_ShowLaserData;
			GtkCheckMenuItem* menuDisplay_ShowOAMotionPlan;
			GtkCheckMenuItem* menuDisplay_ShowMPPMotionPlan;
			GtkCheckMenuItem* menuDisplay_ShowCommandPlan;
			GtkCheckMenuItem* menuDisplay_ShowDynamicObjects;
			GtkCheckMenuItem* menuDisplay_ShowDynamicPoints;
			GtkCheckMenuItem* menuDisplay_ShowAnnotations;
			GtkCheckMenuItem* menuDisplay_ShowLaneMarkings;
			GtkCheckMenuItem* menuDisplay_ShowCollisionRange;
			GtkCheckMenuItem* menuSimulatorShowTruePosition;
			GtkCheckMenuItem* menuSimulator_ShowObjects;
			GtkCheckMenuItem* menuGoals_EditRddfGoals;
			GtkCheckMenuItem* menuMaps_Map;
			GtkCheckMenuItem* menuMaps_MapLevel1;
			GtkCheckMenuItem* menuMaps_OfflineMap;
			GtkCheckMenuItem* menuMaps_Utility;
			GtkCheckMenuItem* menuMaps_Costs;
			GtkCheckMenuItem* menuMaps_Likelihood;
			GtkCheckMenuItem* menuMaps_GlobalLikelihood;
			GtkCheckMenuItem* menuMaps_Lane;
			GtkCheckMenuItem* menuMaps_CompleteMap;
			GtkCheckMenuItem* menuMaps_RemissionMap;
			GtkCheckMenuItem* menuMaps_MovingObjects;
			GtkCheckMenuItem* menuMaps_RoadMap;
			GtkCheckMenuItem* menuSuperimposedMaps_None;
			GtkCheckMenuItem* menuSuperimposedMaps_Map;
			GtkCheckMenuItem* menuSuperimposedMaps_MapLevel1;
			GtkCheckMenuItem* menuSuperimposedMaps_OfflineMap;
			GtkCheckMenuItem* menuSuperimposedMaps_Utility;
			GtkCheckMenuItem* menuSuperimposedMaps_Costs;
			GtkCheckMenuItem* menuSuperimposedMaps_Likelihood;
			GtkCheckMenuItem* menuSuperimposedMaps_GlobalLikelihood;
			GtkCheckMenuItem* menuSuperimposedMaps_Lane;
			GtkCheckMenuItem* menuSuperimposedMaps_CompleteMap;
			GtkCheckMenuItem* menuSuperimposedMaps_RemissionMap;
			GtkCheckMenuItem* menuSuperimposedMaps_MovingObjects;
			GtkCheckMenuItem* menuSuperimposedMaps_RoadMap;
		};

		Controls controls_;

		// **** mouse events

		// **** draw window size
		carmen_navigator_panel_config_t *nav_panel_config;
		carmen_world_point_t *path_vector[PATH_VECTOR_SIZE];
		int path_vector_size[PATH_VECTOR_SIZE];
		GdkColor *path_vector_color[PATH_VECTOR_SIZE];
		GdkColor robot_colour, goal_colour, tree_colour, people_colour, path_colour;

		char *map_path;

		int goal_source;
		int behavior_selector_active;

		carmen_world_point_t fused_odometry_position;
		carmen_world_point_t parking_assistant_goal;

		int	  display_needs_updating;
		double time_of_last_redraw;

		carmen_world_point_t **canditade_path;
		int *candidate_path_size;
		int num_candidate_path;

		carmen_world_point_t *plan_tree_p1;
		carmen_world_point_t *plan_tree_p2;
		int *mask;
		int num_plan_tree_points;

		carmen_list_t *simulator_objects;
		carmen_list_t *people;
		carmen_list_t *moving_objects_list;

		carmen_world_point_t simulator_trueposition;
		double time_of_simulator_update;
		double simulator_hidden;

		double last_simulator_update;
		int update_local_map;
		carmen_navigator_map_t display;

		carmen_world_point_t *path;
		int	  num_path_points;

		carmen_world_point_t *navigator_waypoint_list;
		int waypoint_list_size;

		carmen_world_point_t *navigator_goal_list;
		int goal_list_size;

		carmen_rddf_waypoint *edited_rddf_goal_list;
		int edited_rddf_goal_size;

		carmen_traj_point_t	 robot_traj;
		carmen_world_point_t	 robot;
		carmen_ackerman_traj_point_t	 goal;

		double last_navigator_update;
		GdkColor RedBlueGradient[GRADIENT_COLORS];

		carmen_localize_ackerman_globalpos_message *globalpos;

		carmen_robot_config_t	 	*robot_config;
		carmen_polygon_config_t		*poly_config;
		carmen_navigator_config_t 	*nav_config;

		carmen_world_point_t cursor_pos;
		placement_t placement_status;

		fpointers *queuePoints;
		int global_view;

		carmen_world_point_t	 robot_temp;
		carmen_world_point_t	 goal_temp;

		carmen_world_point_t	 new_person;
		carmen_world_point_t	 new_simulator;

		carmen_world_point_t	 final_goal;

		carmen_localize_ackerman_particle_message particle_msg;
		carmen_localize_ackerman_sensor_message	  sensor_msg;

		int	 is_filming;
		int filming_timeout;

		carmen_navigator_ackerman_plan_message obstacle_avoider_msg;
		carmen_world_point_t *obstacle_avoider_path;
		int obstacle_avoider_path_size;

		carmen_navigator_ackerman_plan_message oa_motion_plan_msg;
		carmen_world_point_t *oa_motion_plan;
		int oa_motion_plan_size;

		carmen_model_predictive_planner_motion_plan_message mpp_motion_plan_msg;
		carmen_world_point_t *mpp_motion_plan;
		int mpp_motion_plan_size;

		carmen_mapper_virtual_laser_message virtual_laser_msg;

		carmen_rddf_annotation_message rddf_annotation_msg;

		carmen_lane_detector_lane_message_t *lane_markings_msg = NULL;

		GdkImage *annotation_image[NUM_RDDF_ANNOTATION_TYPES][NUM_RDDF_ANNOTATION_CODES];

		carmen_robot_ackerman_road_velocity_control_message road_velocity_control;

		carmen_rddf_waypoint* near_rddf_point;
		int near_rddf_point_index;

		bool freeze_status;

		car_panel *car_panel_gl;

		void ConfigureMenu();
		void ConfigureMapViewer();
		void InitializePathVector();
		int get_algorithm_code(char *algorithm_name);
		int get_goal_source_code(char* goal_source_name);
		int get_state_code(char* state_name);
		void do_redraw(void);
		void label_autonomy_button(char *str);
		void change_cursor(GdkColor *fg, GdkColor *bg);
		void execute_decrement_point();
		void world_point_to_global_world_point(carmen_world_point_t *world_point);

		GtkGui(int argc, char *argv[]);
		virtual ~GtkGui();

		Controls *getControls() { return &controls_; }

		void navigator_graphics_initialize(int argc, char **argv, carmen_localize_ackerman_globalpos_message *msg,
						carmen_robot_config_t *robot_conf_param, carmen_polygon_config_t *poly_config_param,
						carmen_navigator_config_t *nav_conf_param, carmen_navigator_panel_config_t *nav_panel_conf_param);

		int navigator_graphics_update_map();
		void navigator_graphics_update_display(carmen_traj_point_p new_robot, carmen_localize_ackerman_globalpos_message *current_globalpos, carmen_ackerman_traj_point_t *new_goal, int autonomous);
		void set_distance_traveled(carmen_point_t robot_pose, double velocity);
		void navigator_graphics_update_goal_list(carmen_ackerman_traj_point_t* goal_list, int size);
		void navigator_graphics_update_waypoint_list(carmen_ackerman_traj_point_t* waypoint_list, int size);
		void navigator_graphics_update_plan(carmen_ackerman_traj_point_p new_plan, int plan_length);
		void navigator_graphics_update_path(carmen_ackerman_traj_point_t* new_path, int path_length, int path_id);
		void navigator_graphics_display_map(carmen_map_t *new_map, carmen_navigator_map_t type);
		void navigator_graphics_set_flags(carmen_navigator_map_t type);

		void navigator_graphics_redraw_superimposed();
		void navigator_graphics_change_map(carmen_map_p new_map);
		void navigator_graphics_update_simulator_truepos(carmen_point_t truepose);
		void navigator_graphics_update_simulator_objects(int num_objects, carmen_traj_point_t *objects_list);
		void navigator_graphics_update_moving_objects(int num_point_clouds, moving_objects_tracking_t *moving_objects_tracking);

		void navigator_graphics_update_plan_to_draw(int path_size, carmen_ackerman_traj_point_t *path);

		void navigator_graphics_update_plan_tree(
				carmen_ackerman_traj_point_p p1,
				carmen_ackerman_traj_point_p p2,
				int *mask,
				int plan_tree_length,
				carmen_ackerman_traj_point_t paths[500][100],
				int path_size[100],
				int num_path);

		void navigator_graphics_update_fused_odometry(carmen_point_t fused_odometry_pose);
		void navigator_graphics_update_behavior_selector_state(carmen_behavior_selector_state_message msg);
		void navigator_graphics_update_traffic_sign_state(carmen_rddf_traffic_sign_message msg);
		void navigator_graphics_update_parking_assistant_goal(carmen_point_t pose);
		void navigator_graphics_reset();
		void navigator_graphics_display_config (char *attribute, int value, char *new_status_message);
		void navigator_graphics_add_ipc_handler(GdkInputFunction handle_ipc);
		void navigator_graphics_start(char *path);

		int placing_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event);
		int placing_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event);
		int placing_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int placing_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int orienting_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event);
		int orienting_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int orienting_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int orienting_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int selecting_final_region_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point);
		int placing_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int orienting_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point);
		int selecting_near_waypoint_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point);
		carmen_rddf_waypoint* find_near_rddf_point(carmen_world_point_t *world_point);
		int select_near_rddf_point(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point);
		void release_near_rddf_point();
		void delete_current_rddf_point();
		void update_edited_rddf_goal_list();
		int original_rdff_list_contains(carmen_world_point_t world_point);

		void add_goal_to_internal_list(carmen_world_point_t goal);
		int received_robot_pose(void);
		void update_point(pointers *reached);
		double x_coord(double x, double y, carmen_world_point_t *offset);
		double y_coord(double x, double y, carmen_world_point_t *offset);

		void draw_parking_assistant_goal(GtkMapViewer *the_map_view);
		void draw_fused_odometry_pose(GtkMapViewer *the_map_view);
		void draw_particles(GtkMapViewer *the_map_view, double pixel_size);
		void draw_gaussians(GtkMapViewer *the_map_view);
		void draw_lasers(GtkMapViewer *the_map_view, double pixel_size);
		void draw_robot(GtkMapViewer *the_map_view);
		void draw_plan_tree(GtkMapViewer *the_map_view, double pixel_size);
		void draw_goal_list(GtkMapViewer	*the_map_view, carmen_world_point_t goal);
		void draw_simulated_robot(GtkMapViewer *the_map_view);
		void draw_simulated_objects(GtkMapViewer *the_map_view);
		void draw_moving_objects(GtkMapViewer *the_map_view);
		void draw_moving_points(GtkMapViewer *the_map_view, double pixel_size);
		void draw_lane_lines(GtkMapViewer *the_map_view, double pixel_size);
		void draw_annotations(GtkMapViewer *the_map_view, double pixel_size);
		void draw_placing_animation(GtkMapViewer *the_map_view);
		void draw_path(carmen_world_point_t *path, int num_path_points, GdkColor path_colour, GdkColor robot_color, GtkMapViewer *the_map_view);
		void draw_road_velocity_control(GtkMapViewer *the_map_view);
		void draw_path_vector(GtkMapViewer *the_map_view);
		void draw_robot_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour);
		void draw_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose);
		void draw_path_color(GtkMapViewer *the_map_view, carmen_world_point_t* path, int size, GdkColor *color);
		void draw_differential_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour);
		void draw_ackerman_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour);
		void draw_differential_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose);
		void draw_ackerman_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose);

		void draw_gl_components();
		void drawAxes();
		void setView();

		void draw_gl_components_car_panel();

		void save_rddf_to_file(char *rddf_filename);
		void load_rddf_from_file(char *rddf_filename);

		void load_annotations_images();

		void navigator_graphics_go_message_received();
		void navigator_graphics_stop_message_received();
	};
}

#endif

