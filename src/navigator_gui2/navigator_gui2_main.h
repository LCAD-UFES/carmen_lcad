#ifndef NAVIGATOR_GUI2_MAIN_H_
#define NAVIGATOR_GUI2_MAIN_H_

#include <carmen/carmen_graphics.h>
#include <carmen/behavior_selector_messages.h>

typedef struct
{
	double initial_map_zoom;
	int	   track_robot;
	int    draw_path;
	int	   draw_waypoints;
	int	   draw_robot_waypoints;
	int	   show_particles;
	int    show_fused_odometry;
	int	   show_gaussians;
	int	   show_lasers;
	int	   show_oa_motion_plan;
	int	   show_mpp_motion_plan;
	int	   show_command_plan;
	int	   show_dynamic_objects;
	int	   show_dynamic_points;
	int	   show_annotations;
	int	   show_lane_markings;
	int	   show_collision_range;
	int	   show_simulator_objects;
	int	   show_true_pos;
	int	   show_tracked_objects;
	int    edit_rddf_goals;
	int	   use_ackerman;
	int	   use_exploration;
	char   *map;
	char   *superimposed_map;
} carmen_navigator_panel_config_t;

void navigator_update_robot(carmen_world_point_p robot);
void navigator_set_goal(double x, double y, double theta);
void navigator_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_state_t state);
void navigator_set_goal_by_place(carmen_place_p place);
void navigator_stop_moving(void);
void navigator_start_moving(void);
void navigator_get_map(carmen_navigator_map_t New_Display, int is_superimposed);
carmen_map_t* navigator_get_offline_map_pointer();
carmen_map_t* navigator_get_road_map_pointer();
carmen_map_t* navigator_get_complete_map_map_pointer();
void get_initial_map(void);


#endif /* NAVIGATOR_GUI2_MAIN_H_ */
