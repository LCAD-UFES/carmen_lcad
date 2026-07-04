
#ifndef GLOBAL_PLANNING_INTERFACE_H_
#define GLOBAL_PLANNING_INTERFACE_H_

#include <carmen/global_planning_messages.h>
#include <carmen/fused_odometry_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif
	/*** mpp/Model Predictive ***/

	void
	carmen_model_predictive_planner_define_motion_plan_message();
	void
	carmen_model_predictive_planner_define_all();
	void
	carmen_model_predictive_planner_subscribe_motion_plan_message(carmen_model_predictive_planner_motion_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_model_predictive_planner_unsubscribe_motion_plan_message(carmen_handler_t handler);
	void
	carmen_model_predictive_planner_publish_motion_plan_message(carmen_robot_and_trailers_traj_point_t *plan, int plan_length);

	/*** Obstacle Avoider ***/

	void
	carmen_robot_ackerman_subscribe_road_velocity_control_message(carmen_robot_ackerman_road_velocity_control_message *msg, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(carmen_obstacle_avoider_robot_will_hit_obstacle_message *msg, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_obstacle_avoider_publish_robot_hit_obstacle_message(int robot_hit_obstacle);

	/*** Route Planner Route ***/

	void
	carmen_route_planner_subscribe_backwards_annotations(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_publish_backwards_annotations(carmen_rddf_annotation_message *message);

	void
	carmen_route_planner_unsubscribe_destination_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_destination_message(char *destination, carmen_point_t destination_point);

	void
	carmen_route_planner_unsubscribe_pallet_and_destination_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_pallet_and_destination(char *pallet, carmen_point_t pallet_point, char *destination, carmen_point_t destination_point);

	void
	carmen_route_planner_subscribe_road_network_message(carmen_route_planner_road_network_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_road_network_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_road_network_message(carmen_route_planner_road_network_message *route_planner_road_network_message);

	void
	carmen_route_planner_subscribe_locked_roads_message(carmen_route_planner_locked_roads_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_locked_roads_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_locked_roads_message(int number_of_locked_node_ids, int *locked_graph_node_ids, int status, char *graph_id);

	void
	carmen_route_planner_unsubscribe_predefined_route_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_predefined_route_message(char *predefined_route, int code);

	void
	carmen_route_planner_subscribe_route_status_change_message(carmen_route_planner_route_status_change_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_route_status_change_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_route_status_change_message(int route_id, int status);

	void
	carmen_route_planner_subscribe_route_reload_message(carmen_route_planner_route_reload_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_route_reload_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_route_reload_message(char *road_network_id);

	void
	carmen_route_planner_subscribe_route_list_request_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_route_list_request_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_route_list_request_message(carmen_position_t center, double range);

	void
	carmen_route_planner_subscribe_route_list_response_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_route_list_response_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_route_list_response_message(carmen_position_t center, double range, int number_of_routes, route_t *routes);

	void
	carmen_route_planner_subscribe_node_ids_ahead_request_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_node_ids_ahead_request_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_node_ids_ahead_request_message(carmen_point_t initial_point, double meters_ahead);

	void
	carmen_route_planner_subscribe_node_ids_ahead_response_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_route_planner_unsubscribe_node_ids_ahead_response_message(carmen_handler_t handler);
	void
	carmen_route_planner_publish_node_ids_ahead_response_message(carmen_point_t initial_point, double meters_ahead, int num_of_node_ids_ahead, int *node_ids_ahead);

	/*** Route Planner Annotation ***/

	void
	carmen_rddf_subscribe_dynamic_annotation_message(carmen_rddf_dynamic_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_unsubscribe_dynamic_annotation_message(carmen_handler_t handler);

	void
	carmen_rddf_subscribe_waypoints_around_end_point_message(carmen_rddf_waypoints_around_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_unsubscribe_waypoints_around_end_point_message(carmen_handler_t handler);
	void
	carmen_rddf_publish_waypoints_around_end_point_message(carmen_robot_and_trailers_traj_point_t *poses_around_end_point, int num_poses);

	void
	carmen_rddf_subscribe_traffic_sign_message(carmen_rddf_traffic_sign_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_publish_traffic_sign_message(int traffic_sign_state, double traffic_sign_data);

	void
	carmen_rddf_subscribe_road_profile_message(carmen_rddf_road_profile_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_publish_road_profile_message(carmen_robot_and_trailers_traj_point_t *poses_ahead, carmen_robot_and_trailers_traj_point_t *poses_back, int num_poses, int num_poses_back, int *annotations, int *annotations_codes);

	void
	carmen_rddf_subscribe_end_point_message(carmen_rddf_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_publish_end_point_message(int number_of_poses_considered_near_endpoint, carmen_robot_and_trailers_pose_t point);

	void
	carmen_rddf_subscribe_add_annotation_message(carmen_rddf_add_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_unsubscribe_add_annotation_message(carmen_handler_t handler);

	void
	carmen_rddf_subscribe_update_annotation_message(carmen_rddf_update_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void
	carmen_rddf_unsubscribe_update_annotation_message(carmen_handler_t handler);
	void
	carmen_rddf_publish_update_annotation_message(crud_t action, carmen_annotation_t old_annotation, carmen_annotation_t new_annotation);

	void
	carmen_rddf_subscribe_annotation_message(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_subscribe_change_weight_node_message(carmen_route_planner_change_weight_node_message *msg, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_publish_change_weight_node_message(carmen_route_planner_change_weight_node_message *msg);

	//void
	//carmen_release_annotation_subscribe_message(carmen_release_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	//void
	//carmen_release_annotation_publish_message(carmen_release_annotation_message *message);

	//void
	//carmen_offroad_activate_subscribe_message(carmen_offroad_activate_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	//void
	//carmen_offroad_activate_publish_message(carmen_offroad_activate_message message);

#ifdef __cplusplus
}
#endif

#endif
