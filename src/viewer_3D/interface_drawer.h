#ifndef INTERFACE_DRAWER_H_
#define INTERFACE_DRAWER_H_

// Se quiser mudar a posição de um botão, só é necessário alterar seu BUTTON_CODE. Se quiser mudar o texto escrito nele, mude o BUTTON_TEXT

#define DRAW_CAR_BUTTON_CODE					1
#define DRAW_COLLISION_MARKERS_BUTTON_CODE		2
#define DRAW_ODOMETRY_BUTTON_CODE				3
#define DRAW_XSENS_GPS_BUTTON_CODE				4
#define FOLLOW_CAR_BUTTON_CODE					5
#define DRAW_GPS_BUTTON_CODE					6
#define DRAW_MAP_IMAGE_BUTTON_CODE				7
#define VELODYNE_360_BUTTON_CODE				8
#define VELODYNE_VBO_BUTTON_CODE				9
#define VELODYNE_BUTTON_CODE					10
#define DRAW_STEREO_CLOUD_BUTTON_CODE			11
#define MAPS_BUTTON_CODE						12
#define DRAW_ANNOTATION_BUTTON_CODE				13
#define DRAW_POINTS_BUTTON_CODE					14
#define DRAW_RAYS_BUTTON_CODE					15
#define DRAW_XSENS_ORIENTATION_BUTTON_CODE		16
#define DRAW_LOCALIZE_ACKERMAN_BUTTON_CODE		17
#define VELODYNE_INTENSITY_BUTTON_CODE			18
#define DRAW_PATH_PLAN_BUTTON_CODE				19
#define DRAW_MOTION_PLAN_BUTTON_CODE			20
#define DRAW_OBSTACLE_AVOIDER_PLAN_BUTTON_CODE	21
#define DRAW_MOVING_OBJECTS_BUTTON_CODE			22
#define DRAW_GPS_AXIS_BUTTON_CODE				23
#define DRAW_ROBOT_WAYPOINTS_BUTTON_CODE		24
#define VELODYNE_REMISSION_BUTTON_CODE			25
#define FORCE_VELODYNE_BUTTON_CODE				26
#define SHOW_SYMOTHA_BUTTON_CODE				27
#define LIDARS_BUTTON_CODE						28
#define SHOW_PATH_PLANS_BUTTON_CODE				29
#define SHOW_PLAN_TREE_BUTTON_CODE				30
#define DRAW_WAYPOINTS_BUTTON_CODE				31
#define TRAFFIC_LIGHT_BUTTON_CODE				32
#define TRAFFIC_SIGNAL_BUTTON_CODE				33
#define PEDESTRIAN_TRACK_BUTTON_CODE			34
#define STOP_BUTTON_CODE						35
#define BARRIER_BUTTON_CODE						36
#define BUMP_BUTTON_CODE						37
#define SPEED_BUTTON_CODE						38
#define DELETE_ANNOTATION_BUTTON_CODE			39
#define HELP_BUTTON_CODE						76
#define HELP_CODE							    77

#define BUTTON0_TEXT							"Options"
#define DRAW_CAR_BUTTON_TEXT					"Car"
#define DRAW_COLLISION_MARKERS_BUTTON_TEXT		"Collision Model"
#define DRAW_ODOMETRY_BUTTON_TEXT				"Fused Odometry"
#define DRAW_XSENS_GPS_BUTTON_TEXT				"GPS XSENS"
#define FOLLOW_CAR_BUTTON_TEXT					"Follow Car"
#define DRAW_GPS_BUTTON_TEXT					"GPS"
#define DRAW_MAP_IMAGE_BUTTON_TEXT				"Google Image"
#define VELODYNE_360_BUTTON_TEXT				"Velodyne 360"
#define VELODYNE_VBO_BUTTON_TEXT				"Velodyne VBO"
#define VELODYNE_BUTTON_TEXT					"Velodyne"
#define DRAW_STEREO_CLOUD_BUTTON_TEXT			"Stereo Velodyne"
#define MAPS_BUTTON_TEXT						"Maps"
#define DRAW_ANNOTATION_BUTTON_TEXT				"Annotation"
#define DRAW_POINTS_BUTTON_TEXT					"SICK"
#define DRAW_RAYS_BUTTON_TEXT					"Show Rays"
#define DRAW_XSENS_ORIENTATION_BUTTON_TEXT		"XSENS Axis"
#define DRAW_LOCALIZE_ACKERMAN_BUTTON_TEXT		"Global Pos"
#define VELODYNE_INTENSITY_BUTTON_TEXT			"Intensity Vldyn"
#define DRAW_PATH_PLAN_BUTTON_TEXT				"Path Plan"
#define DRAW_MOTION_PLAN_BUTTON_TEXT			"Motion Plan"
#define DRAW_OBSTACLE_AVOIDER_PLAN_BUTTON_TEXT	"Obst. Av. Plan"
#define DRAW_MOVING_OBJECTS_BUTTON_TEXT			"Moving Objects"
#define DRAW_GPS_AXIS_BUTTON_TEXT				"GPS Axis"
#define DRAW_ROBOT_WAYPOINTS_BUTTON_TEXT		"Robot Waypoints"
#define VELODYNE_REMISSION_BUTTON_TEXT			"Vldyn Remission"
#define FORCE_VELODYNE_BUTTON_TEXT				"Force Velodyne"
#define SHOW_SYMOTHA_BUTTON_TEXT				"Show Symotha"
#define LIDARS_BUTTON_TEXT						"Lidars"
#define SHOW_PATH_PLANS_BUTTON_TEXT				"Show Path Plans"
#define SHOW_PLAN_TREE_BUTTON_TEXT				"Plan Tree"
#define DRAW_WAYPOINTS_BUTTON_TEXT				"Waypoints"
#define TRAFFIC_LIGHT_BUTTON_TEXT				"Traffic Light"
#define TRAFFIC_SIGNAL_BUTTON_TEXT				"Traffic Signal"
#define PEDESTRIAN_TRACK_BUTTON_TEXT			"Pedestrian Track"
#define STOP_BUTTON_TEXT						"Stop"
#define BARRIER_BUTTON_TEXT						"Barrier"
#define BUMP_BUTTON_TEXT						"Bump"
#define SPEED_BUTTON_TEXT						"Speed"
#define DELETE_ANNOTATION_BUTTON_TEXT			"Delete"
#define HELP_BUTTON_TEXT						"Help"
#define HELP_TEXT							    "Camera Views:$$7: Sattelite View (Navigator $GUI View)$8: Sattelite View (Fixed on car)$9: Back View (Follow Robot)$Arrow Keys: Camera Moviment$Mouse: Camera Orientation$Mouse Scroll: Zoom$Space: Unlock and Lock Cam$0: PrintScreen (Default folder: $ ~/Pictures)"

#define DRAW_CAR_FLAG_CODE						4
#define DRAW_COLLISION_MARKERS_FLAG_CODE		0
#define DRAW_ODOMETRY_FLAG_CODE					9
#define DRAW_XSENS_GPS_FLAG_CODE				10
#define FOLLOW_CAR_FLAG_CODE					11
#define DRAW_GPS_FLAG_CODE						8
#define DRAW_MAP_IMAGE_FLAG_CODE				6
#define DRAW_VELODYNE_FLAG_CODE					2
#define DRAW_STEREO_CLOUD_FLAG_CODE				3
#define DRAW_MAP_FLAG_CODE						12
#define DRAW_ANNOTATION_FLAG_CODE				19
#define DRAW_POINTS_FLAG_CODE					1
#define DRAW_RAYS_FLAG_CODE						5
#define DRAW_XSENS_ORIENTATION_FLAG_CODE		15
#define DRAW_LOCALIZE_ACKERMAN_FLAG_CODE		16
#define DRAW_PATH_PLAN_FLAG_CODE				14
#define DRAW_MOTION_PLAN_FLAG_CODE				17
#define DRAW_OBSTACLE_AVOIDER_PLAN_FLAG_CODE	18
#define DRAW_MOVING_OBJECTS_FLAG_CODE			28
#define DRAW_GPS_AXIS_FLAG_CODE					29
#define DRAW_ROBOT_WAYPOINTS_FLAG_CODE			30
#define VELODYNE_REMISSION_FLAG_CODE			31
#define FORCE_VELODYNE_FLAG_CODE				32
#define SHOW_SYMOTHA_FLAG_CODE					33
#define SHOW_PATH_PLANS_FLAG_CODE				35
#define SHOW_PLAN_TREE_FLAG_CODE				36
#define DRAW_WAYPOINTS_FLAG_CODE				37
#define TRAFFIC_LIGHT_FLAG_CODE					20
#define PEDESTRIAN_TRACK_FLAG_CODE				22
#define STOP_FLAG_CODE							23
#define BARRIER_FLAG_CODE						24
#define BUMP_FLAG_CODE							25
#define DELETE_ANNOTATION_CODE					27
#define SPEED_CODE								26
#define TRAFFIC_SIGN_CODE						21
#define DRAW_LIDAR_FLAG_CODE					34
#define ZERO_Z_FLAG_CODE						13
#define WEIGHT_TYPE_FLAG_CODE					7

typedef struct interface_drawer interface_drawer;

interface_drawer* create_interface_drawer(int window_width, int window_height);
void destroy_interface_drawer(interface_drawer* i_drawer);
void interface_mouse_func(interface_drawer* i_drawer, int type, int button, int x, int y, int window_height);
void draw_interface(interface_drawer* i_drawer, int window_width, int window_height);
void update_buttons_size(interface_drawer* i_drawer, int window_width, int window_height);

#endif
