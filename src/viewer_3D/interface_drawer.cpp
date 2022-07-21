#include <carmen/carmen.h>

#include "viewer_3D.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "interface_drawer.h"
#include "Window.h"

struct button
{
	double x, y;
	double width, height;

	int mouse_over;
	int visible;
	int state;
	int code;

	const char* text;
};
typedef struct button button;

struct interface_drawer
{
	button* butt;
	int num_buttons;
};

static void init_buttons(interface_drawer* i_drawer, int window_width, int window_height);
static void handle_mouse_movement(interface_drawer* i_drawer, int x, int y);
static void handle_mouse_left_click(interface_drawer* i_drawer, int x, int y);
static void handle_mouse_right_click(interface_drawer* i_drawer, int x, int y);
static int test_mouse_over_button(button b, int x, int y);
static void draw_button(button b);
static void drawText(float x, float y, const char* msg, int font_help ...);

interface_drawer*
create_interface_drawer(int window_width, int window_height)
{
	interface_drawer* i_drawer = (interface_drawer*) malloc(sizeof(interface_drawer));

	i_drawer->num_buttons = 84;
	i_drawer->butt = (button*) malloc(i_drawer->num_buttons * sizeof(button));

	init_buttons(i_drawer, window_width, window_height);

	return i_drawer;
}


void
destroy_interface_drawer(interface_drawer* i_drawer)
{
	free(i_drawer->butt);
	free(i_drawer);
}


static void
init_buttons(interface_drawer* i_drawer, int window_width, int window_height)
{
	int i;

	double base_x = 80.0 * window_width / 1000.0;
	double base_y = 50.0 * window_height / 600.0;
	double button_width = 100.0 * window_width / 1000.0;
	double button_height = 20.0 * window_height / 600.0;
	double horizontal_space = 120.0 * window_width / 1000.0;

	for (i = 0; i < i_drawer->num_buttons; i++)
	{
		i_drawer->butt[i].code = i;
		i_drawer->butt[i].mouse_over = 0;
		i_drawer->butt[i].state = 0;
		i_drawer->butt[i].text = "\0";

		if (i < 8)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 16)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 24)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 32)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 4 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 40)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 48)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 56)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}
		else if (i < 64)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i < 72)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i < 77)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i < 82)
		{
//			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].x = base_x + (i - 77) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i == 82)
		{
			i_drawer->butt[i].visible = 1;
		}

		else if (i == 83)
		{
			i_drawer->butt[i].visible = 0;
		}

		if (i == 0)
		{
			i_drawer->butt[i].visible = 1;
		}
	}

	i_drawer->butt[0].text =									BUTTON0_TEXT;
	i_drawer->butt[DRAW_CAR_BUTTON_CODE].text =					DRAW_CAR_BUTTON_TEXT;
	i_drawer->butt[DRAW_COLLISION_MARKERS_BUTTON_CODE].text =	DRAW_COLLISION_MARKERS_BUTTON_TEXT;
	i_drawer->butt[DRAW_ODOMETRY_BUTTON_CODE].text =			DRAW_ODOMETRY_BUTTON_TEXT;
	i_drawer->butt[DRAW_XSENS_GPS_BUTTON_CODE].text =			DRAW_XSENS_GPS_BUTTON_TEXT;
	i_drawer->butt[FOLLOW_CAR_BUTTON_CODE].text =				FOLLOW_CAR_BUTTON_TEXT;
	i_drawer->butt[DRAW_GPS_BUTTON_CODE].text =					DRAW_GPS_BUTTON_TEXT;
//	i_drawer->butt[DRAW_MAP_IMAGE_BUTTON_CODE].text =			DRAW_MAP_IMAGE_BUTTON_TEXT;
	i_drawer->butt[WINDOW_SIZES_BUTTON_CODE].text =				WINDOW_SIZES_BUTTON_TEXT;

	i_drawer->butt[VELODYNE_360_BUTTON_CODE].text =			VELODYNE_360_BUTTON_TEXT;
	i_drawer->butt[VELODYNE_VBO_BUTTON_CODE].text =			VELODYNE_VBO_BUTTON_TEXT;
	i_drawer->butt[VELODYNE_BUTTON_CODE].text =				VELODYNE_BUTTON_TEXT;
	i_drawer->butt[DRAW_STEREO_CLOUD_BUTTON_CODE].text =	DRAW_STEREO_CLOUD_BUTTON_TEXT;
	i_drawer->butt[MAPS_BUTTON_CODE].text =					MAPS_BUTTON_TEXT;
	i_drawer->butt[DRAW_ANNOTATION_BUTTON_CODE].text =		DRAW_ANNOTATION_BUTTON_TEXT;
	i_drawer->butt[DRAW_POINTS_BUTTON_CODE].text =			DRAW_POINTS_BUTTON_TEXT;
	i_drawer->butt[DRAW_RAYS_BUTTON_CODE].text =			DRAW_RAYS_BUTTON_TEXT;

	i_drawer->butt[DRAW_XSENS_ORIENTATION_BUTTON_CODE].text =		DRAW_XSENS_ORIENTATION_BUTTON_TEXT;
	i_drawer->butt[DRAW_LOCALIZE_ACKERMAN_BUTTON_CODE].text =		DRAW_LOCALIZE_ACKERMAN_BUTTON_TEXT;
	i_drawer->butt[VELODYNE_INTENSITY_BUTTON_CODE].text =			VELODYNE_INTENSITY_BUTTON_TEXT;
	i_drawer->butt[DRAW_PATH_PLAN_BUTTON_CODE].text =				DRAW_PATH_PLAN_BUTTON_TEXT;
	i_drawer->butt[DRAW_MOTION_PLAN_BUTTON_CODE].text =				DRAW_MOTION_PLAN_BUTTON_TEXT;
	i_drawer->butt[DRAW_OBSTACLE_AVOIDER_PLAN_BUTTON_CODE].text =	DRAW_OBSTACLE_AVOIDER_PLAN_BUTTON_TEXT;
	i_drawer->butt[DRAW_MOVING_OBJECTS_BUTTON_CODE].text =			DRAW_MOVING_OBJECTS_BUTTON_TEXT;
	i_drawer->butt[DRAW_GPS_AXIS_BUTTON_CODE].text =				DRAW_GPS_AXIS_BUTTON_TEXT;

	i_drawer->butt[DRAW_ROBOT_WAYPOINTS_BUTTON_CODE].text =	DRAW_ROBOT_WAYPOINTS_BUTTON_TEXT;
	i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].text =	VELODYNE_REMISSION_BUTTON_TEXT;
	i_drawer->butt[FORCE_VELODYNE_BUTTON_CODE].text =		FORCE_VELODYNE_BUTTON_TEXT;
	i_drawer->butt[SHOW_SYMOTHA_BUTTON_CODE].text =			SHOW_SYMOTHA_BUTTON_TEXT;
	i_drawer->butt[LIDARS_BUTTON_CODE].text =				LIDARS_BUTTON_TEXT;
	i_drawer->butt[SHOW_PATH_PLANS_BUTTON_CODE].text =		SHOW_PATH_PLANS_BUTTON_TEXT;
	i_drawer->butt[SHOW_PLAN_TREE_BUTTON_CODE].text =		SHOW_PLAN_TREE_BUTTON_TEXT;
	i_drawer->butt[DRAW_WAYPOINTS_BUTTON_CODE].text =		DRAW_WAYPOINTS_BUTTON_TEXT;

	//Annotations
	i_drawer->butt[TRAFFIC_LIGHT_BUTTON_CODE].text =		TRAFFIC_LIGHT_BUTTON_TEXT;
	i_drawer->butt[TRAFFIC_SIGNAL_BUTTON_CODE].text =		TRAFFIC_SIGNAL_BUTTON_TEXT;
	i_drawer->butt[PEDESTRIAN_TRACK_BUTTON_CODE].text =		PEDESTRIAN_TRACK_BUTTON_TEXT;
	i_drawer->butt[STOP_BUTTON_CODE].text =					STOP_BUTTON_TEXT;
	i_drawer->butt[BARRIER_BUTTON_CODE].text =				BARRIER_BUTTON_TEXT;
	i_drawer->butt[BUMP_BUTTON_CODE].text =					BUMP_BUTTON_TEXT;
	i_drawer->butt[SPEED_BUTTON_CODE].text =				SPEED_BUTTON_TEXT;
	i_drawer->butt[DELETE_ANNOTATION_BUTTON_CODE].text =	DELETE_ANNOTATION_BUTTON_TEXT;
	//Speed Annotations
	i_drawer->butt[40].text = "0 km/h";
	i_drawer->butt[41].text = "5 km/h";
	i_drawer->butt[42].text = "10 km/h";
	i_drawer->butt[43].text = "15 km/h";
	i_drawer->butt[44].text = "20 km/h";
	i_drawer->butt[45].text = "30 km/h";
	i_drawer->butt[46].text = "40 km/h";
	i_drawer->butt[47].text = "60 km/h";
	//Traffic Signals Annotations
	i_drawer->butt[48].text = "Bump";
	i_drawer->butt[49].text = "20 km/h";
	i_drawer->butt[50].text = "30 km/h";
	i_drawer->butt[51].text = "Empty";
	i_drawer->butt[52].text = "Empty";
	i_drawer->butt[53].text = "Empty";
	i_drawer->butt[54].text = "Empty";
	i_drawer->butt[55].text = "Empty";

	//Lidars Numbers - when i_drawer->butt[28] active shows buttons to variable_scan_message
	i_drawer->butt[56].text = "lidar0";
	i_drawer->butt[57].text = "lidar1";
	i_drawer->butt[58].text = "lidar2";
	i_drawer->butt[59].text = "lidar3";
	i_drawer->butt[60].text = "lidar4";
	i_drawer->butt[61].text = "lidar5";
	i_drawer->butt[62].text = "lidar6";
	i_drawer->butt[63].text = "lidar7";

	i_drawer->butt[64].text = "lidar8";
	i_drawer->butt[65].text = "lidar9";
	i_drawer->butt[66].text = "lidar10";
	i_drawer->butt[67].text = "lidar11";
	i_drawer->butt[68].text = "lidar12";
	i_drawer->butt[69].text = "lidar13";
	i_drawer->butt[70].text = "lidar14";
	i_drawer->butt[71].text = "lidar15";

	i_drawer->butt[72].text = "Map";
	i_drawer->butt[73].text = "Costs Map";
	i_drawer->butt[74].text = "Offline Map";
	i_drawer->butt[75].text = "Remission Map";
	i_drawer->butt[76].text = "Google images";

	i_drawer->butt[77].text = "1920x1080";
	i_drawer->butt[78].text = "1500x920";
	i_drawer->butt[79].text = "1000x600";
	i_drawer->butt[80].text = "800x400";
	i_drawer->butt[81].text = "500x300";

	//Help
	i_drawer->butt[HELP_BUTTON_CODE].text = HELP_BUTTON_TEXT;
	i_drawer->butt[HELP_CODE].text = HELP_TEXT;

}


void
interface_mouse_func(interface_drawer* i_drawer, int type, int button, int x, int y, int window_height)
{
	y = window_height - y;

//	printf("mouse - type: %d, button: %d, x: %d, y: %d\n", type, button, x, y);
	if (type == 0 && button == 0) // Mouse movement
	{
		handle_mouse_movement(i_drawer, x, y);
	}
	else if (type == 5 && button == 1) // Left Click
	{
		handle_mouse_left_click(i_drawer, x, y);
	}
	else if (type == 5 && button == 3) // Right Click
	{
		handle_mouse_right_click(i_drawer, x, y);
	}
}


void
update_buttons_size(interface_drawer* i_drawer, int window_width, int window_height)
{
	int i;

	double base_x = 80.0 * window_width / 1000.0;
	double base_y = 30.0 * window_height / 600.0;
	double button_width = 100.0 * window_width / 1000.0;
	double button_height = 20.0 * window_height / 600.0;
	double horizontal_space = 120.0 * window_width / 1000.0;

	for (i = 0; i < i_drawer->num_buttons; i++)
	{
		if (i < 8)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 16)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 24)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 32)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 4 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 40)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 48)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 56)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 64)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}
		else if (i < 72)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 2 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;
		}

		else if (i < 77)
		{
			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i < 82)
		{
//			i_drawer->butt[i].x = base_x + (i % 8) * horizontal_space;
			i_drawer->butt[i].x = base_x + (i - 77) * horizontal_space;
			i_drawer->butt[i].y = 3 * base_y;
			i_drawer->butt[i].width = button_width;
			i_drawer->butt[i].height = button_height;

			i_drawer->butt[i].visible = 0;
		}

		else if (i == 82)
		{
			i_drawer->butt[i].x = window_width - 60;
			i_drawer->butt[i].y = window_height - 30;
			i_drawer->butt[i].width = 80;
			i_drawer->butt[i].height = 20;
		}

		else if (i == 83)
		{
			i_drawer->butt[i].x = window_width - 170;
			i_drawer->butt[i].y = window_height - 170;
			i_drawer->butt[i].width = 300;
			i_drawer->butt[i].height = 220;
		}
	}
}


static void
handle_mouse_movement(interface_drawer* i_drawer, int x, int y)
{
	int i;
	for (i = 0; i < i_drawer->num_buttons; i++)
	{
		if (i_drawer->butt[i].visible)
		{
			i_drawer->butt[i].mouse_over = test_mouse_over_button(i_drawer->butt[i], x, y);
		}
	}
}

static void
handle_mouse_left_click(interface_drawer* i_drawer, int x, int y)
{
	int i;
	for (i = 0; i < i_drawer->num_buttons; i++)
	{
		if (i_drawer->butt[i].visible)
		{
			if (test_mouse_over_button(i_drawer->butt[i], x, y))
			{
				if (i_drawer->butt[i].code == 0) // option
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					int j;
					for (j = 1; j < 32; j++)
					{
						i_drawer->butt[j].visible = !i_drawer->butt[j].visible;
					}

					if (i_drawer->butt[13].state)
					{
						i_drawer->butt[13].state = !(i_drawer->butt[13].state);
						set_flag_viewer_3D(19, i_drawer->butt[13].state);
						for (j = 32; j < 56; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}
					//if lidars
					if (i_drawer->butt[28].state)
					{
						i_drawer->butt[28].state = !(i_drawer->butt[28].state);
//						set_flag_viewer_3D(19, i_drawer->butt[13].state);
						for (j = 56; j < 72; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}

					// if maps
					if(i_drawer->butt[MAPS_BUTTON_CODE].state)
					{
						i_drawer->butt[MAPS_BUTTON_CODE].state = !(i_drawer->butt[MAPS_BUTTON_CODE].state);
						for(j = 72; j < 77; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}

					// if window sizes
					if(i_drawer->butt[WINDOW_SIZES_BUTTON_CODE].state)
					{
						i_drawer->butt[WINDOW_SIZES_BUTTON_CODE].state = !(i_drawer->butt[WINDOW_SIZES_BUTTON_CODE].state);
						for(j = 77; j < 82; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}
				}
				else if (i_drawer->butt[i].code == DRAW_CAR_BUTTON_CODE) // draw car
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_CAR_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_COLLISION_MARKERS_BUTTON_CODE) // draw collision markers
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_COLLISION_MARKERS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_ODOMETRY_BUTTON_CODE) // draw odometry
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_ODOMETRY_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_XSENS_GPS_BUTTON_CODE) // draw gps xsens
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_XSENS_GPS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == FOLLOW_CAR_BUTTON_CODE) // follow car
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(FOLLOW_CAR_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_GPS_BUTTON_CODE) // draw gps
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_GPS_FLAG_CODE, i_drawer->butt[i].state);
				}

				else if (i_drawer->butt[i].code == VELODYNE_360_BUTTON_CODE) // velodyne 360
				{
					if (i_drawer->butt[i].state == 0)
						i_drawer->butt[i].state = 3;
					else
						i_drawer->butt[i].state = 0;

					if (i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state)
						i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state = 0;
					set_flag_viewer_3D(DRAW_VELODYNE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == VELODYNE_VBO_BUTTON_CODE) // velodyne VBO
				{
					if (i_drawer->butt[i].state == 0)
						i_drawer->butt[i].state = 2;
					else
						i_drawer->butt[i].state = 0;

					if (i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state)
						i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state = 0;
					set_flag_viewer_3D(DRAW_VELODYNE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == VELODYNE_BUTTON_CODE) // velodyne
				{
					if (i_drawer->butt[i].state == 0)
						i_drawer->butt[i].state = 1;
					else
						i_drawer->butt[i].state = 0;

					if (i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state)
						i_drawer->butt[VELODYNE_REMISSION_BUTTON_CODE].state = 0;
					set_flag_viewer_3D(DRAW_VELODYNE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_STEREO_CLOUD_BUTTON_CODE) // stereo variable velodyne
				{
//                    if (i_drawer->butt[i].state == 0)
//                        i_drawer->butt[i].state = 4;
//                    else
//                        i_drawer->butt[i].state = 0;

					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_STEREO_CLOUD_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == MAPS_BUTTON_CODE) // Maps
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					for (int j = 72; j < 77; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 1; j < 32; j++)
					{
						i_drawer->butt[j].visible = !i_drawer->butt[i].state;
					}

//					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, i_drawer->butt[i].state);
//					set_flag_viewer_3D(ZERO_Z_FLAG_CODE, i_drawer->butt[i].state);
				}

				else if (i_drawer->butt[i].code == WINDOW_SIZES_BUTTON_CODE) // Window sizes
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					for (int j = 77; j < 82; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 1; j < 32; j++)
					{
						i_drawer->butt[j].visible = !i_drawer->butt[i].state;
					}

//					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, i_drawer->butt[i].state);
//					set_flag_viewer_3D(ZERO_Z_FLAG_CODE, i_drawer->butt[i].state);
				}

				else if (i_drawer->butt[i].code == DRAW_ANNOTATION_BUTTON_CODE) // Annotation
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);
					set_flag_viewer_3D(DRAW_ANNOTATION_FLAG_CODE, i_drawer->butt[i].state);

					if (i_drawer->butt[38].state == 1)
					{
						for (int j = 40; j < 48; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}
					if (i_drawer->butt[37].state == 1)
					{
						for (int j = 48; j < 56; j++)
						{
							i_drawer->butt[j].visible = 0;
						}
					}

					for (int j = 32; j < 40; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 1; j < 32; j++)
					{
						i_drawer->butt[j].visible = !i_drawer->butt[i].state;
					}

				}
				else if (i_drawer->butt[i].code == DRAW_POINTS_BUTTON_CODE) // SICK
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_POINTS_FLAG_CODE, i_drawer->butt[i].state);
					set_flag_viewer_3D(ZERO_Z_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_RAYS_BUTTON_CODE) // SICK Rays
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_RAYS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_XSENS_ORIENTATION_BUTTON_CODE) // XSENS Orientation
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_XSENS_ORIENTATION_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_LOCALIZE_ACKERMAN_BUTTON_CODE) // Localize Ackerman
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_LOCALIZE_ACKERMAN_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == VELODYNE_INTENSITY_BUTTON_CODE) // Velodyne Intensity
				{
					if (i_drawer->butt[i].state == 0)
						i_drawer->butt[i].state = 5;
					else
						i_drawer->butt[i].state = 0;

					set_flag_viewer_3D(DRAW_VELODYNE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_PATH_PLAN_BUTTON_CODE) // Path Plan
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_PATH_PLAN_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_MOTION_PLAN_BUTTON_CODE) // Trajectory Plan
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MOTION_PLAN_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_OBSTACLE_AVOIDER_PLAN_BUTTON_CODE) // Obst. Av. Plan
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_OBSTACLE_AVOIDER_PLAN_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_MOVING_OBJECTS_BUTTON_CODE) // Moving Objects
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MOVING_OBJECTS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_GPS_AXIS_BUTTON_CODE) // GPS Axis
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_GPS_AXIS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_ROBOT_WAYPOINTS_BUTTON_CODE) // Robot Waypoints
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_ROBOT_WAYPOINTS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == VELODYNE_REMISSION_BUTTON_CODE) // Remission (works on pointcloud from Velodyne VBO)
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(VELODYNE_REMISSION_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == FORCE_VELODYNE_BUTTON_CODE) // Force Velodyne
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(FORCE_VELODYNE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == SHOW_SYMOTHA_BUTTON_CODE) // Show Symotha
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SHOW_SYMOTHA_FLAG_CODE, i_drawer->butt[i].state);
				}
				//TODO @vinicius lidars
				else if (i_drawer->butt[i].code == LIDARS_BUTTON_CODE) // Lidars
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);
//					set_flag_viewer_3D(19, i_drawer->butt[i].state);

//					if (i_drawer->butt[38].state == 1)
//					{
//						for (int j = 40; j < 48; j++)
//						{
//							i_drawer->butt[j].visible = 0;
//						}
//					}
//					if (i_drawer->butt[37].state == 1)
//					{
//						for (int j = 48; j < 56; j++)
//						{
//							i_drawer->butt[j].visible = 0;
//						}
//					}

					for (int j = 56; j < 72; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 1; j < 32; j++)
					{
						i_drawer->butt[j].visible = !i_drawer->butt[i].state;
					}

//					for (int j = 32; j < 56; j++)
//					{
//						i_drawer->butt[j].visible = 0;
//					}

				}
				else if (i_drawer->butt[i].code == SHOW_PATH_PLANS_BUTTON_CODE) // Show Path Plans
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SHOW_PATH_PLANS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == SHOW_PLAN_TREE_BUTTON_CODE) // Plan Tree
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SHOW_PLAN_TREE_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == DRAW_WAYPOINTS_BUTTON_CODE) // Waypoints
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_WAYPOINTS_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == TRAFFIC_LIGHT_BUTTON_CODE) // Traffic Light
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(TRAFFIC_LIGHT_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == TRAFFIC_SIGNAL_BUTTON_CODE) // Traffic Signal
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					for (int j = 48; j < 56; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 40; j < 48; j++)
					{
						i_drawer->butt[j].visible = 0;
					}
				}
				else if (i_drawer->butt[i].code == PEDESTRIAN_TRACK_BUTTON_CODE) // Pedestrian Track
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(PEDESTRIAN_TRACK_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == STOP_BUTTON_CODE) // Stop
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(STOP_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == BARRIER_BUTTON_CODE) // Barrier
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(BARRIER_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == BUMP_BUTTON_CODE) // Bump
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(BUMP_FLAG_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == SPEED_BUTTON_CODE) // Speed
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					for (int j = 40; j < 48; j++)
					{
						i_drawer->butt[j].visible = i_drawer->butt[i].state;
					}
					for (int j = 48; j < 56; j++)
					{
						i_drawer->butt[j].visible = 0;
					}
				}
				else if (i_drawer->butt[i].code == DELETE_ANNOTATION_BUTTON_CODE) // Delete Annotation
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DELETE_ANNOTATION_CODE, i_drawer->butt[i].state);
				}
				else if (i_drawer->butt[i].code == 40) // Speed 0
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 0);
				}
				else if (i_drawer->butt[i].code == 41) // Speed 5
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 5);
				}
				else if (i_drawer->butt[i].code == 42) // Speed 10
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 10);
				}
				else if (i_drawer->butt[i].code == 43) // Speed 15
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 15);
				}
				else if (i_drawer->butt[i].code == 44) // Speed 20
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 20);
				}
				else if (i_drawer->butt[i].code == 45) // Speed 30
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 30);
				}
				else if (i_drawer->butt[i].code == 46) // Speed 40
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 40);
				}
				else if (i_drawer->butt[i].code == 47) // Speed 60
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(SPEED_CODE, 60);
				}
				else if (i_drawer->butt[i].code == 48) // Bump Traffic Signal
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(TRAFFIC_SIGN_CODE, 0);
				}
				else if (i_drawer->butt[i].code == 49) // Speed 20 Traffic Signal
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(TRAFFIC_SIGN_CODE, 20);
				}
				else if (i_drawer->butt[i].code == 50) // Speed 30 Traffic Signal
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(TRAFFIC_SIGN_CODE, 30);
				}
				else if (i_drawer->butt[i].code > 55 && i_drawer->butt[i].code < 72) // Lidar variable_scan active
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);
					int lidar_number = i_drawer->butt[i].code - 56;
					set_flag_viewer_3D(DRAW_LIDAR_FLAG_CODE, lidar_number);
				}

				else if (i_drawer->butt[i].code == 72) // Map
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, 0);
				}

				else if (i_drawer->butt[i].code == 73) // Costs Map
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, 1);
				}

				else if (i_drawer->butt[i].code == 74) // Offline Map
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, 2);
				}

				else if (i_drawer->butt[i].code == 75) // Remission Map
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, 3);
				}

				else if (i_drawer->butt[i].code == 76)	// Google images
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(DRAW_MAP_FLAG_CODE, 4);
				}

				else if (i_drawer->butt[i].code == 77)	// 1920x1080
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(WINDOW_SIZES_FLAG_CODE, 1);
				}

				else if (i_drawer->butt[i].code == 78)	// 1500x920
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(WINDOW_SIZES_FLAG_CODE, 2);
				}

				else if (i_drawer->butt[i].code == 79)	// 1000x600
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(WINDOW_SIZES_FLAG_CODE, 3);
				}

				else if (i_drawer->butt[i].code == 80)	// 800x480
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(WINDOW_SIZES_FLAG_CODE, 4);
				}

				else if (i_drawer->butt[i].code == 81)	// 500x300
				{
					i_drawer->butt[i].state = !(i_drawer->butt[i].state);

					set_flag_viewer_3D(WINDOW_SIZES_FLAG_CODE, 5);
				}

				else if (i_drawer->butt[i].code == 82) // Help
				{
					i_drawer->butt[HELP_CODE].visible = !i_drawer->butt[HELP_CODE].visible;
				}
			}
		}
	}
}


static void
handle_mouse_right_click(interface_drawer* i_drawer, int x, int y)
{
	i_drawer = i_drawer;
	x = x;
	y = y;
}


static int
test_mouse_over_button(button b, int x, int y)
{
	if (x > b.x - b.width / 2 && x < b.x + b.width / 2 && y > b.y - b.height / 2 && y < b.y + b.height / 2)
	{
		return 1;
	}

	return 0;
}


void
draw_interface(interface_drawer * i_drawer, int window_width, int window_height)
{
	glPushMatrix();

	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, window_width, 0, window_height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int i;
	for (i = 0; i < i_drawer->num_buttons; i++)
	{
		if (i_drawer->butt[i].visible)
		{
			draw_button(i_drawer->butt[i]);
		}
	}

	glPopMatrix();
}


static void
draw_button(button b)
{
	glPushMatrix();

	if (b.mouse_over && b.code != HELP_CODE)
	{
		glColor3d(1.0, 1.0, 0.0);
	}
	else
	{
		glColor3d(1.0, 1.0, 1.0);
	}

	glTranslated(b.x, b.y, 0.0);

	glBegin(GL_QUADS);

	glVertex3d(-b.width / 2, -b.height / 2, 0.0);
	glVertex3d(b.width / 2, -b.height / 2, 0.0);
	glVertex3d(b.width / 2, b.height / 2, 0.0);
	glVertex3d(-b.width / 2, b.height / 2, 0.0);

	glEnd();

	glColor3d(0.0, 0.0, 0.0);

	int fontHelp;

	if(b.code != 83){
		fontHelp = 0;
		drawText(-b.width / 2 + 5, -5, b.text, fontHelp);
	}
	else
	{
		fontHelp = 1;
		drawText(-b.width/2 + 5, 60, b.text, fontHelp);
	}

	glPopMatrix();
}


static void
drawText(float x, float y, const char* msg, int font_help, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, msg);
	vsprintf(buf, msg, args);
	va_end(args);

	int l, i;

	XWindowAttributes attr;
	attr = get_window_atrr();

	l = strlen(buf);
	glRasterPos2f(x, y);
	for (i = 0; i < l; i++)
	{
		if(buf[i] == '$')
		{
			y -= 15;
			glRasterPos2f(x, y);
	    }

		else
	    {
			if(font_help){
				glutBitmapCharacter(GLUT_BITMAP_9_BY_15, buf[i]);
			} else {
				if(attr.width > 980 && attr.height > 500)
					glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, buf[i]);
				else
					glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, buf[i]);
			}
	    }
	}
}
