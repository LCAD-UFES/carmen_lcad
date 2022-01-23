#include <locale.h>
#include "navigator_gui2_main.h"
#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/mapper_interface.h>
#include <prob_map.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/map_server_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/lane_detector_interface.h>
#include <carmen/model_predictive_planner_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>

#include <carmen/navigator_gui2_interface.h>
#include <carmen/parking_assistant_interface.h>
#include <carmen/route_planner_interface.h>
#include <carmen/offroad_planner_interface.h>

#include <carmen/carmen_graphics.h>
#include <gtk_gui.h>

#ifdef USE_DOT
#include <carmen/dot.h>
#include <carmen/dot_messages.h>
#include <carmen/dot_interface.h>
#endif

static carmen_robot_config_t	 	robot_config;
static carmen_semi_trailer_config_t semi_trailer_config;
static carmen_polygon_config_t		robot_poly_config;
static carmen_polygon_config_t		semi_trailer_poly_config;
static carmen_navigator_config_t nav_config;
static carmen_navigator_panel_config_t nav_panel_config;
static carmen_navigator_map_t map_type = CARMEN_NAVIGATOR_MAP_v;
static carmen_navigator_map_t superimposedmap_type = CARMEN_NONE_v;
static carmen_map_p online_map = NULL, map_level1 = NULL, cost_map = NULL, offline_map = NULL, likelihood_map = NULL, global_likelihood_map = NULL,
		complete_map = NULL, moving_objects_map = NULL, lane_map = NULL, remission_map = NULL, road_map = NULL, offline_map_tmp = NULL;
carmen_localize_ackerman_map_t localize_all_maps;
int first_localize_map_message_received = 1;
static double last_navigator_status = 0.0;
static int	  is_graphics_up = 0;
int height_max_level = 0;

static double last_v = 0, last_phi = 0;
static carmen_robot_and_trailer_traj_point_t last_goal;
static int goal_set = 0, autonomous = 0;

static char *map_path = NULL;
char *annotation_path = NULL;
int autonomous_record_screen = 0;

int publish_map_view = 0;
double publish_map_view_interval = 0.5;

static carmen_point_t localize_std;
static View::GtkGui *gui;

int record_screen;
int use_glade_with_annotations = 0;
int use_route_planner_in_graph_mode;

char place_of_interest[2048];
char previous_place_of_interest[2048];

char predefined_route[2048];
int predefined_route_code;

int window_width  = -1;
int window_height = -1;
int window_x  = -1;
int window_y = -1;

std::vector <carmen_annotation_t> place_of_interest_list;
std::vector <carmen_annotation_t> predefined_route_list;

static int argc_global;
static char **argv_global;

static void
navigator_get_empty_map()
{
	superimposedmap_type = CARMEN_NONE_v;
	carmen_map_interface_set_superimposed_map(NULL);
}


static void
navigator_get_specific_map(int is_superimposed, carmen_map_t *specific_map, carmen_navigator_map_t type)
{
	if (!is_superimposed)
		map_type = type;
	else
		superimposedmap_type = type;
	gui->navigator_graphics_set_flags(type);

	if ((specific_map == NULL) || (specific_map->complete_map == NULL))
		return;

	if (!is_superimposed)
		gui->navigator_graphics_display_map(specific_map, type);
	else
		carmen_map_interface_set_superimposed_map(specific_map);
}


carmen_map_t *
navigator_get_complete_map(int is_superimposed)
{
	if (is_superimposed)
		return (NULL);

	if (complete_map == NULL)
	{
		complete_map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		int result = carmen_grid_mapping_read_complete_map(map_path, complete_map);
		if (result == -1)
		{
			free(complete_map);
			complete_map = NULL;
			return (NULL);
		}
	}

	if (map_type != CARMEN_COMPLETE_MAP_v)
	{
		gui->navigator_graphics_display_map(complete_map, CARMEN_COMPLETE_MAP_v);
		map_type = CARMEN_COMPLETE_MAP_v;
	}

	return (complete_map);
}


void
navigator_get_map(carmen_navigator_map_t type, int is_superimposed)
{
	switch (type)
	{
	case CARMEN_NONE_v:
		navigator_get_empty_map();
		gui->nav_panel_config->map = (char *) "None";
		break;
	case CARMEN_LOCALIZE_LMAP_v:
		navigator_get_specific_map(is_superimposed, likelihood_map, CARMEN_LOCALIZE_LMAP_v);
		gui->nav_panel_config->map = (char *) "Likelihood";
		break;
	case CARMEN_LOCALIZE_GMAP_v:
		navigator_get_specific_map(is_superimposed, global_likelihood_map, CARMEN_LOCALIZE_GMAP_v);
		gui->nav_panel_config->map = (char *) "Global Likelihood";
		break;
	case CARMEN_LANE_MAP_v:
		navigator_get_specific_map(is_superimposed, lane_map, CARMEN_LANE_MAP_v);
		gui->nav_panel_config->map = (char *) "Lane";
		break;
	case CARMEN_COST_MAP_v:
		navigator_get_specific_map(is_superimposed, cost_map, CARMEN_COST_MAP_v);
		gui->nav_panel_config->map = (char *) "Costs";
		break;
	case CARMEN_NAVIGATOR_MAP_v:
		navigator_get_specific_map(is_superimposed, online_map, CARMEN_NAVIGATOR_MAP_v);
		gui->nav_panel_config->map = (char *) "Map";
		break;
	case CARMEN_NAVIGATOR_MAP_LEVEL1_v:
		navigator_get_specific_map(is_superimposed, map_level1, CARMEN_NAVIGATOR_MAP_LEVEL1_v);
		gui->nav_panel_config->map = (char *) "Map Level1";
		break;
	case CARMEN_OFFLINE_MAP_v:
		navigator_get_specific_map(is_superimposed, offline_map, CARMEN_OFFLINE_MAP_v);
		gui->nav_panel_config->map = (char *) "Offline Map";
		break;
	case CARMEN_COMPLETE_MAP_v:
		navigator_get_complete_map(is_superimposed);
		gui->nav_panel_config->map = (char *) "Complete Map";
		break;
	case CARMEN_REMISSION_MAP_v:
		navigator_get_specific_map(is_superimposed, remission_map, CARMEN_REMISSION_MAP_v);
		gui->nav_panel_config->map = (char *) "Remission Map";
		break;
	case CARMEN_MOVING_OBJECTS_MAP_v:
		navigator_get_specific_map(is_superimposed, moving_objects_map, CARMEN_MOVING_OBJECTS_MAP_v);
		gui->nav_panel_config->map = (char *) "Moving Objects";
		break;
	case CARMEN_ROAD_MAP_v:
		navigator_get_specific_map(is_superimposed, road_map, CARMEN_ROAD_MAP_v);
		gui->nav_panel_config->map = (char *) "Road Map";
		break;

	default:
		navigator_get_specific_map(is_superimposed, online_map, CARMEN_NAVIGATOR_MAP_v);
		gui->nav_panel_config->map = (char *) "Map";
		break;
	}
}


static carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_mapper_map_message *grid_map)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		map->map = (double **) malloc(grid_map->config.x_size * sizeof(double *));
	}

	map->config = grid_map->config;
	map->complete_map = grid_map->complete_map;
	for (i = 0; i < map->config.x_size; i++)
		map->map[i] = map->complete_map + i * map->config.y_size;

	return (map);
}


static carmen_map_t *
copy_grid_mapping_to_map_with_complete_map(carmen_map_t *map, carmen_mapper_map_message *grid_map)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		map->map = (double **) malloc(grid_map->config.x_size * sizeof(double *));
		map->complete_map = (double *) malloc(grid_map->config.x_size * grid_map->config.y_size * sizeof(double));
	}

	map->config = grid_map->config;
	memcpy(map->complete_map, grid_map->complete_map, grid_map->config.x_size * grid_map->config.y_size * sizeof(double));
	for (i = 0; i < map->config.x_size; i++)
		map->map[i] = map->complete_map + i * map->config.y_size;

	return (map);
}


static carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_moving_objects_map_message *grid_map)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		map->map = (double **) malloc(grid_map->config.x_size * sizeof(double *));
	}

	map->config = grid_map->config;
	map->complete_map = grid_map->complete_map;
	for (i = 0; i < map->config.x_size; i++)
		map->map[i] = map->complete_map + i * map->config.y_size;

	return (map);
}


carmen_map_t*
navigator_get_complete_map_map_pointer()
{
	if (complete_map == NULL)
	{
		if (navigator_get_complete_map(0) == NULL)
			return (NULL);
	}
	map_type = CARMEN_COMPLETE_MAP_v;

	return complete_map;
}


carmen_map_t*
navigator_get_offline_map_pointer()
{
	map_type = CARMEN_OFFLINE_MAP_v;

	return offline_map;
}


carmen_map_t*
navigator_get_road_map_pointer()
{
	map_type = CARMEN_ROAD_MAP_v;

	return road_map;
}


void
navigator_unset_goal(double x, double y)
{
	carmen_navigator_ackerman_unset_goal(x, y);
}


void
read_annotation_list_from_file(char *carmen_annotation_filename, std::vector<carmen_annotation_t> &list1, int annotation_type1,
		std::vector<carmen_annotation_t> &list2, int annotation_type2)
{
	char line[1024], description[1024];
	carmen_annotation_t a;
	FILE *stream = fopen(carmen_annotation_filename, "r");
	if (stream == NULL)
	{
		printf("Error: annotation file not found: %s\n", carmen_annotation_filename);
		return;
	}

	while (fgets(line, 1023, stream) != NULL)
	{
		if (sscanf(line, "%s %d %d %lf %lf %lf %lf", description, &a.annotation_type, &a.annotation_code,
				&a.annotation_orientation, &a.annotation_point.x, &a.annotation_point.y, &a.annotation_point.z) != 7)
			continue;

		if (description[0] == '#')
			continue;

		if (a.annotation_type != annotation_type1 && a.annotation_type != annotation_type2)
			continue;

		a.annotation_description = (char *) malloc((strlen(description) + 1) * sizeof(char));
		strcpy(a.annotation_description, description);

		if (a.annotation_type == annotation_type1)
			list1.push_back(a);
		else if (a.annotation_type == annotation_type2)
			list2.push_back(a);
	}
	fclose(stream);
}


std::vector <string>
read_glade_file(char *glade_path)
{
	std::vector <string> glade_file; //each position is a line of .glade file
	int bufferLength = 1024;
	char buffer[bufferLength];

	FILE *f_glade = fopen (glade_path, "r");
	if (f_glade == NULL)
	{
		fprintf(stderr, "Error: glade file not found: %s\n", glade_path);
		exit(1);
	}

	while (fgets(buffer, bufferLength, f_glade))
	{
		string g (buffer);
		glade_file.push_back(g);
	}

	fclose(f_glade);

	return glade_file;
}


std::vector <string>
format_annotation_to_gtk_list_store(std::vector <carmen_annotation_t> annotation_list, const char *annotation_prefix)
{
	std::vector <string> gtk_list_store;
	std::vector <string> sorted_items;
	string row_begin = "      <row>\n";
	string row_end = "      </row>\n";

	for (unsigned int i = 0; i < annotation_list.size(); i++)
	{
		string d(annotation_list[i].annotation_description);
		if (d.substr(0, strlen(annotation_prefix)) == std::string(annotation_prefix))
			d = d.substr(strlen(annotation_prefix));
		sorted_items.push_back(d);
	}
	std::sort(sorted_items.begin(), sorted_items.end());

	for (unsigned int i = 0; i < sorted_items.size(); i++)
	{
		gtk_list_store.push_back(row_begin);
		string d(sorted_items[i]);
		string col = "        <col id=\"0\" translatable=\"yes\">" + d + "</col>\n";
		gtk_list_store.push_back(col);
		gtk_list_store.push_back(row_end);
	}

	return gtk_list_store;
}


int
get_gtk_list_store_data_index(std::vector <string> &glade_file, const char *gtk_list_store_id)
{
	int index;

	for (index = 0; index < (int) glade_file.size(); index++)
		if (glade_file[index].find("\"GtkListStore\"") != std::string::npos && glade_file[index].find(gtk_list_store_id) != std::string::npos)
			break;

	for (; index < (int) glade_file.size(); index++)
		if (glade_file[index].find("</") != std::string::npos && glade_file[index].find("data") != std::string::npos)
			break;

	if (index == (int) glade_file.size())
		index = -1;

	return index;
}


void
build_glade_with_annotation (char *annotation_path)
{
	const char *place_annotation_prefix = "RDDF_PLACE_";
	const char *place_gtk_list_store_id = "\"listPlaceOfInterest\"";
	const char *predefined_route_annotation_prefix = "PREDEFINED_ROUTE_";
	const char *predefined_route_gtk_list_store_id = "\"listPredefinedRoute\"";

	read_annotation_list_from_file(annotation_path, place_of_interest_list, RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST,
			predefined_route_list, RDDF_ANNOTATION_TYPE_PREDEFINED_ROUTE);

	char glade_path[1000];
	char *carmen_home_path = getenv("CARMEN_HOME");
	sprintf(glade_path, "%s/data/gui/navigator_gui2.glade", carmen_home_path);
	std::vector <string> glade_file = read_glade_file(glade_path);

	std::vector <string> places_in_glade = format_annotation_to_gtk_list_store(place_of_interest_list, place_annotation_prefix);
	int place_list_data_index = get_gtk_list_store_data_index(glade_file, place_gtk_list_store_id);
	if (place_list_data_index >= 0)
		glade_file.insert(glade_file.begin() + place_list_data_index, places_in_glade.begin(), places_in_glade.end());

	std::vector <string> predefined_routes_in_glade = format_annotation_to_gtk_list_store(predefined_route_list, predefined_route_annotation_prefix);
	int predefined_route_list_data_index = get_gtk_list_store_data_index(glade_file, predefined_route_gtk_list_store_id);
	if (predefined_route_list_data_index >= 0)
		glade_file.insert(glade_file.begin() + predefined_route_list_data_index, predefined_routes_in_glade.begin(), predefined_routes_in_glade.end());

	char glade_path_with_annotation[1000];
	sprintf(glade_path_with_annotation, "%s/data/gui/navigator_gui2_annotation.glade", carmen_home_path);
	FILE *f_glade_with_annotations = fopen (glade_path_with_annotation, "w");

	for (unsigned int i = 0; i < glade_file.size(); i++)
	{
		fprintf(f_glade_with_annotations, "%s", glade_file[i].c_str());
	}
	fclose(f_glade_with_annotations);
}


void
get_active_maps_from_menu(char **map, char **superimposed_map)
{
	View::GtkGui::Controls controls_ = gui->controls_;

	if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_Map)))
		*map = (char *) "Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_MapLevel1)))
		*map = (char *) "Map Level1";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_OfflineMap)))
		*map = (char *) "Offline Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_Utility)))
		*map = (char *) "Utility";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_Costs)))
		*map = (char *) "Costs";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_Likelihood)))
		*map = (char *) "Likelihood";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_GlobalLikelihood)))
		*map = (char *) "Global Likelihood";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_Lane)))
		*map = (char *) "Lane";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_CompleteMap)))
		*map = (char *) "Complete Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_RemissionMap)))
		*map = (char *) "Remission Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_MovingObjects)))
		*map = (char *) "Moving Objects";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuMaps_RoadMap)))
		*map = (char *) "Road Map";

	if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_None)))
		*superimposed_map = (char *) "None";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Map)))
		*superimposed_map = (char *) "Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_MapLevel1)))
		*superimposed_map = (char *) "Map Level1";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_OfflineMap)))
		*superimposed_map = (char *) "Offline Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Utility)))
		*superimposed_map = (char *) "Utility";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Costs)))
		*superimposed_map = (char *) "Costs";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Likelihood)))
		*superimposed_map = (char *) "Likelihood";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_GlobalLikelihood)))
		*superimposed_map = (char *) "Global Likelihood";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Lane)))
		*superimposed_map = (char *) "Lane";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_RemissionMap)))
		*superimposed_map = (char *) "Remission Map";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_MovingObjects)))
		*superimposed_map = (char *) "Moving Objects";
	else if (gtk_check_menu_item_get_active(GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_RoadMap)))
		*superimposed_map = (char *) "Road Map";
}


void
set_window_size_and_position()
{
	if (window_width > 0 && window_height > 0)
		gtk_window_resize(GTK_WINDOW(gui->controls_.main_window), window_width, window_height);
	if (window_x >= 0 && window_y >= 0)
		gtk_window_move(GTK_WINDOW(gui->controls_.main_window), window_x, window_y);
}


void
update_moving_objects_list(int id, carmen_moving_objects_point_clouds_message *msg)
{
	moving_objects_tracking_t *moving_objects_tracking = (moving_objects_tracking_t *) malloc(msg->num_point_clouds * sizeof(moving_objects_tracking_t));

	for (int i = 0; i < msg->num_point_clouds; i++)
	{
		moving_objects_tracking[i].moving_objects_pose.orientation.yaw = msg->point_clouds[i].orientation;
		moving_objects_tracking[i].moving_objects_pose.orientation.roll = 0.0;
		moving_objects_tracking[i].moving_objects_pose.orientation.pitch = 0.0;
		moving_objects_tracking[i].moving_objects_pose.position.x = msg->point_clouds[i].object_pose.x;
		moving_objects_tracking[i].moving_objects_pose.position.y = msg->point_clouds[i].object_pose.y;
		moving_objects_tracking[i].moving_objects_pose.position.z = msg->point_clouds[i].object_pose.z;
		moving_objects_tracking[i].length = msg->point_clouds[i].model_features.geometry.length;
		moving_objects_tracking[i].width = msg->point_clouds[i].model_features.geometry.width;
		moving_objects_tracking[i].height = msg->point_clouds[i].height;
		moving_objects_tracking[i].linear_velocity = msg->point_clouds[i].linear_velocity;
		moving_objects_tracking[i].geometric_model = msg->point_clouds[i].geometric_model;
		moving_objects_tracking[i].model_features = msg->point_clouds[i].model_features;
		moving_objects_tracking[i].num_associated = msg->point_clouds[i].num_associated;
	}

	gui->navigator_graphics_update_moving_objects(id, msg->num_point_clouds, moving_objects_tracking);
	free(moving_objects_tracking);
}

///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
navigator_update_robot(carmen_world_robot_and_trailer_pose_t *robot)
{
	if (robot == NULL)
	{
		carmen_localize_ackerman_initialize_uniform_command();
		IPC_listen(50);
	}
	else
	{
		carmen_verbose("Set robot position to %d %d %f\n",
				carmen_round(robot->pose.x),
				carmen_round(robot->pose.y),
				carmen_radians_to_degrees(robot->pose.theta));

		carmen_point_t pose_without_beta = {robot->pose.x,robot->pose.y, robot->pose.theta};
		carmen_localize_ackerman_initialize_gaussian_command(pose_without_beta, localize_std, robot->pose.beta);
		IPC_listen(50);
	}
}


void
navigator_set_goal_by_place(carmen_place_p place)
{
	carmen_navigator_ackerman_set_goal_place(place->name);
}


void
navigator_stop_moving(void)
{
	if (!carmen_navigator_ackerman_stop())
	{
		IPC_listen(50);
		carmen_verbose("Said stop\n");
	}
	else
		carmen_verbose("Could not say stop\n");
}


void
navigator_start_moving(void)
{

	if (!carmen_navigator_ackerman_go())
	{
		carmen_verbose("Said go!\n");
		IPC_listen(50);
	}
	else
		carmen_verbose("could not say go!\n");
}


void
navigator_set_goal(double x, double y, double theta)
{
	carmen_verbose("Set goal to %.1f %.1f\n", x, y);
	carmen_navigator_ackerman_set_goal(x, y, theta);
	IPC_listen(50);
}


void
navigator_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_task_t task)
{
	carmen_behavior_selector_set_algorithm(algorithm, task);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_parse_polygon_file(carmen_polygon_config_t *poly_config, char *poly_file)
{
	FILE *poly = fopen(poly_file, "r");
	if (poly == NULL)
	{
		printf("Could not open polygon file %s\n", poly_file);
		exit(1);
	}

	fscanf(poly,"%lf\n", &(poly_config->displacement));
	fscanf(poly,"%d\n", &(poly_config->n_points));
	poly_config->points = (double *) malloc(poly_config->n_points * 2 * sizeof(double));
	for (int i = 0; i < poly_config->n_points; i++)
	{
		double p1, p2;
		fscanf(poly,"%lf %lf\n", &p1, &p2);
		printf("%lf %lf\n", p1, p2);
		poly_config->points[2 * i] = p1;
		poly_config->points[2 * i + 1] = p2;
	}

	fclose(poly);
}


static void
read_parameters_semi_trailer(int argc, char **argv, int semi_trailer_type)
{
	semi_trailer_config.type = semi_trailer_type;
	if (semi_trailer_type == 0)
		return;

	char semi_trailer_string[2048];

	sprintf(semi_trailer_string, "%s%d", "semi_trailer", semi_trailer_config.type);

	char *semi_trailer_poly_file;

	carmen_param_t semi_trailer_param_list[] = {
		{semi_trailer_string, (char *) "d",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.d),							   	  0, NULL},
		{semi_trailer_string, (char *) "M",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.M),							   	  0, NULL},
		{semi_trailer_string, (char *) "width",							 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.width),							  0, NULL},
		{semi_trailer_string, (char *) "distance_between_axle_and_front", 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.distance_between_axle_and_front), 0, NULL},
		{semi_trailer_string, (char *) "distance_between_axle_and_back",	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.distance_between_axle_and_back),  0, NULL},
		{semi_trailer_string, (char *) "max_beta",						 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.max_beta),						  0, NULL},
		{semi_trailer_string, (char *) "polygon_file",					 	CARMEN_PARAM_STRING, &(semi_trailer_poly_file), 							  	0, NULL}
	};
	carmen_param_install_params(argc, argv, semi_trailer_param_list, sizeof(semi_trailer_param_list)/sizeof(semi_trailer_param_list[0]));

	semi_trailer_config.max_beta = carmen_degrees_to_radians(semi_trailer_config.max_beta);

	char polygon_file[1024];
	strcpy(polygon_file, getenv("CARMEN_HOME"));
	strcat(polygon_file,"/bin/");

	strcat(polygon_file, semi_trailer_poly_file);
	printf("%s\n", polygon_file);
	carmen_parse_polygon_file(&semi_trailer_poly_config, polygon_file);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
mapper_handler(carmen_mapper_map_message *message)
{
	static double last_time_stamp = 0.0;

	if (message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (online_map && (message->config.x_size != online_map->config.x_size || message->config.y_size != online_map->config.y_size))
		carmen_map_destroy(&online_map);

	online_map = copy_grid_mapping_to_map(online_map, message);

	if (superimposedmap_type == CARMEN_NAVIGATOR_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(online_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_NAVIGATOR_MAP_v)
		gui->navigator_graphics_change_map(online_map);
}


void
carmen_mapper_compact_map_message_handler(carmen_mapper_compact_map_message *message)
{
	static carmen_compact_map_t *compact_occupancy_map = NULL;
	static carmen_map_t occupancy_map;

	if (compact_occupancy_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&occupancy_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(occupancy_map.complete_map, 0, occupancy_map.config.x_size * occupancy_map.config.y_size * sizeof(double));

		compact_occupancy_map = (carmen_compact_map_t *) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_map_message_to_compact_map(compact_occupancy_map, message);
		carmen_prob_models_uncompress_compact_map(&occupancy_map, compact_occupancy_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&occupancy_map, compact_occupancy_map, 0.0);
		carmen_prob_models_free_compact_map(compact_occupancy_map);
		carmen_cpy_compact_map_message_to_compact_map(compact_occupancy_map, message);
		carmen_prob_models_uncompress_compact_map(&occupancy_map, compact_occupancy_map);
	}

	online_map = &occupancy_map;

	if (superimposedmap_type == CARMEN_NAVIGATOR_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(online_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_NAVIGATOR_MAP_v)
		gui->navigator_graphics_change_map(online_map);
}


void
mapper_level1_handler(carmen_mapper_map_message *message)
{
	static double last_time_stamp = 0.0;

	if (message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (map_level1 && (message->config.x_size != map_level1->config.x_size || message->config.y_size != map_level1->config.y_size))
		carmen_map_destroy(&map_level1);

	map_level1 = copy_grid_mapping_to_map(map_level1, message);

	if (superimposedmap_type == CARMEN_NAVIGATOR_MAP_LEVEL1_v)
	{
		carmen_map_interface_set_superimposed_map(map_level1);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_NAVIGATOR_MAP_LEVEL1_v)
		gui->navigator_graphics_change_map(map_level1);
}


void
ford_escape_status_handler(carmen_ford_escape_status_message *message)
{
	int yellow_button = message->g_XGV_component_status & XGV_MANUAL_OVERRIDE_FLAG;//carmen_get_bit_value(message->g_XGV_component_status, 0);

	if (yellow_button)
		record_screen = 0;

	if (!yellow_button && (autonomous_record_screen == 1))
		record_screen = 1;
}


static void
offline_map_update_handler(carmen_mapper_map_message *new_map)
{
	if (new_map->size <= 0)
		return;

	if (offline_map && (new_map->config.x_size != offline_map->config.x_size || new_map->config.y_size != offline_map->config.y_size))
		carmen_map_destroy(&offline_map);

	if (strcmp(previous_place_of_interest, place_of_interest) == 0)
	{
		if (strcmp (place_of_interest, "Robot") == 0)
			offline_map = copy_grid_mapping_to_map_with_complete_map(offline_map, new_map);
		else
			offline_map_tmp = copy_grid_mapping_to_map_with_complete_map(offline_map_tmp, new_map);

	}
	else
	{
		if (strcmp (place_of_interest, "Robot") == 0)
			carmen_grid_mapping_copy_map(offline_map, offline_map_tmp);
		else
			offline_map_tmp = copy_grid_mapping_to_map_with_complete_map(offline_map_tmp, new_map);
	}
	strcpy(previous_place_of_interest, place_of_interest);

	if (superimposedmap_type == CARMEN_OFFLINE_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(offline_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_OFFLINE_MAP_v)
		gui->navigator_graphics_change_map(offline_map);
}

static void
road_map_update_handler(carmen_mapper_map_message *new_map)
{
	if (new_map->size <= 0)
		return;

	if (road_map && (new_map->config.x_size != road_map->config.x_size || new_map->config.y_size != road_map->config.y_size))
		carmen_map_destroy(&road_map);

	road_map = copy_grid_mapping_to_map(road_map, new_map);

	if (superimposedmap_type == CARMEN_ROAD_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(road_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_ROAD_MAP_v)
		gui->navigator_graphics_change_map(road_map);
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static double last_time_stamp = 0.0;
	static carmen_compact_map_t *compact_cost_map = NULL;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(cost_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(cost_map->complete_map, 0, cost_map->config.x_size * cost_map->config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t*) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(cost_map, compact_cost_map);
	}

	cost_map->config = message->config;

	if (superimposedmap_type == CARMEN_COST_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(cost_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_COST_MAP_v)
		gui->navigator_graphics_change_map(cost_map);
}


static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
	static carmen_compact_map_t *compact_lane_map = NULL;

	if (compact_lane_map == NULL)
	{
		carmen_grid_mapping_create_new_map(lane_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');

		for (int i = 0; i < lane_map->config.x_size * lane_map->config.y_size; ++i)
			lane_map->complete_map[i] = 1.0;

		compact_lane_map = (carmen_compact_map_t*) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_lane_message_to_compact_map(compact_lane_map, message);
		carmen_prob_models_uncompress_compact_map(lane_map, compact_lane_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(lane_map, compact_lane_map, 1.0);
		carmen_prob_models_free_compact_map(compact_lane_map);
		carmen_cpy_compact_lane_message_to_compact_map(compact_lane_map, message);
		carmen_prob_models_uncompress_compact_map(lane_map, compact_lane_map);
	}

	lane_map->config = message->config;

	if (superimposedmap_type == CARMEN_LANE_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(lane_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_LANE_MAP_v)
		gui->navigator_graphics_change_map(lane_map);
}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	if (first_localize_map_message_received)
	{
		memset(&localize_all_maps, 0, sizeof(localize_all_maps));
		localize_all_maps.config = message->config;
		first_localize_map_message_received = 0;
	}

	carmen_map_server_localize_map_message_to_localize_map(message, &localize_all_maps);

	if (likelihood_map == NULL)
	{
		likelihood_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		carmen_test_alloc(likelihood_map);
		carmen_grid_mapping_initialize_map(likelihood_map, localize_all_maps.config.x_size, localize_all_maps.config.resolution, 'm');
	}

	if (global_likelihood_map == NULL)
	{
		global_likelihood_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		carmen_test_alloc(global_likelihood_map);
		carmen_grid_mapping_initialize_map(global_likelihood_map, localize_all_maps.config.x_size, localize_all_maps.config.resolution, 'm');
	}

	if (remission_map == NULL)
	{
		remission_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		carmen_test_alloc(remission_map);
		carmen_grid_mapping_initialize_map(remission_map, localize_all_maps.config.x_size, localize_all_maps.config.resolution, 'm');
	}

	remission_map->config = likelihood_map->config = global_likelihood_map->config = localize_all_maps.config = message->config;
	localize_all_maps.carmen_mean_remission_map.config = localize_all_maps.config;
	memcpy(likelihood_map->complete_map, localize_all_maps.complete_prob, localize_all_maps.config.x_size * localize_all_maps.config.y_size * sizeof(double));
	memcpy(global_likelihood_map->complete_map, localize_all_maps.complete_gprob, localize_all_maps.config.x_size * localize_all_maps.config.y_size * sizeof(double));
	memcpy(remission_map->complete_map, localize_all_maps.carmen_mean_remission_map.complete_map, localize_all_maps.config.x_size * localize_all_maps.config.y_size * sizeof(double));

	if (superimposedmap_type == CARMEN_LOCALIZE_LMAP_v)
		carmen_map_interface_set_superimposed_map(likelihood_map);
	else if (superimposedmap_type == CARMEN_LOCALIZE_GMAP_v)
		carmen_map_interface_set_superimposed_map(global_likelihood_map);
	else if (superimposedmap_type == CARMEN_REMISSION_MAP_v)
		carmen_map_interface_set_superimposed_map(remission_map);
	gui->navigator_graphics_redraw_superimposed();

	if (gui->navigator_graphics_update_map() && is_graphics_up)
	{
		if (map_type == CARMEN_LOCALIZE_LMAP_v)
			gui->navigator_graphics_change_map(likelihood_map);
		else if (map_type == CARMEN_LOCALIZE_GMAP_v)
			gui->navigator_graphics_change_map(global_likelihood_map);
		else if (map_type == CARMEN_REMISSION_MAP_v)
			gui->navigator_graphics_change_map(remission_map);
	}
}


static void
grid_mapping_moving_objects_raw_map_handler(carmen_moving_objects_map_message *message)
{
	static double last_time_stamp = 0.0;

	if (message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (moving_objects_map && (message->config.x_size != moving_objects_map->config.x_size || message->config.y_size != moving_objects_map->config.y_size))
		carmen_map_destroy(&moving_objects_map);

	moving_objects_map = copy_grid_mapping_to_map(moving_objects_map, message);

	if (superimposedmap_type == CARMEN_MOVING_OBJECTS_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(moving_objects_map);
		gui->navigator_graphics_redraw_superimposed();
	}

	if (gui->navigator_graphics_update_map() && is_graphics_up && map_type == CARMEN_MOVING_OBJECTS_MAP_v)
		gui->navigator_graphics_change_map(moving_objects_map);
}


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_traj_point_t new_robot;

	new_robot.x		= msg->globalpos.x;
	new_robot.y		= msg->globalpos.y;
	new_robot.theta = msg->globalpos.theta;
	new_robot.t_vel = msg->v;
	new_robot.r_vel = msg->phi;

	if (!is_graphics_up)
		return;

	if (goal_set)
		gui->navigator_graphics_update_display(&new_robot, msg, &last_goal, autonomous);
	else
		gui->navigator_graphics_update_display(&new_robot, msg, NULL, autonomous);

	if (msg->semi_trailer_type != semi_trailer_config.type)
		read_parameters_semi_trailer(argc_global, argv_global, msg->semi_trailer_type);
}


static void
carmen_simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	gui->navigator_graphics_update_simulator_truepos(msg->truepose, msg->beta);
}


static void
navigator_ackerman_status_handler(carmen_navigator_ackerman_status_message *msg)
{
	carmen_verbose("Got Status message: Robot %.1f %.1f %.2f Goal: %.0f %.0f\n",
			msg->robot.x, msg->robot.y, msg->robot.theta,
			msg->goal.x, msg->goal.y);

	last_navigator_status = msg->timestamp;

	goal_set = msg->goal_set;
	autonomous = msg->autonomous;

	if (!is_graphics_up)
		return;

	if (msg->goal_set)
	{
//		last_goal = msg->goal;

		gui->navigator_graphics_update_display(NULL, NULL, &last_goal, msg->autonomous);
	}
	else
		gui->navigator_graphics_update_display(NULL, NULL, NULL, msg->autonomous);
}


//static void
//navigator_goal_list_message(carmen_behavior_selector_goal_list_message *goals)
//{
//	gui->navigator_graphics_update_goal_list(goals->goal_list, goals->size);
//}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations)
{
	gui->navigator_graphics_update_goal_list(path_goals_and_annotations->goal_list, path_goals_and_annotations->goal_list_size);
	if (path_goals_and_annotations->goal_list_size > 0)
		last_goal = path_goals_and_annotations->goal_list[0];
}


static void
navigator_rddf_waypoints_handler(carmen_rddf_waypoints_around_end_point_message *waypoints)
{
	gui->navigator_graphics_update_waypoint_list(waypoints->poses, waypoints->number_of_poses);
}



static void
lane_detector_handler(carmen_lane_detector_lane_message_t *msg)
{
	gui->lane_markings_msg = msg;
}




static void
carmen_navigator_ackerman_plan_message_handler(carmen_navigator_ackerman_plan_message *plan)
{
	gui->navigator_graphics_update_plan(plan->path, plan->path_length);
}


static void
carmen_navigator_gui_path_message_handler(carmen_navigator_gui_path_message *msg)
{
	gui->navigator_graphics_update_path(msg->path, msg->path_length, msg->path_id);
}


static void
carmen_simulator_ackerman_objects_message_handler(carmen_simulator_ackerman_objects_message *msg)
{
	gui->navigator_graphics_update_simulator_objects(msg->num_objects, msg->objects);
}


static void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	update_moving_objects_list(5, moving_objects_point_clouds_message);
}


void
moving_objects_point_clouds_message_handler_0(carmen_moving_objects_point_clouds_message *msg)
{
	update_moving_objects_list(0, msg);
}


static void
plan_tree_handler(carmen_navigator_ackerman_plan_tree_message *msg)
{
	gui->navigator_graphics_update_plan_tree(msg->p1, msg->p2, msg->mask, msg->num_edges, msg->paths, msg->path_size, msg->num_path);
}


static void
plan_to_draw_handler(carmen_navigator_ackerman_plan_to_draw_message *message)
{
	gui->navigator_graphics_update_plan_to_draw(message->path_size, message->path);
}


static void
carmen_parking_assistant_goal_handler(carmen_parking_assistant_goal_message *msg)
{
	gui->navigator_graphics_update_parking_assistant_goal(msg->pose);
}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	carmen_point_t pose;
	pose.x = msg->pose.position.x;
	pose.y = msg->pose.position.y;
	pose.theta = msg->pose.orientation.yaw;

	last_v = msg->velocity.x;
	last_phi = msg->phi;

	gui->navigator_graphics_update_fused_odometry(pose);
}


static void
carmen_behavior_selector_current_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	gui->navigator_graphics_update_behavior_selector_state(msg);
}


static void
traffic_sign_handler(carmen_rddf_traffic_sign_message *msg)
{
	gui->navigator_graphics_update_traffic_sign_state(msg);
}


static void
odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	last_phi = msg->phi;
	last_v = msg->v;
}


static void
display_config_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR	formatter;
	carmen_navigator_ackerman_display_config_message msg;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg, sizeof(carmen_navigator_ackerman_display_config_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	if (msg.reset_all_to_defaults)
		gui->navigator_graphics_reset();
	else
		gui->navigator_graphics_display_config(msg.attribute, msg.value, msg.status_message);

	free(msg.attribute);

	if (msg.status_message)
		free(msg.status_message);
}


void
navigator_ackerman_go_message_handler()
{
	gui->navigator_graphics_go_message_received();
}


void
navigator_ackerman_stop_message_handler()
{
	gui->navigator_graphics_stop_message_received();
}


void
carmen_route_planner_road_network_message_handler(carmen_route_planner_road_network_message *message)
{
	gui->route_planner_route = message;
}


void
carmen_offroad_planner_plan_message_handler(carmen_offroad_planner_plan_message *message)
{
	gui->offroad_planner_plan = message;
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	gui->final_goal.map = gui->controls_.map_view->internal_map;
	gui->final_goal.pose = {rddf_end_point_message->point.x, rddf_end_point_message->point.y, rddf_end_point_message->point.theta, rddf_end_point_message->point.beta};
	gui->final_goal_placed_and_oriented = 1;
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
}


static gint
handle_ipc(gpointer			*data __attribute__ ((unused)),
		gint				 source __attribute__ ((unused)),
		GdkInputCondition condition __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);

	carmen_graphics_update_ipc_callbacks((GdkInputFunction) handle_ipc);

	return 1;
}


static void
nav_shutdown(int signo __attribute__ ((unused)))
{
	static int done = 0;

	if (!done)
	{
		done = 1;
		carmen_ipc_disconnect();
		exit(1);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
read_parameters(int argc, char *argv[],
		carmen_robot_config_t *robot_config,
		carmen_semi_trailer_config_t *semi_trailer_config,
		carmen_polygon_config_t *robot_poly_config,
		carmen_navigator_config_t *nav_config,
		carmen_navigator_panel_config_t *navigator_panel_config)
{
	int num_items;

	char *robot_poly_file;


	carmen_param_t param_list[] =
	{
		{(char *) "robot",			 (char *) "length",					CARMEN_PARAM_DOUBLE, &(robot_config->length),						0, NULL},
		{(char *) "robot",			 (char *) "width",					CARMEN_PARAM_DOUBLE, &(robot_config->width),						0, NULL},
		{(char *) "robot",			 (char *) "acceleration",			CARMEN_PARAM_DOUBLE, &(robot_config->acceleration),					1, NULL},
		{(char *) "robot",			 (char *) "rectangular",			CARMEN_PARAM_ONOFF,  &(robot_config->rectangular),					1, NULL},
		{(char *) "robot", 			 (char *) "polygon_file",			CARMEN_PARAM_STRING, &(robot_poly_file), 							0, NULL},
		{(char *) "semi_trailer",	 (char *) "initial_type",			CARMEN_PARAM_INT, 	 &(semi_trailer_config->type), 					0, NULL},

		{(char *) "navigator",		 (char *) "map_update_radius",		CARMEN_PARAM_INT,    &(nav_config->map_update_radius),		1, NULL},
		{(char *) "navigator",		 (char *) "goal_size",				CARMEN_PARAM_DOUBLE, &(nav_config->goal_size),				1, NULL},
		{(char *) "navigator",		 (char *) "goal_theta_tolerance",	CARMEN_PARAM_DOUBLE, &(nav_config->goal_theta_tolerance),	1, NULL},
		{(char *) "navigator_panel", (char *) "initial_map_zoom",		CARMEN_PARAM_DOUBLE, &(navigator_panel_config->initial_map_zoom),	1, NULL},
		{(char *) "navigator_panel", (char *) "track_robot",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->track_robot),		1, NULL},
		{(char *) "navigator_panel", (char *) "draw_path",				CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_path),			1, NULL},
		{(char *) "navigator_panel", (char *) "draw_waypoints",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_waypoints),		1, NULL},
		{(char *) "navigator_panel", (char *) "draw_robot_waypoints",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_robot_waypoints),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_particles",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_particles),		1, NULL},
		{(char *) "navigator_panel", (char *) "show_gaussians",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_gaussians),		1, NULL},
		{(char *) "navigator_panel", (char *) "show_laser",				CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_lasers),		1, NULL},
		{(char *) "navigator_panel", (char *) "show_simulator_objects", CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_simulator_objects), 1, NULL},
		{(char *) "navigator_panel", (char *) "show_true_pos",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_true_pos),		1, NULL},
		{(char *) "navigator_panel", (char *) "show_tracked_objects",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_tracked_objects),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_command_plan",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_command_plan),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_mpp_motion_plan",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_mpp_motion_plan),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_oa_motion_plan",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_oa_motion_plan),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_offroad_plan",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_offroad_plan),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_nearby_lanes",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_nearby_lanes),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_path_plans",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_path_plans),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_dynamic_points",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_dynamic_points),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_dynamic_objects",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_dynamic_objects),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_annotations",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_annotations),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_lane_markings",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_lane_markings),	1, NULL},
		{(char *) "navigator_panel", (char *) "show_collision_range",	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_collision_range),	1, NULL},
		{(char *) "navigator_panel", (char *) "use_ackerman",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->use_ackerman),		1, NULL},
		{(char *) "navigator_panel", (char *) "localize_std_x",			CARMEN_PARAM_DOUBLE, &localize_std.x,								1, NULL},
		{(char *) "navigator_panel", (char *) "localize_std_y",			CARMEN_PARAM_DOUBLE, &localize_std.y,								1, NULL},
		{(char *) "navigator_panel", (char *) "localize_std_theta",		CARMEN_PARAM_DOUBLE, &localize_std.theta,							1, NULL},
		{(char *) "navigator_panel", (char *) "map",					CARMEN_PARAM_STRING, &(navigator_panel_config->map),				0, NULL},
		{(char *) "navigator_panel", (char *) "superimposed_map",		CARMEN_PARAM_STRING, &(navigator_panel_config->superimposed_map),	0, NULL},
		{(char *) "navigator_panel", (char *) "window_width",			CARMEN_PARAM_INT, &window_width,	0, NULL},
		{(char *) "navigator_panel", (char *) "window_height",			CARMEN_PARAM_INT, &window_height,	0, NULL},
		{(char *) "navigator_panel", (char *) "window_x",				CARMEN_PARAM_INT, &window_x,	0, NULL},
		{(char *) "navigator_panel", (char *) "window_y",				CARMEN_PARAM_INT, &window_y,	0, NULL},
		{(char *) "mapper",			 (char *) "height_max_level",		CARMEN_PARAM_INT, &height_max_level,								0, NULL},
		{(char *) "route_planner",	 (char *) "in_graph_mode", 			CARMEN_PARAM_ONOFF,  &use_route_planner_in_graph_mode, 0, NULL},
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	localize_std.theta = carmen_degrees_to_radians(localize_std.theta);

	char polygon_file[1024];
	strcpy(polygon_file, getenv("CARMEN_HOME"));
	strcat(polygon_file,"/bin/");

	strcat(polygon_file, robot_poly_file);
	printf("%s\n", polygon_file);
	carmen_parse_polygon_file(robot_poly_config, polygon_file);

	if (semi_trailer_config->type > 0)
		read_parameters_semi_trailer(argc, argv, semi_trailer_config->type);

	carmen_param_t param_cmd_list[] =
	{
		{(char *) "commandline", (char *) "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},
		{(char *) "commandline", (char *) "autonomous_record_screen", CARMEN_PARAM_INT, &autonomous_record_screen, 0, NULL},
		{(char *) "commandline", (char *) "annotation_path", CARMEN_PARAM_STRING, &annotation_path, 0, NULL},
	};

	num_items = sizeof(param_cmd_list) / sizeof(param_cmd_list[0]);

	carmen_param_allow_unfound_variables(1);

	carmen_param_install_params(argc, argv, param_cmd_list, num_items);

	if (annotation_path != NULL)
	{
		build_glade_with_annotation (annotation_path);
		use_glade_with_annotations = 1;
	}

	carmen_param_t param_publish_list[] =
	{
		{(char *) "navigator_panel", (char *) "publish_map_view", 		   CARMEN_PARAM_ONOFF,  &publish_map_view, 		    1, NULL},
		{(char *) "navigator_panel", (char *) "publish_map_view_interval", CARMEN_PARAM_DOUBLE, &publish_map_view_interval, 1, NULL},
	};

	num_items = sizeof(param_publish_list) / sizeof(param_publish_list[0]);
	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_publish_list, num_items);
}


void
subscribe_ipc_messages()

{
	IPC_RETURN_TYPE err;

	carmen_navigator_ackerman_subscribe_status_message(NULL, (carmen_handler_t) (navigator_ackerman_status_handler), CARMEN_SUBSCRIBE_LATEST);
//	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) (navigator_goal_list_message), CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) (path_goals_and_annotations_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_ackerman_subscribe_plan_message(NULL, (carmen_handler_t) (carmen_navigator_ackerman_plan_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) (carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) (carmen_simulator_ackerman_truepos_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_simulator_ackerman_subscribe_objects_message(NULL, (carmen_handler_t) (carmen_simulator_ackerman_objects_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) (carmen_behavior_selector_current_state_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_traffic_sign_message(NULL, (carmen_handler_t) (traffic_sign_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_ackerman_subscribe_plan_tree_message(NULL, (carmen_handler_t) (plan_tree_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) (fused_odometry_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) (odometry_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_gui_subscribe_path_message(NULL, (carmen_handler_t) (carmen_navigator_gui_path_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME);

	err = IPC_subscribe(CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME, display_config_handler, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe message", CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME);

    // Todo: Add a function in the interfaces.
	carmen_subscribe_message((char *) CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME,
			(char *) CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_FMT,
			NULL, sizeof(carmen_navigator_ackerman_plan_to_draw_message),
			(carmen_handler_t) plan_to_draw_handler, CARMEN_SUBSCRIBE_LATEST);


	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_road_map(NULL, (carmen_handler_t) road_map_update_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
	if (height_max_level > 0)
		carmen_mapper_subscribe_map_level1_message(NULL, (carmen_handler_t) mapper_level1_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_grid_mapping_moving_objects_raw_map_subscribe_message(NULL, (carmen_handler_t) grid_mapping_moving_objects_raw_map_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_moving_objects_map_subscribe_message(NULL, (carmen_handler_t) grid_mapping_moving_objects_raw_map_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message_generic(0, NULL, (carmen_handler_t) moving_objects_point_clouds_message_handler_0, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_compact_lane_map(NULL, (carmen_handler_t) map_server_compact_lane_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_compact_cost_map(NULL, (carmen_handler_t) map_server_compact_cost_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_waypoints_around_end_point_message(NULL, (carmen_handler_t) navigator_rddf_waypoints_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_lane_subscribe(NULL, (carmen_handler_t) lane_detector_handler, CARMEN_SUBSCRIBE_LATEST);

	err = IPC_defineMsg(CARMEN_RDDF_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_END_POINT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_END_POINT_MESSAGE_NAME);

	carmen_parking_assistant_subscribe_goal(NULL, (carmen_handler_t) carmen_parking_assistant_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t) navigator_ackerman_go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t) navigator_ackerman_stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) carmen_route_planner_road_network_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_offroad_planner_subscribe_plan_message(NULL, (carmen_handler_t) carmen_offroad_planner_plan_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
init_navigator_gui_variables(int argc, char *argv[])
{
	carmen_localize_ackerman_globalpos_message globalpos;
	gui->navigator_graphics_initialize(argc, argv, &globalpos, &robot_config, &semi_trailer_config, &robot_poly_config, &semi_trailer_poly_config, &nav_config, &nav_panel_config);

	carmen_graphics_update_ipc_callbacks((GdkInputFunction) (handle_ipc));

	is_graphics_up = 1;

	online_map = (carmen_map_t*) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(online_map);
	offline_map = (carmen_map_t*) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(offline_map);
	cost_map = (carmen_map_t*) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(cost_map);
	lane_map = (carmen_map_t*) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(lane_map);
	road_map = (carmen_map_t*) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(road_map);

	strcpy(place_of_interest, "Robot");
	strcpy(previous_place_of_interest, "Robot");
	strcpy(predefined_route, "None");
	predefined_route_code = 0;
}


int
main(int argc, char *argv[])
{
	argc_global = argc;
	argv_global = argv;

	setlocale(LC_ALL, "C");

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, nav_shutdown);

	read_parameters(argc, argv, &robot_config, &semi_trailer_config, &robot_poly_config, &nav_config, &nav_panel_config);

	carmen_grid_mapping_init_parameters(0.2, 210);

	// Esta incializacao evita que o valgrind reclame de varias variaveis nao inicializadas
	static View::GtkGui _gui(argc, argv);
	gui = &_gui;
//	gui->GtkGui(argc, argv);
//	gui = new View::GtkGui(argc, argv);
	// Verificar pastas de record_screen se a pasta existir no /dados e se o parâmetro de entrada na linha de comando
	// for igual a 1

	init_navigator_gui_variables(argc, argv);

 	set_window_size_and_position();

	subscribe_ipc_messages();

#ifdef USE_DOT
	initialize_dynamics();
#endif

	gui->navigator_graphics_start(map_path);

	return 0;
}
