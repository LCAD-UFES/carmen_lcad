#include "gtk_gui.h"

extern void
mapper_handler(carmen_mapper_map_message *message);

GdkColor *
build_color_gradient()
{
	static GdkColor gradient[256];
	int i;
	
	for (i = 0; i < 256; i++)
		gradient[i] = carmen_graphics_add_color_rgb(i, 255 - i, 0);
	
	return (gradient);
}


static double
get_log_prob(double prob)
{
#define	MIN_PROB	0.000000000000000000000000000000000001
	double log_prob;
	
	if (prob < 0.0) // unknown
		prob = 0.5;

	if (prob < MIN_PROB)
		log_prob = log(MIN_PROB);
	else 
		log_prob = log(prob);

	return (log_prob);
}


int *
compute_particle_weight_color(carmen_localize_ackerman_particle_ipc_t *particles, int num_particles)
{
	int *weight_color;
	double *log_prob_weight;
	double max_log_prob_weight;
	double min_log_prob_weight;
	double log_prob;
	int index;
	
	weight_color = (int *) malloc(num_particles * sizeof(int));
	log_prob_weight = (double *) malloc(num_particles * sizeof(double));
	max_log_prob_weight = log(MIN_PROB);
	min_log_prob_weight = 0.0;
	for (index = 0; index < num_particles; index++)
	{
		log_prob = get_log_prob(particles[index].weight);
		if (log_prob > max_log_prob_weight)
			max_log_prob_weight = log_prob;
		if (log_prob < min_log_prob_weight)
			min_log_prob_weight = log_prob;
		log_prob_weight[index] = log_prob;
	}
	for (index = 0; index < num_particles; index++)
	{
		if ((max_log_prob_weight - min_log_prob_weight) > 0.0)
			weight_color[index] = (int) (255.0 * ((log_prob_weight[index] - min_log_prob_weight) / (max_log_prob_weight - min_log_prob_weight)));
		else if (max_log_prob_weight == 0.0)
			weight_color[index] = 255;
		else
			weight_color[index] = 0;
	}

//	FILE *caco = fopen("cacoxxx.txt", "a");
//	for (int i = 0; i < num_particles; i++)
//		fprintf(caco, "%03d %3.1lf\n", i, (double) weight_color[i]);
//	fprintf(caco, "++++++++++++++++++++++++++++++\n");
//	fclose(caco);

	free(log_prob_weight);
	
	return (weight_color);
}


bool
valid_carmen_ackerman_traj_point(carmen_ackerman_traj_point_t world_point, carmen_map_t *map)
{
	if (!map)
		return (false);

	int x = carmen_round((world_point.x - map->config.x_origin) / map->config.resolution);
	int y = carmen_round((world_point.y - map->config.y_origin) / map->config.resolution);

	if (x < 0 || x >= map->config.x_size || y < 0 || y >= map->config.y_size)
		return (false);

	return (true);
}

GdkImage *
get_annotation_image(char *filename)
{
	if (!carmen_file_exists(filename))
		carmen_die("Image file %s does not exist.\n", filename);

	GError *error = NULL;
	GdkPixbuf *pixbuf = gdk_pixbuf_new_from_file(filename, &error);
	if (pixbuf == NULL)
		carmen_die("Couldn't open %s for reading\n", filename);

	int n_channels = gdk_pixbuf_get_n_channels(pixbuf);
//		if (n_channels != 3)
//			carmen_die("File has alpha channel. carmen_pixbuf_to_map failed\n");

	int x_size = gdk_pixbuf_get_width(pixbuf);
	int y_size = gdk_pixbuf_get_height(pixbuf);
	GdkImage *image = gdk_image_new(GDK_IMAGE_FASTEST, gdk_visual_get_system(), x_size, y_size);

	int rowstride = gdk_pixbuf_get_rowstride(pixbuf);
	guchar *pixels = gdk_pixbuf_get_pixels(pixbuf);

	for (int x_index = 0; x_index < x_size; x_index++)
	{
		for (int y_index = 0; y_index < x_size; y_index++)
		{
			unsigned char r, g, b;
			guchar *p = pixels + y_index * rowstride + x_index * n_channels;
			r = p[0];
			g = p[1];
			b = p[2];
			gdk_image_put_pixel(image, x_index, y_index, b | g << 8 | r << 16);
		}
	}

	return (image);
}


bool
well_behaved_origin_string(carmen_map_config_t config)
{
	for (unsigned int i = 0; i < sizeof(config.origin); i++)
	{
		if (!(isprint(config.origin[i]) || (config.origin[i] == '\0')))
			return (false);
	}
	return (true);
}


namespace View
{
	GtkGui::GtkGui(int argc, char *argv[])
	{
		fused_odometry_position.pose.x = 0.0;
		fused_odometry_position.pose.y = 0.0;
		fused_odometry_position.pose.theta = 0.0;
		fused_odometry_position.map = NULL;

		display_needs_updating = 0;
		time_of_last_redraw = 0;

		plan_tree_p1 = NULL;
		plan_tree_p2 = NULL;

		simulator_objects = NULL;
		people = NULL;
		moving_objects_list = NULL;

		simulator_trueposition.pose.x = 0.0;
		simulator_trueposition.pose.y = 0.0;
		simulator_trueposition.pose.theta = 0.0;
		simulator_trueposition.map = NULL;

		time_of_simulator_update = 0.0;
		last_simulator_update = 0.0;

		update_local_map = 1;
		path = NULL;

		navigator_waypoint_list = NULL;
		waypoint_list_size = 0;

		navigator_goal_list = NULL;
		goal_list_size = 0;

		edited_rddf_goal_list = NULL;
		edited_rddf_goal_size = 0;

		last_navigator_update = 0.0;

		placement_status = NO_PLACEMENT;

		queuePoints = NULL;
		global_view = 0;

		obstacle_avoider_path = NULL;
		obstacle_avoider_path_size = 0;

		oa_motion_plan = NULL;
		oa_motion_plan_size = 0;

		mpp_motion_plan = NULL;
		mpp_motion_plan_size = 0;

		car_panel_gl = car_panel::get_instance(argc, argv);
		
		num_path_points = 0;
		robot_config = NULL;
	}

	GtkGui::~GtkGui()
	{

	}

	void GtkGui::ConfigureMapViewer()
	{
		controls_.map_view = carmen_map_graphics_new_viewer(460, 400, nav_panel_config->initial_map_zoom);

		if (controls_.map_view != NULL)
		{
			carmen_map_graphics_add_motion_event (controls_.map_view, (carmen_graphics_mapview_callback_t)motion_handler);
			carmen_map_graphics_add_button_release_event (controls_.map_view, (carmen_graphics_mapview_callback_t)button_release_handler);
			carmen_map_graphics_add_button_press_event (controls_.map_view, (carmen_graphics_mapview_callback_t)button_press_handler);
			carmen_map_graphics_add_drawing_func (controls_.map_view, (carmen_graphics_mapview_drawing_func_t)draw_robot_objects);
			carmen_map_graphics_add_keyboard_press_event (controls_.map_view, (carmen_graphics_mapview_callback_t)keyboard_press_handler);

			gtk_box_pack_start(GTK_BOX(controls_.box_map_2d), controls_.map_view->map_box, TRUE, TRUE, 0);
		}
	}

	void GtkGui::ConfigureMenu()
	{
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_TrackRobot), nav_panel_config->track_robot);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_DrawPath), nav_panel_config->draw_path);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_DrawWaipoints), nav_panel_config->draw_waypoints);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_DrawRobotWaipoints), nav_panel_config->draw_robot_waypoints);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowParticles), nav_panel_config->show_particles);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowFusedOdometry), nav_panel_config->show_fused_odometry);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowGaussians), nav_panel_config->show_gaussians);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowLaserData), nav_panel_config->show_lasers);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowCommandPlan), nav_panel_config->show_command_plan);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowMPPMotionPlan), nav_panel_config->show_mpp_motion_plan);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowOAMotionPlan), nav_panel_config->show_oa_motion_plan);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowDynamicObjects), nav_panel_config->show_dynamic_objects);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowDynamicPoints), nav_panel_config->show_dynamic_points);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowAnnotations), nav_panel_config->show_annotations);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowLaneMarkings), nav_panel_config->show_lane_markings);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuDisplay_ShowCollisionRange), nav_panel_config->show_collision_range);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSimulatorShowTruePosition), nav_panel_config->show_true_pos);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSimulator_ShowObjects), nav_panel_config->show_simulator_objects);
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuGoals_EditRddfGoals), nav_panel_config->edit_rddf_goals);

		if (strcmp(nav_panel_config->superimposed_map, "None") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_None), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Map), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Map Level1") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_MapLevel1), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Offline Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_OfflineMap), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Utility") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Utility), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Costs") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Costs), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Likelihood") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Likelihood), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Global Likelihood") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_GlobalLikelihood), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Lane") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_Lane), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Remission Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_RemissionMap), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Moving Objects") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_MovingObjects), true);
		else if (strcmp(nav_panel_config->superimposed_map, "Road Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuSuperimposedMaps_RoadMap), true);
		else
			carmen_die("Unknown superimpose_map named \"%s\" set as parameter in the carmen ini file. Exiting...\n", nav_panel_config->superimposed_map);

		if (strcmp(nav_panel_config->map, "Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_Map), true);
		else if (strcmp(nav_panel_config->map, "Map Level1") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_MapLevel1), true);
		else if (strcmp(nav_panel_config->map, "Offline Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_OfflineMap), true);
		else if (strcmp(nav_panel_config->map, "Utility") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_Utility), true);
		else if (strcmp(nav_panel_config->map, "Costs") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_Costs), true);
		else if (strcmp(nav_panel_config->map, "Likelihood") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_Likelihood), true);
		else if (strcmp(nav_panel_config->map, "Global Likelihood") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_GlobalLikelihood), true);
		else if (strcmp(nav_panel_config->map, "Lane") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_Lane), true);
		else if (strcmp(nav_panel_config->map, "Complete Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_CompleteMap), true);
		else if (strcmp(nav_panel_config->map, "Remission Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_RemissionMap), true);
		else if (strcmp(nav_panel_config->map, "Moving Objects") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_MovingObjects), true);
		else if (strcmp(nav_panel_config->map, "Road Map") == 0)
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(controls_.menuMaps_RoadMap), true);
		else
			carmen_die("Unknown map named \"%s\" set as parameter in the carmen ini file. Exiting...\n", nav_panel_config->map);

		if (nav_panel_config->show_particles || nav_panel_config->show_gaussians)
			carmen_localize_ackerman_subscribe_particle_correction_message(&particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_lasers)
			carmen_localize_ackerman_subscribe_sensor_message(&sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_command_plan)
			carmen_obstacle_avoider_subscribe_path_message(&obstacle_avoider_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_oa_motion_plan)
			carmen_obstacle_avoider_subscribe_motion_planner_path_message(&oa_motion_plan_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_mpp_motion_plan)
			carmen_model_predictive_planner_subscribe_motion_plan_message(&mpp_motion_plan_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_dynamic_points)
			carmen_mapper_subscribe_virtual_laser_message(&virtual_laser_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_annotations)
			carmen_rddf_subscribe_annotation_message(&rddf_annotation_msg, NULL, CARMEN_SUBSCRIBE_LATEST);

		if (nav_panel_config->show_lane_markings)
			carmen_rddf_subscribe_annotation_message(&rddf_annotation_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}

	void GtkGui::navigator_graphics_initialize(int argc, char **argv, carmen_localize_ackerman_globalpos_message *msg,
			carmen_robot_config_t *robot_conf_param, carmen_polygon_config_t *poly_config_param,
			carmen_navigator_config_t *nav_conf_param, carmen_navigator_panel_config_t *nav_panel_conf_param)
	{
		GdkGLConfig *glconfig;
		GtkBuilder  *builder;
		GError      *error = NULL;
		char *carmen_home_path, glade_path[1000], annotation_image_filename[1000];

		gtk_init(&argc, &argv);
		gtk_gl_init(&argc, &argv);

		//Try double-buffered visual
		glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH | GDK_GL_MODE_DOUBLE));

		if (glconfig == NULL)
		{
			g_print ("*** Cannot find the double-buffered visual.\n");
			g_print ("*** Trying single-buffered visual.\n");

			// Try single-buffered visual
			glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH));

			if (glconfig == NULL)
				g_print ("*** No appropriate OpenGL-capable visual found.\n");
		}

		// Create new GtkBuilder object
		builder = gtk_builder_new();
		carmen_home_path = getenv("CARMEN_HOME");

		if (!carmen_home_path)
		{
			printf("CARMEN_HOME is not configured\n");
			exit(1);
		}

		sprintf(glade_path, "%s/data/gui/navigator_gui2.glade", carmen_home_path);
		if (!gtk_builder_add_from_file(builder, glade_path, &error))
		{
			g_warning("%s", error->message);
			g_free(error);
		}

		for (int i = 0; i < NUM_RDDF_ANNOTATION_TYPES; i++)
			for (int j = 0; j < NUM_RDDF_ANNOTATION_CODES; j++)
				annotation_image[i][j] = NULL;

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/end_point_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_END_POINT_AREA][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/intervention3_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/pedestrian_2_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/pedestrian_2_15_red.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK][RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/stop-line_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/stop_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_STOP][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/barrier_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_BARRIER][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/bump_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_BUMP][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_neutral_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_red_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT][RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_green_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT][RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_yellow_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT][RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_neutral_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT][RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_light_stop_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_turn_right_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN][RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_turn_left_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN][RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_go_straight_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN][RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_off_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN][RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_5_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_5] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_10_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_10] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_15_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_15] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_20_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_20] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_30_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_30] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_40_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_40] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_60_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_60] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_80_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_80] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_100_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_100] = get_annotation_image(annotation_image_filename);
		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/traffic_sign_110_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_SPEED_LIMIT][RDDF_ANNOTATION_CODE_SPEED_LIMIT_110] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/dynamic_stop_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_DYNAMIC][RDDF_ANNOTATION_CODE_DYNAMIC_STOP] = get_annotation_image(annotation_image_filename);

		sprintf(annotation_image_filename, "%s/data/gui/annotations_images/place_15.png", carmen_home_path);
		annotation_image[RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST][RDDF_ANNOTATION_CODE_NONE] = get_annotation_image(annotation_image_filename);

		controls_.main_window  = GTK_WIDGET(gtk_builder_get_object(builder, "mainWindow" ));
		controls_.drawArea = GTK_WIDGET(gtk_builder_get_object(builder, "drawingArea"));
		controls_.drawAreaCarPanel = GTK_WIDGET(gtk_builder_get_object(builder, "drawingAreaCarPanel"));
		controls_.box_map_2d = GTK_HBOX(gtk_builder_get_object(builder, "boxMap2D" ));

		controls_.menuDisplay_TrackRobot = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_TrackRobot" ));
		controls_.menuDisplay_DrawPath = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_DrawPath" ));
		controls_.menuDisplay_DrawRobotWaipoints = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_DrawRobotWaipoints" ));
		controls_.menuDisplay_DrawWaipoints = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_DrawWaipoints" ));
		controls_.menuDisplay_ShowCommandPlan = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowCommandPlan" ));
		controls_.menuDisplay_ShowDynamicObjects = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowDynamicObjects" ));
		controls_.menuDisplay_ShowDynamicPoints = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowDynamicPoints" ));
		controls_.menuDisplay_ShowAnnotations = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowAnnotations" ));
		controls_.menuDisplay_ShowLaneMarkings = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowLaneMarkings" ));
		controls_.menuDisplay_ShowCollisionRange = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowCollisionRange" ));
		controls_.menuDisplay_ShowFusedOdometry = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowFusedOdometry" ));
		controls_.menuDisplay_ShowGaussians = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowGaussians" ));
		controls_.menuDisplay_ShowLaserData = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowLaserData" ));
		controls_.menuDisplay_ShowLateralOffset = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowLateralOffset" ));
		controls_.menuDisplay_ShowOAMotionPlan = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowOAMotionPlan" ));
		controls_.menuDisplay_ShowMPPMotionPlan = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowMPPMotionPlan" ));
		controls_.menuDisplay_ShowParticles = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuDisplay_ShowParticles" ));

		controls_.menuSimulatorShowTruePosition = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSimulatorShowTruePosition" ));
		controls_.menuSimulator_ShowObjects = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSimulator_ShowObjects" ));

		controls_.menuGoals_EditRddfGoals = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuGoals_EditRddfGoals" ));

		controls_.menuSuperimposedMaps_None = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_None" ));
		controls_.menuSuperimposedMaps_Map = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_Map" ));
		controls_.menuSuperimposedMaps_MapLevel1 = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_MapLevel1" ));
		controls_.menuSuperimposedMaps_OfflineMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_OfflineMap" ));
		controls_.menuSuperimposedMaps_Utility = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_Utility" ));
		controls_.menuSuperimposedMaps_Costs = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_Costs" ));
		controls_.menuSuperimposedMaps_Likelihood = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_Likelihood" ));
		controls_.menuSuperimposedMaps_GlobalLikelihood = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_GlobalLikelihood" ));
		controls_.menuSuperimposedMaps_Lane = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_Lane" ));
		controls_.menuSuperimposedMaps_RemissionMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_RemissionMap" ));
		controls_.menuSuperimposedMaps_MovingObjects = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_MovingObjects" ));
		controls_.menuSuperimposedMaps_RoadMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuSuperimposedMaps_RoadMap" ));

		controls_.menuMaps_Map = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_Map" ));
		controls_.menuMaps_MapLevel1 = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_MapLevel1" ));
		controls_.menuMaps_OfflineMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_OfflineMap" ));
		controls_.menuMaps_Utility = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_Utility" ));
		controls_.menuMaps_Costs = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_Costs" ));
		controls_.menuMaps_Likelihood = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_Likelihood" ));
		controls_.menuMaps_GlobalLikelihood = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_GlobalLikelihood" ));
		controls_.menuMaps_Lane = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_Lane" ));
		controls_.menuMaps_CompleteMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_CompleteMap" ));
		controls_.menuMaps_RemissionMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_RemissionMap" ));
		controls_.menuMaps_MovingObjects = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_MovingObjects" ));
		controls_.menuMaps_RoadMap = GTK_CHECK_MENU_ITEM(gtk_builder_get_object(builder, "menuMaps_RoadMap" ));

		controls_.comboGoalSource = GTK_COMBO_BOX(gtk_builder_get_object(builder, "comboGoalSource" ));
		controls_.comboState = GTK_COMBO_BOX(gtk_builder_get_object(builder, "comboState" ));
		controls_.comboFollowLane = GTK_COMBO_BOX(gtk_builder_get_object(builder, "comboFollowLane" ));
		controls_.comboParking = GTK_COMBO_BOX(gtk_builder_get_object(builder, "comboParking" ));

		controls_.labelStatusMap = GTK_LABEL(gtk_builder_get_object(builder, "labelStatusMap" ));
		controls_.labelFusedOdometry = GTK_LABEL(gtk_builder_get_object(builder, "labelFusedOdometry" ));
		controls_.labelOrigin = GTK_LABEL(gtk_builder_get_object(builder, "labelOrigin" ));
		controls_.labelRobot = GTK_LABEL(gtk_builder_get_object(builder, "labelRobot" ));
		controls_.labelVelocity = GTK_LABEL(gtk_builder_get_object(builder, "labelVelocity" ));
		controls_.labelGoal = GTK_LABEL(gtk_builder_get_object(builder, "labelGoal" ));
		controls_.labelGridCell = GTK_LABEL(gtk_builder_get_object(builder, "labelGridCell" ));
		controls_.labelValue = GTK_LABEL(gtk_builder_get_object(builder, "labelValue" ));
		controls_.labelDistTraveled = GTK_LABEL(gtk_builder_get_object(builder, "labelDistTraveled" ));
		controls_.labelGlobalPosTimeStamp = GTK_LABEL(gtk_builder_get_object(builder, "labelGlobalPosTimeStamp" ));
		controls_.labelLowLevelState = GTK_LABEL(gtk_builder_get_object(builder, "labelLowLevelState" ));
		controls_.labelTrafficSignState = GTK_LABEL(gtk_builder_get_object(builder, "labelTrafficSignState" ));

		controls_.labelNavConTimestamp = GTK_LABEL(gtk_builder_get_object(builder, "labelNavConTimestamp" ));
		controls_.buttonSyncMode = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "buttonSyncMode" ));
		controls_.buttonNextTick = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "buttonNextTick" ));

		controls_.buttonGo = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "buttonGo" ));
		GdkColor color;
		gdk_color_parse ("red", &color);
		gtk_widget_modify_bg(GTK_WIDGET(controls_.buttonGo), GTK_STATE_NORMAL, &color);

		controls_.buttonRecord = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "buttonRecord" ));
		gdk_color_parse ("yellow", &color);
		gtk_widget_modify_bg(GTK_WIDGET(controls_.buttonRecord), GTK_STATE_NORMAL, &color);
		gdk_color_parse ("green", &color);
		gtk_widget_modify_bg(GTK_WIDGET(controls_.buttonRecord), GTK_STATE_ACTIVE, &color);

		carmen_graphics_setup_colors();
		robot_colour  = DEFAULT_ROBOT_COLOUR;
		goal_colour	  = DEFAULT_GOAL_COLOUR;
		path_colour	  = DEFAULT_PATH_COLOUR;
		tree_colour   = DEFAULT_TREE_COLOUR;
		people_colour = DEFAULT_PEOPLE_COLOUR;

		this->InitializePathVector();

		nav_panel_config = nav_panel_conf_param;

		if ((nav_panel_config->initial_map_zoom < 1.0) || (nav_panel_config->initial_map_zoom > 100.0))
			nav_panel_config->initial_map_zoom = 100.0;

		this->ConfigureMenu();
		this->ConfigureMapViewer();

		// Connect signals
		gtk_builder_connect_signals(builder, this);

		// Redraws
		gtk_container_set_reallocate_redraws (GTK_CONTAINER(controls_.main_window), TRUE);

		// Add OpenGL-capability to drawArea.
		if(!gtk_widget_set_gl_capability (controls_.drawArea, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE))
			return;

		// Add OpenGL-capability to drawAreaCarPanel.
		if(!gtk_widget_set_gl_capability (controls_.drawAreaCarPanel, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE))
			return;

		// Register idle function
		g_idle_add (on_drawArea_idle, this);

		// Show window. All other widgets are automatically shown by GtkBuilder
		gtk_widget_show_all(controls_.main_window);

		// Destroy builder, since we don't need it anymore
		g_object_unref( G_OBJECT( builder ) );

		for (int index = 0; index < GRADIENT_COLORS; index++)
		{
			RedBlueGradient[index] =
					carmen_graphics_add_color_rgb(255 - 255 * index / (double)GRADIENT_COLORS,
							0, 255 * index / (double)GRADIENT_COLORS);
		}

		simulator_hidden = 1;
		globalpos = msg;

		robot_config = robot_conf_param;
		poly_config	 = poly_config_param;
		nav_config	 = nav_conf_param;

		cursor_pos.map = NULL;

		is_filming = 0;
		filming_timeout = 0;

		if (strcmp(nav_panel_config->superimposed_map, "None") == 0)
			; // Do nothing
		else if (strcmp(nav_panel_config->superimposed_map, "Map") == 0)
		{
			carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
			navigator_get_map(CARMEN_NAVIGATOR_MAP_v, 1);
		}
		else if (strcmp(nav_panel_config->superimposed_map, "Map Level1") == 0)
			navigator_get_map(CARMEN_NAVIGATOR_MAP_LEVEL1_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Offline Map") == 0)
			navigator_get_map(CARMEN_OFFLINE_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Utility") == 0)
			navigator_get_map(CARMEN_NAVIGATOR_UTILITY_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Costs") == 0)
			navigator_get_map(CARMEN_COST_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Likelihood") == 0)
			navigator_get_map(CARMEN_LOCALIZE_LMAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Global Likelihood") == 0)
			navigator_get_map(CARMEN_LOCALIZE_GMAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Lane") == 0)
			navigator_get_map(CARMEN_LANE_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Complete Map") == 0)
			navigator_get_map(CARMEN_COMPLETE_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Remission Map") == 0)
			navigator_get_map(CARMEN_REMISSION_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Moving Objects") == 0)
			navigator_get_map(CARMEN_MOVING_OBJECTS_MAP_v, 1);
		else if (strcmp(nav_panel_config->superimposed_map, "Road Map") == 0)
			navigator_get_map(CARMEN_ROAD_MAP_v, 1);
		else
			carmen_die("Unknown superimpose_map named \"%s\" set as parameter in the carmen ini file. Exiting...\n", nav_panel_config->superimposed_map);

		if (strcmp(nav_panel_config->map, "Map") == 0)
		{
			carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
			navigator_get_map(CARMEN_NAVIGATOR_MAP_v, 0);
		}
		else if (strcmp(nav_panel_config->map, "Map Level1") == 0)
			navigator_get_map(CARMEN_NAVIGATOR_MAP_LEVEL1_v, 1);
		else if (strcmp(nav_panel_config->map, "Offline Map") == 0)
			navigator_get_map(CARMEN_OFFLINE_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Utility") == 0)
			navigator_get_map(CARMEN_NAVIGATOR_UTILITY_v, 0);
		else if (strcmp(nav_panel_config->map, "Costs") == 0)
			navigator_get_map(CARMEN_COST_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Likelihood") == 0)
			navigator_get_map(CARMEN_LOCALIZE_LMAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Global Likelihood") == 0)
			navigator_get_map(CARMEN_LOCALIZE_GMAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Lane") == 0)
			navigator_get_map(CARMEN_LANE_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Complete Map") == 0)
			navigator_get_map(CARMEN_COMPLETE_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Remission Map") == 0)
			navigator_get_map(CARMEN_REMISSION_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Moving Objects") == 0)
			navigator_get_map(CARMEN_MOVING_OBJECTS_MAP_v, 0);
		else if (strcmp(nav_panel_config->map, "Road Map") == 0)
			navigator_get_map(CARMEN_ROAD_MAP_v, 0);
		else
			carmen_die("Unknown map named \"%s\" set as parameter in the carmen ini file. Exiting...\n", nav_panel_config->map);
	}

	void
	GtkGui::InitializePathVector()
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

	void
	GtkGui::label_autonomy_button(char *str)
	{
		GtkWidget *label;

		label = GTK_BIN(this->controls_.buttonGo)->child;
		gtk_label_set_text(GTK_LABEL(label), str);
	}

	void
	GtkGui::set_distance_traveled(carmen_point_t robot_pose, double velocity)
	{
		char buffer[2048];
		static bool first_time = true;
		static double dist_traveled;
		static carmen_point_t previous_robot_pose;

		if (first_time)
		{
			dist_traveled = 0.0;
			previous_robot_pose = robot_pose;
			first_time = false;
		}
		else
		{
			if (velocity > 0.2)
			{
				dist_traveled += carmen_distance(&robot_pose, &previous_robot_pose);
				previous_robot_pose = robot_pose;
			}
		}

		sprintf(buffer, "Dist. Traveled: %'.3lf (Km)", dist_traveled / 1000.0);
		gtk_label_set_text(GTK_LABEL(this->controls_.labelDistTraveled), buffer);
	}

	void
	GtkGui::navigator_graphics_update_display(carmen_traj_point_p new_robot,
			carmen_localize_ackerman_globalpos_message *current_globalpos,
			carmen_ackerman_traj_point_t *new_goal,
			int					autonomous)
	{
		char   buffer[2048];
		double robot_distance = 0.0, goal_distance = 0.0;
		carmen_world_point_t new_robot_w;
		static int previous_width = 0, previous_height = 0;
		double	   delta_angle;
		double adjust_distance;

		if (!this->controls_.map_view->internal_map)
			return;

		if (current_globalpos)
			globalpos = current_globalpos;

		adjust_distance = carmen_fmax
				(this->controls_.map_view->internal_map->config.x_size / (double)this->controls_.map_view->port_size_x,
						this->controls_.map_view->internal_map->config.y_size / (double)this->controls_.map_view->port_size_y);

		adjust_distance *= this->controls_.map_view->internal_map->config.resolution *
				(this->controls_.map_view->zoom / 100.0);
		adjust_distance *= 10;


		if (new_robot)
			robot_traj = *new_robot;

		new_robot_w.pose.x	   = robot_traj.x;
		new_robot_w.pose.y	   = robot_traj.y;
		new_robot_w.pose.theta = robot_traj.theta;
		new_robot_w.map = this->controls_.map_view->internal_map;

		robot_distance	= carmen_distance_world(&new_robot_w, &(this->controls_.map_view->centre));

		if (nav_panel_config->track_robot &&
				((robot_distance > adjust_distance) ||
						(previous_width != this->controls_.map_view->image_widget->allocation.width) ||
						(previous_height != this->controls_.map_view->image_widget->allocation.height)))
			carmen_map_graphics_adjust_scrollbars(this->controls_.map_view, &robot);

		robot_distance = carmen_distance_world(&new_robot_w, &robot);
		delta_angle	   = carmen_normalize_theta(new_robot_w.pose.theta - robot.pose.theta);

		robot = new_robot_w;

		if (new_goal)
		{
			goal_distance = DIST2D_P(new_goal, &goal);
			goal = *new_goal;
		}
		else
			goal_distance = 0.0;

		previous_width	= this->controls_.map_view->image_widget->allocation.width;
		previous_height = this->controls_.map_view->image_widget->allocation.height;

		if ((robot_distance > 1.0) || (goal_distance > 1.0) || (fabs(delta_angle) > carmen_degrees_to_radians(0.01)))
			display_needs_updating = 1;

		if (!freeze_status)
		{
			sprintf(buffer, "Robot: %.2f, %.2f, %2.3f (%3.2f)", robot.pose.x,
					robot.pose.y, robot.pose.theta, carmen_radians_to_degrees(robot.pose.theta));
			gtk_label_set_text(GTK_LABEL(this->controls_.labelRobot), buffer);

			sprintf(buffer, "Velocity: %5.1f km/h (%.2f m/s), %.2f %s", robot_traj.t_vel * 3.6, robot_traj.t_vel,
					carmen_radians_to_degrees(robot_traj.r_vel), (nav_panel_config->use_ackerman ? "deg" : "deg/s"));
			gtk_label_set_text(GTK_LABEL(this->controls_.labelVelocity), buffer);

			sprintf(buffer, "Goal: %.2f, %.2f, %.3f (%.2f deg) (%.2lf Km/h)", goal.x, goal.y, goal.theta, carmen_radians_to_degrees(goal.theta), 3.6 * goal.v);
			gtk_label_set_text(GTK_LABEL(this->controls_.labelGoal), buffer);

			set_distance_traveled(robot.pose, robot_traj.t_vel);

			sprintf(buffer, "globalpos timestamp: %lf", globalpos->timestamp);
			gtk_label_set_text(GTK_LABEL(this->controls_.labelGlobalPosTimeStamp), buffer);
		}

		last_navigator_update = carmen_get_time();

		sprintf(buffer, "NavCon timestamp: %lf", last_navigator_update);
		gtk_label_set_text(GTK_LABEL(this->controls_.labelNavConTimestamp), buffer);

		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_goal_list(carmen_ackerman_traj_point_t *goal_list, int size)
	{
//		int RDDF_MAX_SIZE = 10000;
		int i;
		
		if (this->navigator_goal_list != NULL)
			free(this->navigator_goal_list);

		this->goal_list_size = size;

		this->navigator_goal_list = (carmen_world_point_t *) malloc(sizeof(carmen_world_point_t) * goal_list_size);

		for (i = 0; i < size; i++)
		{
			if (valid_carmen_ackerman_traj_point(goal_list[i], this->controls_.map_view->internal_map))
			{
				this->navigator_goal_list[i].pose.x = goal_list[i].x;
				this->navigator_goal_list[i].pose.y = goal_list[i].y;
				this->navigator_goal_list[i].pose.theta = goal_list[i].theta;
				this->navigator_goal_list[i].map = this->controls_.map_view->internal_map;
			}
			else
			{
				this->goal_list_size = i;
				break;
			}
		}

//		if (this->edited_rddf_goal_list == NULL)
//			this->edited_rddf_goal_list = (carmen_world_point_t*) malloc (sizeof(carmen_world_point_t) * RDDF_MAX_SIZE);
//		if (this->original_rddf_goal_list == NULL)
//			this->original_rddf_goal_list = (carmen_world_point_t*) malloc (sizeof(carmen_world_point_t) * RDDF_MAX_SIZE);
//		update_edited_rddf_goal_list();

		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_waypoint_list(carmen_ackerman_traj_point_t* waypoint_list, int size)
	{
		int i;

		if (this->navigator_waypoint_list != NULL)
			free(this->navigator_waypoint_list);

		this->waypoint_list_size = size;

		this->navigator_waypoint_list = (carmen_world_point_t*) malloc(sizeof(carmen_world_point_t) * waypoint_list_size);

		for(i = 0; i < size; i++)
		{
			this->navigator_waypoint_list[i].pose.x = waypoint_list[i].x;
			this->navigator_waypoint_list[i].pose.y = waypoint_list[i].y;
			this->navigator_waypoint_list[i].pose.theta = waypoint_list[i].theta;
			this->navigator_waypoint_list[i].map = this->controls_.map_view->internal_map;
		}

		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_plan(carmen_ackerman_traj_point_p new_plan, int plan_length)
	{
		int index;

		if (this->controls_.map_view->internal_map == NULL)
			return;

		if (this->path != NULL)
		{
			free(this->path);
			this->path = NULL;
		}

		num_path_points = plan_length;

		if (plan_length > 0)
		{
			this->path = (carmen_world_point_t *) calloc (plan_length, sizeof(carmen_world_point_t));
			carmen_test_alloc(this->path);
			carmen_verbose("Got path of length %d\n", plan_length);

			for (index = 0; index < num_path_points; index++)
			{
				this->path[index].pose.x	   = new_plan[index].x;
				this->path[index].pose.y	   = new_plan[index].y;
				this->path[index].pose.theta = new_plan[index].theta;
				this->path[index].map = this->controls_.map_view->internal_map;
				carmen_verbose("%.1f %.1f\n", this->path[index].pose.x,
						this->path[index].pose.y);
			}
		}
		else
		{
			num_path_points = 0;
		}

		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_path(carmen_ackerman_traj_point_t *new_path, int path_length, int path_id)
	{
		int i, path_index;

		if (this->controls_.map_view->internal_map == NULL)
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
			path_vector[path_index] = (carmen_world_point_t *) calloc(path_vector_size[path_index], sizeof(carmen_world_point_t));
			carmen_test_alloc(path_vector[path_index]);

			for (i = 0; i < path_vector_size[path_index]; i++)
			{
				path_vector[path_index][i].pose.x	   = new_path[i].x;
				path_vector[path_index][i].pose.y	   = new_path[i].y;
				path_vector[path_index][i].pose.theta = new_path[i].theta;
				path_vector[path_index][i].map = this->controls_.map_view->internal_map;
			}
		}

		do_redraw();
	}

	int	 flags = 0;

	void
	GtkGui::navigator_graphics_set_flags(carmen_navigator_map_t type)
	{
		switch (type)
		{
		case CARMEN_NAVIGATOR_MAP_v:
			flags = 0;
			break;

		case CARMEN_NAVIGATOR_MAP_LEVEL1_v:
			flags = 0;
			break;

		case CARMEN_NAVIGATOR_ENTROPY_v:
			flags = CARMEN_GRAPHICS_RESCALE;
			break;

		case CARMEN_NAVIGATOR_COST_v:
			flags = CARMEN_GRAPHICS_RESCALE;
			break;

		case CARMEN_NAVIGATOR_UTILITY_v:
			flags = CARMEN_GRAPHICS_RESCALE | CARMEN_GRAPHICS_INVERT;
			break;

		case CARMEN_LOCALIZE_LMAP_v:
			flags = CARMEN_GRAPHICS_LOG_ODDS | CARMEN_GRAPHICS_INVERT;
			break;

		case CARMEN_LOCALIZE_GMAP_v:
			flags = CARMEN_GRAPHICS_LOG_ODDS | CARMEN_GRAPHICS_INVERT;
			break;

		case CARMEN_LANE_MAP_v:
			flags = 0;
			break;

		case CARMEN_COST_MAP_v:
			flags = 0;
			break;

		case CARMEN_OFFLINE_MAP_v:
			flags = 0;
			break;

		case CARMEN_COMPLETE_MAP_v:
			flags = 0;
			break;

		case CARMEN_REMISSION_MAP_v:
			flags = CARMEN_GRAPHICS_REMOVE_MINUS_ONE | CARMEN_GRAPHICS_INVERT | CARMEN_GRAPHICS_ENHANCE_CONTRAST;// | CARMEN_GRAPHICS_RESCALE;// | CARMEN_GRAPHICS_ENHANCE_CONTRAST;
			// Configuracao antiga para ver mapas de remission (antes de GPX)
//			flags = CARMEN_GRAPHICS_REMOVE_MINUS_ONE | CARMEN_GRAPHICS_INVERT | CARMEN_GRAPHICS_RESCALE;// | CARMEN_GRAPHICS_ENHANCE_CONTRAST;
			break;

		case CARMEN_MOVING_OBJECTS_MAP_v:
			flags = CARMEN_GRAPHICS_REMOVE_MINUS_ONE | CARMEN_GRAPHICS_INVERT | CARMEN_GRAPHICS_RESCALE;// | CARMEN_GRAPHICS_ENHANCE_CONTRAST;
//			flags = 0; // CARMEN_GRAPHICS_LOG_ODDS | CARMEN_GRAPHICS_INVERT;
			break;

		case CARMEN_ROAD_MAP_v:
			flags = CARMEN_GRAPHICS_ROAD_CONTRAST;
			break;

		default:
			flags = 0;
			return;
		}
	}

	void
	GtkGui::navigator_graphics_display_map(carmen_map_t *new_map, carmen_navigator_map_t type)
	{
		display = type;
		navigator_graphics_set_flags(type);

		carmen_map_graphics_add_map(this->controls_.map_view, new_map, flags);
	}

	void
	GtkGui::navigator_graphics_redraw_superimposed()
	{
		carmen_map_graphics_redraw_superimposed(this->controls_.map_view);
	}

	int
	GtkGui::navigator_graphics_update_map()
	{
		if (update_local_map)
			return 1;
		return 0;
	}

	void
	GtkGui::navigator_graphics_change_map(carmen_map_p new_map)
	{
		char buffer[2048];
		memset(buffer, 0, 2048);

		carmen_map_graphics_add_map(this->controls_.map_view, new_map, flags);

		if (people)
			people->length = 0;

		if (!this->freeze_status)
		{
			if (well_behaved_origin_string(new_map->config))
			{
				sprintf(buffer, "Status: %s", new_map->config.origin);
				gtk_label_set_text(GTK_LABEL(this->controls_.labelStatusMap), buffer);
			}

			sprintf(buffer, "Origin: (%ld, %ld)", (long int) new_map->config.x_origin, (long int) new_map->config.y_origin);
			gtk_label_set_text(GTK_LABEL(this->controls_.labelOrigin), buffer);

			//Descomentar para gravacao automatica
//			if(!log_first_it)
//				navigator_graphics_start_recording_message_received();

		}
	}


	void
	GtkGui::navigator_graphics_update_simulator_truepos(carmen_point_t truepose)
	{
		this->time_of_simulator_update = carmen_get_time();

		if (this->simulator_hidden)
			this->simulator_hidden = 0;

		simulator_trueposition.pose = truepose;
		simulator_trueposition.map	= this->controls_.map_view->internal_map;
		last_simulator_update  = carmen_get_time();
		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_simulator_objects(int num_objects, carmen_traj_point_t *objects_list)
	{
		int i;

		if (simulator_objects == NULL)
		{
			if (num_objects == 0)
				return;

			simulator_objects = carmen_list_create(sizeof(carmen_traj_point_t), num_objects);
		}

		simulator_objects->length = 0;

		for (i = 0; i < num_objects; i++)
			carmen_list_add(simulator_objects, objects_list + i);

		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_moving_objects(int num_point_clouds, moving_objects_tracking_t *moving_objects_tracking)
	{
		int i;
		if (moving_objects_list == NULL)
		{
			if (num_point_clouds == 0)
				return;

			moving_objects_list = carmen_list_create(sizeof(moving_objects_tracking_t), num_point_clouds);
		}

		moving_objects_list->length = 0;

		for (i = 0; i < num_point_clouds; i++)
			carmen_list_add(moving_objects_list, moving_objects_tracking + i);

		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_parking_assistant_goal(carmen_point_t pose)
	{
		this->parking_assistant_goal.map = this->controls_.map_view->internal_map;
		this->parking_assistant_goal.pose = pose;
		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_plan_to_draw(int path_size, carmen_ackerman_traj_point_t *path)
	{
		static int first = 1;
		static int allocated_size = 0;

		if (first)
		{
			this->candidate_path_size = (int *) calloc (1, sizeof(int));
			this->canditade_path = (carmen_world_point_t **) calloc (1, sizeof(carmen_world_point_t *));
			this->canditade_path[0] = (carmen_world_point_t *) calloc (path_size, sizeof(carmen_world_point_t));
			allocated_size = path_size;
			first = 0;
		}

		if (path_size != allocated_size)
		{
			this->canditade_path[0] = (carmen_world_point_t *) realloc (this->canditade_path[0], path_size * sizeof(carmen_world_point_t));
			allocated_size = path_size;
		}

		this->num_candidate_path = 1;
		this->candidate_path_size[0] = path_size;

		for (int j = 0; j < path_size; j++)
		{
			this->canditade_path[0][j].pose.x	   = path[j].x;
			this->canditade_path[0][j].pose.y	   = path[j].y;
			this->canditade_path[0][j].pose.theta  = path[j].theta;
			this->canditade_path[0][j].map = this->controls_.map_view->internal_map;
		}

		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_plan_tree(
			carmen_ackerman_traj_point_p p1,
			carmen_ackerman_traj_point_p p2,
			int *mask,
			int plan_tree_length,
			carmen_ackerman_traj_point_t paths[500][100],
			int path_size[100],
			int num_path)
	{
		int index;

		if (this->controls_.map_view->internal_map == NULL)
			return;

		if (this->plan_tree_p1 != NULL)
		{
			free(this->plan_tree_p1);
			this->plan_tree_p1 = NULL;
		}

		if (this->plan_tree_p2 != NULL)
		{
			free(this->plan_tree_p2);
			this->plan_tree_p2 = NULL;
		}

		num_plan_tree_points = plan_tree_length;

		this->mask = mask;

		if (plan_tree_length > 0)
		{
			this->plan_tree_p1 = (carmen_world_point_t *)calloc(plan_tree_length, sizeof(carmen_world_point_t));
			carmen_test_alloc(plan_tree_p1);
			this->plan_tree_p2 = (carmen_world_point_t *)calloc(plan_tree_length, sizeof(carmen_world_point_t));
			carmen_test_alloc(this->plan_tree_p2);
			carmen_verbose("Got path of length %d\n", plan_tree_length);

			for (index = 0; index < num_plan_tree_points; index++)
			{
				this->plan_tree_p1[index].pose.x	   = p1[index].x;
				this->plan_tree_p1[index].pose.y	   = p1[index].y;
				this->plan_tree_p1[index].pose.theta   = p1[index].theta;
				this->plan_tree_p1[index].map = this->controls_.map_view->internal_map;

				this->plan_tree_p2[index].pose.x	   = p2[index].x;
				this->plan_tree_p2[index].pose.y	   = p2[index].y;
				this->plan_tree_p2[index].pose.theta = p2[index].theta;
				this->plan_tree_p2[index].map = this->controls_.map_view->internal_map;
			}
		}
		else
		{
			num_plan_tree_points = 0;
		}

		if (this->canditade_path != NULL)
		{
			for (int i = 0; i < this->num_candidate_path; i++)
				free(this->canditade_path[i]);
			free(this->canditade_path);
			this->canditade_path = NULL;
		}

		this->num_candidate_path = num_path;
		this->candidate_path_size = path_size;

		if (num_path > 0)
		{
			this->canditade_path = (carmen_world_point_t **) calloc(num_path, sizeof(carmen_world_point_t *));

			for (int i = 0; i < num_path; i++)
			{
				this->canditade_path[i] = (carmen_world_point_t *) calloc(path_size[i], sizeof(carmen_world_point_t));


				for (int j = 0; j < path_size[i]; j++)
				{
					this->canditade_path[i][j].pose.x	   = paths[i][j].x;
					this->canditade_path[i][j].pose.y	   = paths[i][j].y;
					this->canditade_path[i][j].pose.theta  = paths[i][j].theta;
					this->canditade_path[i][j].map = this->controls_.map_view->internal_map;
				}
			}
		}

		display_needs_updating = 1;
		do_redraw();
	}

	void
	GtkGui::navigator_graphics_update_fused_odometry(carmen_point_t fused_odometry_pose)
	{
		char buffer[2048];

		fused_odometry_position.map = this->controls_.map_view->internal_map;
		fused_odometry_position.pose = fused_odometry_pose;

		sprintf(buffer, "Fused Odom: %5.1f m, %5.1f m, %6.2f", fused_odometry_pose.x,
				fused_odometry_pose.y, carmen_radians_to_degrees(fused_odometry_pose.theta));
		gtk_label_set_text(GTK_LABEL(this->controls_.labelFusedOdometry), buffer);

		do_redraw();
	}

	int
	GtkGui::get_algorithm_code(char *algorithm_name)
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

		return code;
	}

	int
	GtkGui::get_goal_source_code(char* goal_source_name)
	{
		if (strcmp(goal_source_name, "User Goal") == 0)
			return 0;
		else if(strcmp(goal_source_name, "Rddf Goal") == 0)
			return 1;

		return -1;
	}

	int
	GtkGui::get_state_code(char* state_name)
	{
		if (strcmp(state_name, "Following Lane") == 0)
			return 0;
		else if(strcmp(state_name, "Parking") == 0)
			return 1;
		else if(strcmp(state_name, "Human Intervention") == 0)
			return 2;

		return -1;
	}

	void
	GtkGui::navigator_graphics_update_behavior_selector_state(carmen_behavior_selector_state_message msg)
	{
		this->behavior_selector_active = 1;
		this->goal_source = msg.goal_source;

		if((int)msg.following_lane_algorithm != get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)this->controls_.comboFollowLane)))
			gtk_combo_box_set_active((GtkComboBox*)this->controls_.comboFollowLane, msg.following_lane_algorithm);

		if((int)msg.parking_algorithm != get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)this->controls_.comboParking)))
			gtk_combo_box_set_active((GtkComboBox*)this->controls_.comboParking, msg.parking_algorithm);

		if((int)msg.goal_source != get_goal_source_code(gtk_combo_box_get_active_text((GtkComboBox*)this->controls_.comboGoalSource)))
			gtk_combo_box_set_active((GtkComboBox*)this->controls_.comboGoalSource, msg.goal_source);

		//TODO: pode ter erro na conversao pra gtkwidget
		if (msg.goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
			gtk_widget_set_sensitive((GtkWidget *) this->controls_.comboState, 1);
		else
			gtk_widget_set_sensitive((GtkWidget *) this->controls_.comboState, 0);

		if((int)msg.state != get_state_code(gtk_combo_box_get_active_text((GtkComboBox*)this->controls_.comboState)))
			gtk_combo_box_set_active((GtkComboBox*)this->controls_.comboState, msg.state);

		char buffer[2048];
		strcpy(buffer, "Low Level State: ");
		strcat(buffer, get_low_level_state_name(msg.low_level_state));

//		strcpy(ndata.low_level_state_navigator,buffer);

		gtk_label_set_text(GTK_LABEL(this->controls_.labelLowLevelState), buffer);
	}

	void
	GtkGui::navigator_graphics_update_traffic_sign_state(carmen_rddf_traffic_sign_message msg)
	{
		char buffer[2048];
		sprintf(buffer, "Traffic Sign State: %s", get_traffic_sign_state_name(msg.traffic_sign_state));
		if (msg.traffic_sign_state != RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF && msg.traffic_sign_data != 0.0)
		{
			int actual_state = (msg.traffic_sign_data > 0.0) ? RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT : RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT;
			if (msg.traffic_sign_state == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT)
				sprintf(buffer, "%s/%s", buffer, get_traffic_sign_state_name(actual_state));
			sprintf(buffer, "%s   %.3lf (%.3lf deg/m)", buffer, msg.traffic_sign_data, carmen_radians_to_degrees(msg.traffic_sign_data));
		}
//		strcpy(ndata.traffic_sign_state_navigator,buffer);
		gtk_label_set_text(GTK_LABEL(this->controls_.labelTrafficSignState), buffer);
	}

	void
	GtkGui::navigator_graphics_reset()
	{
		robot_colour  = DEFAULT_ROBOT_COLOUR;
		goal_colour	  = DEFAULT_GOAL_COLOUR;
		path_colour	  = DEFAULT_PATH_COLOUR;
		tree_colour	  = DEFAULT_TREE_COLOUR;
		people_colour = DEFAULT_PEOPLE_COLOUR;
//
//		assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot", -1,
//				DEFAULT_TRACK_ROBOT);
//		assign_variable("/ui/MainMenu/DisplayMenu/DrawPath", -1,
//				DEFAULT_DRAW_PATH);
//		assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints", -1,
//				DEFAULT_DRAW_WAYPOINTS);
//		assign_variable("/ui/MainMenu/DisplayMenu/DrawRobotWaypoints", -1,
//				DEFAULT_DRAW_ROBOT_WAYPOINTS);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowLateralOffset", -1,
//				DEFAULT_SHOW_LATERAL_OFFSET);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles", -1,
//				DEFAULT_SHOW_PARTICLES);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowFusedOdometry", -1,
//				DEFAULT_SHOW_FUSED_ODOMETRY);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians", -1,
//				DEFAULT_SHOW_GAUSSIANS);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData", -1,
//				DEFAULT_SHOW_LASER);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowCommandPath", -1,
//				DEFAULT_SHOW_COMMAND_PATH);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowMotionPath", -1,
//				DEFAULT_SHOW_MOTION_PATH);
//		assign_variable("/ui/MainMenu/DisplayMenu/ShowDynamicObjects", -1,
//				DEFAULT_SHOW_DYNAMIC_OBJECTS);
//		assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition", -1,
//				DEFAULT_SHOW_SIMULATOR);
//		assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects", -1,
//				DEFAULT_SHOW_TRACKED_OBJECTS);
	}

	void
	GtkGui::navigator_graphics_display_config (char *attribute, int value, char *new_status_message __attribute__ ((unused)))
	{
//		if (strncmp(attribute, "robot colour", 12) == 0)
//		{
//			if (value == -1)
//			{
//				robot_colour = DEFAULT_ROBOT_COLOUR;
//			}
//			else
//			{
//				assign_colour(&robot_colour, value);
//			}
//		}
//		else if (strncmp(attribute, "goal colour", 11) == 0)
//		{
//			if (value == -1)
//			{
//				goal_colour = DEFAULT_GOAL_COLOUR;
//			}
//			else
//			{
//				assign_colour(&goal_colour, value);
//			}
//		}
//		else if (strncmp(attribute, "goal colour", 11) == 0)
//		{
//			if (value == -1)
//			{
//				tree_colour = DEFAULT_TREE_COLOUR;
//			}
//			else
//			{
//				assign_colour(&tree_colour, value);
//			}
//		}
//		else if (strncmp(attribute, "path colour", 11) == 0)
//		{
//			if (value == -1)
//			{
//				path_colour = DEFAULT_PATH_COLOUR;
//			}
//			else
//			{
//				assign_colour(&path_colour, value);
//			}
//		}
//		else if (strncmp(attribute, "people colour", 11) == 0)
//		{
//			if (value == -1)
//			{
//				path_colour = DEFAULT_PATH_COLOUR;
//			}
//			else
//			{
//				assign_colour(&people_colour, value);
//			}
//		}
//		else if (strncmp(attribute, "track robot", 11) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/TrackRobot",
//					value, DEFAULT_TRACK_ROBOT);
//		}
//		else if (strncmp(attribute, "draw path", 20) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/DrawPath",
//					value, DEFAULT_DRAW_PATH);
//		}
//		else if (strncmp(attribute, "draw waypoints", 14) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/DrawWaypoints",
//					value, DEFAULT_DRAW_WAYPOINTS);
//		}
//		else if (strncmp(attribute, "draw robot waypoints", 14) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/DrawRobotWaypoints",
//					value, DEFAULT_DRAW_ROBOT_WAYPOINTS);
//		}
//		else if (strncmp(attribute, "show particles", 14) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowParticles",
//					value, DEFAULT_SHOW_PARTICLES);
//		}
//		else if (strncmp(attribute, "show fused odometry", 19) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowFusedOdometry",
//					value, DEFAULT_SHOW_FUSED_ODOMETRY);
//		}
//		else if (strncmp(attribute, "show gaussians", 14) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowGaussians",
//					value, DEFAULT_SHOW_GAUSSIANS);
//		}
//		else if (strncmp(attribute, "show laser", 10) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowLaserData",
//					value, DEFAULT_SHOW_LASER);
//		}
//		else if (strncmp(attribute, "Show Command Path", 17) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowCommandPath",
//					value, DEFAULT_SHOW_COMMAND_PATH);
//		}
//		else if (strncmp(attribute, "Show Command Path", 50) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowMotionPath",
//					value, DEFAULT_SHOW_MOTION_PATH);
//		}
//		else if (strncmp(attribute, "Show Dynamic Objects", 20) == 0)
//		{
//			assign_variable("/ui/MainMenu/DisplayMenu/ShowDynamicObjects",
//					value, DEFAULT_SHOW_DYNAMIC_OBJECTS);
//		}
//		else if (strncmp(attribute, "show simulator", 14) == 0)
//		{
//			assign_variable("/ui/MainMenu/SimulatorMenu/SimShowTruePosition",
//					value, DEFAULT_SHOW_SIMULATOR);
//		}
//		else if (strncmp(attribute, "show tracked objects", 20) == 0)
//		{
//			assign_variable("/ui/MainMenu/SimulatorMenu/SimShowObjects",
//					value, DEFAULT_SHOW_TRACKED_OBJECTS);
//		}
//
		carmen_map_graphics_redraw(this->controls_.map_view);
	}

	void
	GtkGui::save_to_image(GtkMapViewer* mapv)
	{
		if(!log_first_it)
		{
			char log_date[100];
			memset(log_buffer,'\0',1000*sizeof(char));
			memset(log_path,'\0',(255)*sizeof(char));
			memset(log_date,'\0',(100)*sizeof(char));

			time_t t = time(NULL);
			struct tm tm = *localtime(&t);
			snprintf(log_date, sizeof(log_date), "%d-%d-%d_%d:%d:%d",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
			snprintf(log_buffer, sizeof(log_buffer), "/dados/navigator_gui2_log/%s", log_date);
			mkdir(log_buffer, 0777);
			snprintf(log_buffer, sizeof(log_buffer), "/dados/navigator_gui2_log/%s/pictures", log_date);
			mkdir(log_buffer, 0777);
			snprintf(log_path, sizeof(log_path), "/dados/navigator_gui2_log/%s/log_file.txt", log_date);

			file_log = fopen(log_path, "a");

			if(NULL == file_log)
			{
				printf("Erro ao abrir o arquivo log_file.txt no método save_to_image (gtk_gui.cpp)\n");
				exit(1);
			}
			snprintf(log_path, sizeof(log_path), "/dados/navigator_gui2_log/%s/", log_date);

			log_first_it = 1;
			log_counter = 0;
		}

		GdkPixbuf * pixbuf = gdk_pixbuf_get_from_drawable(NULL, mapv->drawing_pixmap, NULL, 0, 0, 0, 0, -1, -1);

		if(NULL != pixbuf)
		{
			GError *error;
			error = NULL;

			snprintf(log_buffer,sizeof(log_buffer),"%spictures/%d.jpg", log_path, log_counter);
			gdk_pixbuf_save(pixbuf, log_buffer, "jpeg", &error, NULL);

			snprintf(log_buffer, sizeof(log_buffer),"%d#%s#%s#%s#%s#%s#%s#%s#%s#%s#%s#Go Button = %d#", log_counter, gtk_label_get_text(GTK_LABEL(this->controls_.labelOrigin)), gtk_label_get_text(GTK_LABEL(this->controls_.labelRobot)), gtk_label_get_text(GTK_LABEL(this->controls_.labelFusedOdometry)), gtk_label_get_text(GTK_LABEL(this->controls_.labelVelocity)), gtk_label_get_text(GTK_LABEL(this->controls_.labelGoal)), gtk_label_get_text(GTK_LABEL(this->controls_.labelGridCell)), gtk_label_get_text(GTK_LABEL(this->controls_.labelValue)), gtk_label_get_text(GTK_LABEL(this->controls_.labelGlobalPosTimeStamp)), gtk_label_get_text(GTK_LABEL(this->controls_.labelLowLevelState)), gtk_label_get_text(GTK_LABEL(this->controls_.labelTrafficSignState)), log_button_go );
			fprintf(file_log,"%s\n", log_buffer);
			log_counter++;
			g_clear_object(&pixbuf);
			g_clear_object(&error);
		}

	}

	void
	GtkGui::do_redraw(void)
	{
		if (this->display_needs_updating &&
				((carmen_get_time() - this->time_of_last_redraw > 0.025) || ALWAYS_REDRAW))
		{
			carmen_map_graphics_redraw(this->controls_.map_view);
			if(log_map_is_ready)
				save_to_image(this->controls_.map_view);
			this->time_of_last_redraw	   = carmen_get_time();
			this->display_needs_updating = 0;
		}
	}

	void
	GtkGui::navigator_graphics_start(char *path)
	{
		this->map_path = path;

		gtk_main();
	}

	void
	GtkGui::navigator_graphics_add_ipc_handler(GdkInputFunction handle_ipc)
	{
		carmen_graphics_update_ipc_callbacks(handle_ipc);
	}

	int
	GtkGui::placing_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
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

	int
	GtkGui::orienting_robot_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
	{
		GdkCursor *cursor;
		double angle;

		if ((placement_status == ORIENTING_ROBOT) ||
			((placement_status == NO_PLACEMENT) && (((event->button == 2) && (event->state & GDK_CONTROL_MASK)) ||
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

			return TRUE;
		}

		return FALSE;
	}

	int
	GtkGui::placing_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventButton *event)
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

	void
	GtkGui::add_goal_to_internal_list(carmen_world_point_t goal)
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


	int
	GtkGui::orienting_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
	{
		GdkCursor *cursor;

		if (placement_status == ORIENTING_GOAL)
		{
			placement_status = NO_PLACEMENT;
			goal_temp.pose.theta = atan2(world_point->pose.y - goal_temp.pose.y,
					world_point->pose.x - goal_temp.pose.x);

			cursor = gdk_cursor_new(GDK_LEFT_PTR);
			gdk_window_set_cursor(the_map_view->image_widget->window, cursor);

			if (!behavior_selector_active || goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
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

	int
	GtkGui::placing_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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

	int
	GtkGui::orienting_person_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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
					(carmen_simulator_ackerman_object_t) CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT);
			cursor = gdk_cursor_new(GDK_LEFT_PTR);
			gdk_window_set_cursor(the_map_view->image_widget->window, cursor);
			return TRUE;
		}
		return FALSE;

	}

	int
	GtkGui::orienting_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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

	int
	GtkGui::placing_simulator_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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


	int
	GtkGui::selecting_final_region_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point)
	{

		if (placement_status == SELECTING_FINAL_REGION)
		{
			carmen_map_t* map;
			carmen_point_t pose;
			pose = world_point->pose;

			robot_temp.pose = pose;
			navigator_update_robot(&robot_temp);

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


	void
	GtkGui::save_rddf_to_file(char *rddf_filename)
	{
		carmen_rddf_play_save_rddf_to_file(rddf_filename, edited_rddf_goal_list, edited_rddf_goal_size);
	}


	void
	GtkGui::load_rddf_from_file(char *rddf_filename)
	{
		edited_rddf_goal_list = carmen_rddf_play_load_rddf_from_file(rddf_filename, &edited_rddf_goal_size);
	}


	carmen_rddf_waypoint*
	GtkGui::find_near_rddf_point(carmen_world_point_t *world_point)
	{
		double MAX_DISTANCE = 4.0;
		double min_distance = 9999.0;
		double distance = 0.0;
		int near_waypoint_index = -1;
		near_rddf_point_index = 0;

		for (int i = 0; i < edited_rddf_goal_size; i++)
		{
			distance = sqrt(pow(world_point->pose.x - edited_rddf_goal_list[i].pose.x, 2) + pow(world_point->pose.y - edited_rddf_goal_list[i].pose.y, 2));

			if (distance < min_distance)
			{
				min_distance = distance;
				near_waypoint_index = i;
				near_rddf_point_index = near_waypoint_index;
			}
		}

		if (min_distance < MAX_DISTANCE)
			return &(edited_rddf_goal_list[near_waypoint_index]);

		return NULL;
	}

	int
	GtkGui::original_rdff_list_contains(carmen_world_point_t world_point)
	{
//		carmen_rddf_waypoint rddf_point;
//
//		for(int i = 0; i < edited_rddf_goal_size; i++)
//		{
//			rddf_point = original_rddf_goal_list[i];
//
//			if((world_point.pose.x == rddf_point.pose.x) &&
//			   (world_point.pose.y == rddf_point.pose.y) &&
//			   (world_point.pose.theta == rddf_point.pose.theta))
//			{
//				return TRUE;
//			}
//		}

		return FALSE;
	}

	void
	GtkGui::update_edited_rddf_goal_list()
	{
//		for (int i = 0; i < this->num_path_points; i++)
//		{
//			if (!original_rdff_list_contains(this->path[i]))
//			{
//				this->original_rddf_goal_list[edited_rddf_goal_size] = this->path[i];
//				this->edited_rddf_goal_list[edited_rddf_goal_size] = this->path[i];
//				edited_rddf_goal_size++;
//			}
//		}
	}

	int
	GtkGui::select_near_rddf_point(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point)
	{
		if (nav_panel_config->edit_rddf_goals)
		{
			placement_status = EDITING_NEAR_RDDF;
			near_rddf_point = find_near_rddf_point(world_point);

			if (near_rddf_point)
				return TRUE;
			else
				return FALSE;

			do_redraw();
		}

		return FALSE;
	}

	void
	GtkGui::release_near_rddf_point()
	{
		if (nav_panel_config->edit_rddf_goals)
		{
			placement_status = NO_PLACEMENT;
			near_rddf_point = NULL;
			near_rddf_point_index = -1;

			do_redraw();
		}
	}

	void
	GtkGui::delete_current_rddf_point()
	{
		if (nav_panel_config->edit_rddf_goals &&
			placement_status == EDITING_NEAR_RDDF &&
			near_rddf_point != NULL &&
			near_rddf_point_index != -1)
		{
			if ((near_rddf_point_index == 0) && (edited_rddf_goal_size == 1))
			{
				return; // Cannot delete the last one of a list with only one
			}
			else if (near_rddf_point_index == (edited_rddf_goal_size - 1)) // The last in the list
			{
				edited_rddf_goal_size--;
				near_rddf_point_index--;
				near_rddf_point = &edited_rddf_goal_list[near_rddf_point_index];
			}
			else
			{
				for (int i = near_rddf_point_index; i < edited_rddf_goal_size - 1; i++)
					edited_rddf_goal_list[i] = edited_rddf_goal_list[i + 1];

				edited_rddf_goal_size--;
				near_rddf_point = &edited_rddf_goal_list[near_rddf_point_index];
			}

			do_redraw();
		}
	}

	int
	GtkGui::placing_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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

	int
	GtkGui::orienting_final_goal_action(GtkMapViewer *the_map_view, carmen_world_point_t *world_point)
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

	int
	GtkGui::selecting_near_waypoint_action(GtkMapViewer *the_map_view __attribute__ ((unused)), carmen_world_point_t *world_point)
	{

		if ( (placement_status == SELECTING_NEAR_WAYPOINT) )
		{
			carmen_rddf_publish_end_point_message(1, world_point->pose);

			placement_status = NO_PLACEMENT;
			return TRUE;
		}

		return FALSE;
	}

	void
	GtkGui::update_point(pointers *reached)
	{
//		queuePoints->curr = reached->next;
//		navigator_set_goal(queuePoints->curr->point.pose.x, queuePoints->curr->point.pose.y, queuePoints->curr->point.pose.theta);
//
//		pointers *point = queuePoints->begin;
//		queuePoints->begin = point->next;
//		free(point);
//
//		navigator_start_moving();
	}

	int
	GtkGui::received_robot_pose(void)
	{
		if (!behavior_selector_active || goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		{
			if ((queuePoints != NULL) && (queuePoints->begin != NULL) && (GTK_TOGGLE_BUTTON(controls_.buttonGo)->active))
			{
				pointers *reached = queuePoints->curr;

				if ((reached != NULL) && (reached->next != NULL))
				{
					if (carmen_distance(&queuePoints->curr->point.pose, &robot.pose) < 0.5) // 0.5 m // @@@ Alberto: Isso deveria ser controlado pelo navigator, nao pela interface
						update_point(reached);
				}
			}
		}

		return (robot.map != NULL);
	}

	/* draw functions */

	void
	GtkGui::draw_fused_odometry_pose(GtkMapViewer *the_map_view)
	{
		draw_robot_shape(the_map_view, &fused_odometry_position, TRUE, &carmen_green);
		draw_robot_shape(the_map_view, &fused_odometry_position, FALSE, &carmen_black);

		draw_orientation_mark(the_map_view, &fused_odometry_position);
	}

	void
	GtkGui::draw_parking_assistant_goal(GtkMapViewer *the_map_view)
	{
		draw_robot_shape(the_map_view, &parking_assistant_goal, TRUE, &carmen_red);
		draw_robot_shape(the_map_view, &parking_assistant_goal, FALSE, &carmen_black);

		draw_orientation_mark(the_map_view, &parking_assistant_goal);
	}
	
	void
	GtkGui::draw_particles(GtkMapViewer *the_map_view, double pixel_size)
	{
		int index;
		carmen_world_point_t particle, final_point;
		static GdkColor *color_gradient = NULL;

		if (color_gradient == NULL)
			color_gradient = build_color_gradient();

		if (!nav_panel_config->show_particles)
			return;

		if (particle_msg.particles != NULL)
		{
			int *weight_color = compute_particle_weight_color(particle_msg.particles, particle_msg.num_particles);
			
			for (index = 0; index < particle_msg.num_particles; index++)
			{
				particle.pose.x = particle_msg.particles[index].x;
				particle.pose.y = particle_msg.particles[index].y;
				particle.pose.theta = particle_msg.particles[index].theta;
				particle.map	= the_map_view->internal_map;

				final_point = particle;
				final_point.pose.x = final_point.pose.x + cos(final_point.pose.theta) * 5.0;
				final_point.pose.y = final_point.pose.y + sin(final_point.pose.theta) * 5.0;

				carmen_map_graphics_draw_line(the_map_view, &carmen_yellow, &particle, &final_point);
			}
			for (index = 0; index < particle_msg.num_particles; index++)
			{
				particle.pose.x = particle_msg.particles[index].x;
				particle.pose.y = particle_msg.particles[index].y;
				particle.pose.theta = particle_msg.particles[index].theta;
				particle.map	= the_map_view->internal_map;

				carmen_map_graphics_draw_circle(the_map_view, &(color_gradient[weight_color[index]]), TRUE, &particle, pixel_size);
			}
			free(weight_color);
		}
	}

	void
	GtkGui::draw_gaussians(GtkMapViewer *the_map_view)
	{
		carmen_world_point_t mean;

		if (!nav_panel_config->show_gaussians || (particle_msg.particles == NULL))
			return;

		mean = robot;
		mean.pose.x		= globalpos->globalpos.x;
		mean.pose.y		= globalpos->globalpos.y;
		mean.pose.theta = globalpos->globalpos.theta;

		carmen_map_graphics_draw_ellipse(the_map_view, &carmen_black, &mean, carmen_square(globalpos->globalpos_std.x),
				globalpos->globalpos_xy_cov, carmen_square(globalpos->globalpos_std.y), 4);
	}

	void
	GtkGui::draw_lasers(GtkMapViewer *the_map_view, double pixel_size)
	{
		double dot_size;
		int	   index;
		carmen_world_point_t particle;
		double angle;

		dot_size = 3 * pixel_size;

		if (!nav_panel_config->show_lasers)
			return;

		particle = robot;

		for (index = 0; index < sensor_msg.num_readings; index += sensor_msg.laser_skip)
		{
			// finale: doesn't this assume a 180 fov?
			// angle = sensor_msg.pose.theta - M_PI_2 +
			//  index / (double)(sensor_msg.num_readings - 1) * M_PI;
			angle = sensor_msg.pose.theta - sensor_msg.config.fov / 2 +
					index / (double)(sensor_msg.num_readings - 1) * sensor_msg.config.fov;


			particle.pose.x = sensor_msg.pose.x + sensor_msg.range[index] * cos(angle);
			particle.pose.y = sensor_msg.pose.y + sensor_msg.range[index] * sin(angle);

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

	void
	GtkGui::draw_robot(GtkMapViewer *the_map_view)
	{
		if (!nav_panel_config->show_particles && !nav_panel_config->show_gaussians)
			draw_robot_shape(the_map_view, &robot, TRUE, &robot_colour);

		if (!nav_panel_config->show_gaussians)
			draw_robot_shape(the_map_view, &robot, FALSE, &carmen_black);

		draw_orientation_mark(the_map_view, &robot);
	}

	void
	GtkGui::draw_plan_tree(GtkMapViewer *the_map_view, double pixel_size)
	{
		int index;
		//carmen_world_point_t path_x_1, path_x_2;

		for (index = 1; index < num_plan_tree_points; index++)
		{
			if (plan_tree_p1->map == NULL)
				break;

			if (plan_tree_p2->map == NULL)
				break;

			//carmen_map_graphics_draw_line(the_map_view, &tree_colour, plan_tree_p1 + index, plan_tree_p2 + index);
			//		time_of_simulator_update = carmen_get_time();

			carmen_map_graphics_draw_line(the_map_view, &carmen_yellow, &plan_tree_p1[index], &plan_tree_p2[index]);
			carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE, &plan_tree_p1[index], pixel_size);
			carmen_map_graphics_draw_circle(the_map_view, &robot_colour, TRUE, &plan_tree_p2[index], pixel_size);
		}
	}

	void
	GtkGui::draw_goal_list(GtkMapViewer	*the_map_view,
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
		if (!behavior_selector_active || goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		{
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
		}

		//draw navigator goal list
		for (i = 0; i < goal_list_size; i++)
		{
			draw_robot_shape(the_map_view, &navigator_goal_list[i], TRUE, &goal_colour);
			draw_robot_shape(the_map_view, &navigator_goal_list[i], FALSE, &carmen_black);
			draw_orientation_mark(the_map_view, &navigator_goal_list[i]);
		}

		// draw rddf goals
		if (nav_panel_config->edit_rddf_goals)
		{
			for (i = 0; i < edited_rddf_goal_size; i++)
			{
				carmen_world_point_t world_point;
				world_point.pose = edited_rddf_goal_list[i].pose;
				world_point.map = the_map_view->internal_map;

				if (i != near_rddf_point_index)
				{
					draw_robot_shape(the_map_view, &world_point, TRUE, &people_colour);
					draw_robot_shape(the_map_view, &world_point, FALSE, &carmen_black);
					draw_orientation_mark(the_map_view, &world_point);
				}
				else
				{
					draw_robot_shape(the_map_view, &world_point, TRUE, &goal_colour);
					draw_robot_shape(the_map_view, &world_point, FALSE, &carmen_black);
					draw_orientation_mark(the_map_view, &world_point);
				}
			}
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
	GtkGui::draw_simulated_robot(GtkMapViewer *the_map_view)
	{
		if (!nav_panel_config->show_true_pos || (simulator_trueposition.map == NULL))
			return;

		draw_robot_shape(the_map_view, &simulator_trueposition, TRUE, &carmen_blue);
		draw_robot_shape(the_map_view, &simulator_trueposition, FALSE, &carmen_black);

		draw_orientation_mark(the_map_view, &simulator_trueposition);
	}

	void
	GtkGui::draw_simulated_objects(GtkMapViewer *the_map_view)
	{
		int index;
		carmen_world_point_t particle;
		carmen_traj_point_t *simulator_object;
		double circle_size;

		if (nav_panel_config->show_simulator_objects)
		{
			circle_size = robot_config->width / 2.0;

			particle.map = the_map_view->internal_map;

			if (simulator_objects)
			{
				for (index = 0; index < simulator_objects->length; index++)
				{
					simulator_object = (carmen_traj_point_t *) carmen_list_get(simulator_objects, index);
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

	void
	GtkGui::draw_moving_objects(GtkMapViewer *the_map_view)
	{
		int index;
		moving_objects_tracking_t* moving_objects_tracking;

		if (nav_panel_config->show_dynamic_objects)
		{
			//particle.map = the_map_view->internal_map;

			if (moving_objects_list)
			{
				for (index = 0; index < moving_objects_list->length; index++)
				{
					moving_objects_tracking = (moving_objects_tracking_t *) carmen_list_get(moving_objects_list, index);

					carmen_world_point_t wp[4];
					carmen_world_point_t *location = (carmen_world_point_t *) malloc(sizeof(carmen_world_point_t));

					double width2, length2;

					location->pose.theta = moving_objects_tracking->moving_objects_pose.orientation.yaw;
					location->pose.x = moving_objects_tracking->moving_objects_pose.position.x;// + fused_odometry_position.pose.x ;
					location->pose.y = moving_objects_tracking->moving_objects_pose.position.y;// + fused_odometry_position.pose.y ;
					location->map = the_map_view->internal_map;

					width2 = moving_objects_tracking->width / 2.0;
					length2 = moving_objects_tracking->length / 2.0;

					wp[0].pose.x = x_coord(-length2, width2, location);
					wp[0].pose.y = y_coord(-length2, width2, location);
					wp[1].pose.x = x_coord(-length2, -width2, location);
					wp[1].pose.y = y_coord(-length2, -width2, location);
					wp[2].pose.x = x_coord(length2, -width2, location);
					wp[2].pose.y = y_coord(length2, -width2, location);
					wp[3].pose.x = x_coord(length2, width2, location);
					wp[3].pose.y = y_coord(length2, width2, location);

					wp[0].map = wp[1].map = wp[2].map = wp[3].map = location->map;

					GdkColor *colour;
					if (strcmp(moving_objects_tracking->model_features.model_name, "pedestrian") == 0)
						colour = &carmen_red;
					else
						colour = &carmen_black;

					if (moving_objects_tracking->model_features.model_id == 'C')
						colour = &carmen_red;
					else if (moving_objects_tracking->model_features.model_id == 'b')
						colour = &carmen_green;
					else if (moving_objects_tracking->model_features.model_id == 'P')
						colour = &carmen_blue;

					carmen_map_graphics_draw_polygon(the_map_view, colour, wp, 4, 0);
				}
			}
		}
	}

	void
	GtkGui::draw_moving_points(GtkMapViewer *the_map_view, double pixel_size)
	{
		if (nav_panel_config->show_dynamic_points)
		{
			int i;
			for (i = 0; i < virtual_laser_msg.num_positions; i++)
			{
				carmen_world_point_t world_point;
				world_point.pose.x = virtual_laser_msg.positions[i].x;
				world_point.pose.y = virtual_laser_msg.positions[i].y;
				world_point.pose.theta = 0.0;
				world_point.map = the_map_view->internal_map;

				carmen_map_graphics_draw_circle(the_map_view, &carmen_colors[(int) virtual_laser_msg.colors[i]], TRUE, &world_point, pixel_size * 2.0);
			}
			if (i != 0)
				display_needs_updating = 1;
		}
	}

	void
	GtkGui::draw_lane_lines(GtkMapViewer *the_map_view, double pixel_size)
	{
		if (lane_markings_msg->lane_vector_size == 0 && lane_markings_msg == NULL)
			return;
		std::vector<carmen_lane_detector_lane_t> left, right;
		if (nav_panel_config->show_lane_markings)
		{
			unsigned int i;
			for (int i = 0; i < lane_markings_msg->lane_vector_size; i++)
			{
				if (lane_markings_msg->lane_vector[i].left == 1)
					left.push_back(lane_markings_msg->lane_vector[i]);
				else
					right.push_back(lane_markings_msg->lane_vector[i]);
			}
			carmen_lane_detector_lane_t anterior_left;
			carmen_lane_detector_lane_t anterior_right;
			if (left.size() > 1)
			{
				for (i = 0; i < left.size(); i++)
				{
					carmen_world_point_t start, end;
					start.pose.x = left[i].lane_segment_position1.x;
					start.pose.y = left[i].lane_segment_position1.y;
					end.pose.x = left[i].lane_segment_position2.x;
					end.pose.y = left[i].lane_segment_position2.y;
					start.map = end.map = the_map_view->internal_map;
					carmen_map_graphics_draw_line(the_map_view, &carmen_red, &start, &end);
					carmen_map_graphics_draw_circle(the_map_view, &carmen_green, TRUE, &start, pixel_size * 2.0);
					carmen_map_graphics_draw_circle(the_map_view, &carmen_green, TRUE, &end, pixel_size * 2.0);
					if (i != 0)
					{
						start.pose.x = anterior_left.lane_segment_position2.x;
						start.pose.y = anterior_left.lane_segment_position2.y;
						end.pose.x = left[i].lane_segment_position1.x;
						end.pose.y = left[i].lane_segment_position1.y;
						carmen_map_graphics_draw_line(the_map_view, &carmen_red, &start, &end);
					}
					anterior_left = left[i];

				}
			}
			if (right.size() > 1)
			{
				for (i = 0; i < right.size(); i++)
				{
					carmen_world_point_t start, end;
					start.pose.x = right[i].lane_segment_position1.x;
					start.pose.y = right[i].lane_segment_position1.y;
					end.pose.x = right[i].lane_segment_position2.x;
					end.pose.y = right[i].lane_segment_position2.y;
					start.map = end.map = the_map_view->internal_map;
					carmen_map_graphics_draw_line(the_map_view, &carmen_red, &end, &start);
					carmen_map_graphics_draw_circle(the_map_view, &carmen_green, TRUE, &start, pixel_size * 2.0);
					carmen_map_graphics_draw_circle(the_map_view, &carmen_green, TRUE, &end, pixel_size * 2.0);
					if (i != 0)
					{
						start.pose.x = anterior_right.lane_segment_position2.x;
						start.pose.y = anterior_right.lane_segment_position2.y;
						if (i == right.size() - 1)
						{
							end.pose.x = right[i].lane_segment_position1.x;
							end.pose.y = right[i].lane_segment_position1.y;
						}else
						{
							end.pose.x = right[i].lane_segment_position2.x;
							end.pose.y = right[i].lane_segment_position2.y;
						}
						carmen_map_graphics_draw_line(the_map_view, &carmen_red, &start, &end);
					}
					anterior_right = right[i];
				}
			}
			display_needs_updating = 1;
			right.clear();
			left.clear();
		}
	}


	void
	format_annotation_description(char *dest_text, char *orig_text)
	{
		static const char *prefix = (char *) "RDDF_PLACE_";
		char *start = orig_text;
		if (strncmp(orig_text, prefix, strlen(prefix)) == 0)
			start += strlen(prefix);
		strcpy(dest_text, start);

		for (char *dest = dest_text; (*dest) != 0; dest++)
		{
			if ((*dest) == '_')
				(*dest) = ' ';
		}
	}


	void
	GtkGui::draw_annotations(GtkMapViewer *the_map_view, double pixel_size)
	{
		carmen_world_point_t world_point;
		char text[2000];

		for (int i = 0; i < rddf_annotation_msg.num_annotations; i++)
		{
			double displacement = poly_config->displacement;//car_config->distance_between_front_and_rear_axles + car_config->distance_between_front_car_and_front_wheels;
			world_point.pose.theta = rddf_annotation_msg.annotations[i].annotation_orientation;
			world_point.pose.x = rddf_annotation_msg.annotations[i].annotation_point.x + displacement * cos(world_point.pose.theta);
			world_point.pose.y = rddf_annotation_msg.annotations[i].annotation_point.y + displacement * sin(world_point.pose.theta);
			world_point.map = the_map_view->internal_map;
//			printf("x %lf, y %lf, theta %lf\n", world_point.pose.x, world_point.pose.y, world_point.pose.theta);

			carmen_world_point_t start, end;
			double theta_left = world_point.pose.theta + M_PI / 2.0;
			start.pose.x = world_point.pose.x + 10.0 * cos(theta_left);
			start.pose.y = world_point.pose.y + 10.0 * sin(theta_left);

			double theta_right = world_point.pose.theta - M_PI / 2.0;
			end.pose.x = world_point.pose.x + 10.0 * cos(theta_right);
			end.pose.y = world_point.pose.y + 10.0 * sin(theta_right);

			start.map = end.map = world_point.map;
			start.pose.theta = end.pose.theta = world_point.pose.theta;

			carmen_map_graphics_draw_line(the_map_view, &carmen_grey, &start, &end);

			GdkImage *image = annotation_image[rddf_annotation_msg.annotations[i].annotation_type][rddf_annotation_msg.annotations[i].annotation_code];
			carmen_map_graphics_draw_image(the_map_view, image, &end, gdk_image_get_width(image), gdk_image_get_height(image));

			if ((rddf_annotation_msg.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) &&
				(rddf_annotation_msg.annotations[i].annotation_point.z > 0.0))
				carmen_map_graphics_draw_circle(the_map_view, &carmen_red, FALSE, &world_point, rddf_annotation_msg.annotations[i].annotation_point.z);

			if ((rddf_annotation_msg.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN) &&
			    (rddf_annotation_msg.annotations[i].annotation_code != RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF) &&
				(rddf_annotation_msg.annotations[i].annotation_point.z != 0.0))
			{
				double radius = 1.0 / fabs(rddf_annotation_msg.annotations[i].annotation_point.z);
				double theta = (rddf_annotation_msg.annotations[i].annotation_point.z > 0.0) ? theta_left : theta_right;
				carmen_world_point_t center = world_point;
				center.pose.x = world_point.pose.x + radius * cos(theta);
				center.pose.y = world_point.pose.y + radius * sin(theta);
				int delta_angle = 180 * 64; /* arbitrary angle in 1/64ths of a degree */
				double max_curve_lenght = 25.0; /* arbitrary length in meters */
				double curve_length = radius * (delta_angle / 64) * (M_PI / 180);
				if (curve_length > max_curve_lenght)
					delta_angle *= (max_curve_lenght / curve_length);
				int start_angle = (theta - M_PI) * (180 / M_PI) * 64;
				if (theta == theta_right)
					start_angle -= delta_angle;
				carmen_map_graphics_draw_arc(the_map_view, &carmen_blue, FALSE, &center, radius, start_angle, delta_angle);
			}

			if (rddf_annotation_msg.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PLACE_OF_INTEREST)
			{
				format_annotation_description(text, rddf_annotation_msg.annotations[i].annotation_description);
				GdkFont *text_font = gdk_font_load("-*-courier-bold-r-normal--0-0-0-0-p-0-iso8859-1");
				carmen_map_graphics_draw_string(the_map_view, &carmen_red, text_font, &world_point, text);
			}
		}

		display_needs_updating = 1;
	}

	void
	GtkGui::draw_placing_animation(GtkMapViewer *the_map_view)
	{
		GdkColor *colour = &carmen_black;
		carmen_world_point_t *draw_point = NULL;

		if ((placement_status != ORIENTING_ROBOT) &&
				(placement_status != ORIENTING_GOAL) &&
				(placement_status != ORIENTING_PERSON) &&
				(placement_status != ORIENTING_FINAL_GOAL) &&
				(placement_status != ORIENTING_SIMULATOR))
			return;

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
			carmen_map_graphics_draw_circle(the_map_view, colour, TRUE, draw_point, 1);
			carmen_map_graphics_draw_circle(the_map_view, &carmen_black, FALSE, draw_point, 1);
			carmen_map_graphics_draw_line(the_map_view, colour, draw_point, &cursor_pos);
		}
	}

	void
	GtkGui::draw_path(carmen_world_point_t *path, int num_path_points, GdkColor path_colour, GdkColor robot_color, GtkMapViewer *the_map_view)
	{
		int index;
		carmen_world_point_t path_x_1, path_x_2;

		if (path == NULL)
			return;

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

	void
	GtkGui::draw_road_velocity_control(GtkMapViewer *the_map_view)
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

	void
	GtkGui::draw_path_vector(GtkMapViewer *the_map_view)
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

	void
	GtkGui::draw_robot_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour)
	{
		if (!robot_config->rectangular)
			draw_differential_shape(the_map_view, location, filled, colour);
		else
			draw_ackerman_shape(the_map_view, location, filled, colour);
	}

	void
	GtkGui::draw_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
	{
		if (!robot_config->rectangular)
			draw_differential_orientation_mark(the_map_view, robot_pose);
		else
			draw_ackerman_orientation_mark(the_map_view, robot_pose);
	}

	void
	GtkGui::draw_path_color(GtkMapViewer *the_map_view, carmen_world_point_t* path, int size, GdkColor *color)
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

	void
	GtkGui::draw_differential_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour)
	{
		double robot_radius;
		robot_radius = robot_config->width / 2.0;

		carmen_map_graphics_draw_circle(the_map_view, colour, filled, location, robot_radius);
	}

	void
	GtkGui::draw_ackerman_shape(GtkMapViewer *the_map_view, carmen_world_point_t *location, int filled, GdkColor *colour)
	{
//		double width2, length, dist_rear_car_rear_wheels;
//
//		dist_rear_car_rear_wheels = car_config->distance_between_rear_car_and_rear_wheels;
//		width2 = robot_config->width / 2;
//		length = robot_config->length;
//
//		wp[0].pose.x = x_coord(-dist_rear_car_rear_wheels, width2, location);
//		wp[0].pose.y = y_coord(-dist_rear_car_rear_wheels, width2, location);
//		wp[1].pose.x = x_coord(-dist_rear_car_rear_wheels, -width2, location);
//		wp[1].pose.y = y_coord(-dist_rear_car_rear_wheels, -width2, location);
//		wp[2].pose.x = x_coord(length - dist_rear_car_rear_wheels, -width2, location);
//		wp[2].pose.y = y_coord(length - dist_rear_car_rear_wheels, -width2, location);
//		wp[3].pose.x = x_coord(length - dist_rear_car_rear_wheels, width2, location);
//		wp[3].pose.y = y_coord(length - dist_rear_car_rear_wheels, width2, location);
//
//		wp[0].map = wp[1].map = wp[2].map  = location->map;

		if (nav_panel_config->show_collision_range)
		{
			carmen_collision_config_t *collision_config = carmen_get_global_collision_config();
			for (int i=0; i < collision_config->n_markers; i++)
			{
				carmen_world_point_t center;
				center.pose.x = x_coord(collision_config->markers[i].x, collision_config->markers[i].y, location);
				center.pose.y = y_coord(collision_config->markers[i].x, collision_config->markers[i].y, location);
				center.map = location->map;
				carmen_map_graphics_draw_circle(the_map_view, colour, filled, &center, collision_config->markers[i].radius);
			}
		}
		else
		{
			carmen_world_point_t wp[poly_config->n_points];
			for (int i=0; i < poly_config->n_points; i++)
			{
				wp[i].pose.x = x_coord(poly_config->points[2*i], poly_config->points[2*i+1], location);
				wp[i].pose.y = y_coord(poly_config->points[2*i], poly_config->points[2*i+1], location);
				wp[i].map = location->map;
			}

			carmen_map_graphics_draw_polygon(the_map_view, colour, wp, poly_config->n_points, filled);
		}
	}

	void
	GtkGui::draw_differential_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
	{
		carmen_world_point_t initial_point, radius;
		initial_point = *robot_pose;

		radius = initial_point;
		radius.pose.x = radius.pose.x + cos(radius.pose.theta) * robot_config->width / 2.0;
		radius.pose.y = radius.pose.y + sin(radius.pose.theta) * robot_config->width / 2.0;

		carmen_map_graphics_draw_line(the_map_view, &carmen_black, &initial_point, &radius);
	}

	void
	GtkGui::draw_ackerman_orientation_mark(GtkMapViewer *the_map_view, carmen_world_point_t *robot_pose)
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

	/* support functions */

	double
	GtkGui::x_coord(double x, double y, carmen_world_point_t *offset)
	{
		return x * cos(offset->pose.theta) - y *sin(offset->pose.theta) + offset->pose.x;
	}

	double
	GtkGui::y_coord(double x, double y, carmen_world_point_t *offset)
	{
		return x * sin(offset->pose.theta) + y *cos(offset->pose.theta) + offset->pose.y;
	}

	void
	GtkGui::change_cursor(GdkColor *fg, GdkColor *bg)
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

		gdk_window_set_cursor(this->controls_.map_view->image_widget->window, cursor);
	}

	void
	GtkGui::execute_decrement_point()
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
//				navigator_stop_moving();
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

	void
	GtkGui::world_point_to_global_world_point(carmen_world_point_t *world_point)
	{
		if (!world_point || !world_point->map)
			return;

		world_point->pose.x += world_point->map->config.x_origin;
		world_point->pose.y += world_point->map->config.y_origin;
	}

	// ##### GL ######## //

	void
	GtkGui::draw_gl_components()
	{
		glScalef(1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		setView();
		drawAxes();
	}

	void
	GtkGui::drawAxes()
	{
		glLineWidth(2.0);

		glColor3d(1.0, 0.0, 0.0);

		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(1.0, 0.0, 0.0);
		glEnd();

		glColor3d(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(0.0, 1.0, 0.0);
		glEnd();

		glColor3d(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 1.0);
		glEnd();

		glLineWidth(1.0);
	}

	void
	GtkGui::setView()
	{
		int width, height;
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		width = 640;
		height = 480;

		glViewport(0,0, height, width);

		gluPerspective(45.0, width/height, 0.01, 10000.0);

		// display camera view
		gluLookAt(10.0,  10.0,  10.0,
				  0.0, 0.0, 0.0,
				  0.0, 0.0, 1.0);
	}

	void
	GtkGui::draw_gl_components_car_panel()
	{
		car_panel_gl->set_view(500, 250);
		car_panel_gl->draw();
	}

	void
	GtkGui::navigator_graphics_go_message_received()
	{
		GdkColor color;

		gdk_color_parse("grey", &color);
		gtk_widget_modify_bg(GTK_WIDGET(this->controls_.buttonGo), GTK_STATE_PRELIGHT, &color);

		gtk_toggle_button_set_active(this->controls_.buttonGo, true);
		GtkWidget *label = GTK_BIN(this->controls_.buttonGo)->child;
		gtk_label_set_text(GTK_LABEL(label), "Stop");
		log_button_go = 1;
	}

	void
	GtkGui::navigator_graphics_stop_message_received()
	{
		GdkColor color;

		gdk_color_parse ("red", &color);
		gtk_widget_modify_bg(GTK_WIDGET(this->controls_.buttonGo), GTK_STATE_PRELIGHT, &color);

		gtk_toggle_button_set_active(this->controls_.buttonGo, false);
		GtkWidget *label = GTK_BIN(this->controls_.buttonGo)->child;
		gtk_label_set_text(GTK_LABEL(label), "Go");
		log_button_go = 0;
	}

	void
	GtkGui::navigator_graphics_start_recording_message_received()
	{


		gtk_toggle_button_set_active(this->controls_.buttonRecord, true);
		GtkWidget *label = GTK_BIN(this->controls_.buttonRecord)->child;
		gtk_label_set_text(GTK_LABEL(label), "Recording");
		log_map_is_ready = 1;

	}

	void
	GtkGui::navigator_graphics_pause_recording_message_received()
	{

		gtk_toggle_button_set_active(this->controls_.buttonRecord, false);
		GtkWidget *label = GTK_BIN(this->controls_.buttonRecord)->child;
		gtk_label_set_text(GTK_LABEL(label), "Record");
		log_map_is_ready = 0;

	}
}
