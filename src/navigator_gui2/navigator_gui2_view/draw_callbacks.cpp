#include "draw_callbacks.h"

extern int record_screen;
extern char place_of_interest[2048];
extern char predefined_route[2048];
extern int predefined_route_code;
extern char mission[2048];
int final_goal_closest_index_in_nearby_lanes;


extern void
mapper_handler(carmen_mapper_map_message *message);

//extern void
//carmen_mapper_compact_map_message_handler(carmen_mapper_compact_map_message *message);


namespace View
{

GtkGui *global_gui = NULL;
int superimposed_is_set = 0;
int first_mission = 1;

//extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void *data)
{
	static double last_time = 0.0;

	double time = carmen_get_time();
	if ((time - last_time) < 0.09)
		return (TRUE);

	GtkGui *gui = static_cast<GtkGui*>(data);

	global_gui = gui;

	if (!GTK_IS_WIDGET(global_gui->controls_.drawArea))
		return TRUE;

	gtk_widget_draw(global_gui->controls_.drawArea, NULL);
	gtk_widget_draw(global_gui->controls_.drawAreaCarPanel, NULL);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	global_gui->draw_gl_components_car_panel();

	last_time = time;

	return (TRUE);
}

//extern "C" G_MODULE_EXPORT
void on_drawingArea_realize (GtkWidget *widget, GtkGui *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

	gdk_gl_drawable_gl_end (gldrawable);
}

//extern "C" G_MODULE_EXPORT
gboolean on_drawingArea_expose_event (GtkWidget       *widget,
                             	     GdkEventExpose  *event __attribute__((unused)),
                                   GtkGui 	 *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context  (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		printf ("error in gdk_gl_drawable_gl_begin\n");
		return FALSE;
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	global_gui->draw_gl_components();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);
	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
void on_drawingAreaCarPanel_realize (GtkWidget *widget, GtkGui *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
		return;

	gdk_gl_drawable_gl_end (gldrawable);
}

//extern "C" G_MODULE_EXPORT
gboolean on_drawingAreaCarPanel_expose_event (GtkWidget       *widget,
                             	     GdkEventExpose  *event __attribute__((unused)),
                                   GtkGui 	 *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context  (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		printf ("error in gdk_gl_drawable_gl_begin\n");
		return FALSE;
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	global_gui->draw_gl_components_car_panel();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);
	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
void on_mainWindow_realize (GtkWidget *widget, GtkGui *gui)
{
	global_gui = gui;
}

//extern "C" G_MODULE_EXPORT
void on_activeMenuQuit(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* data	__attribute__((unused)))
{
	gtk_main_quit();
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Map_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
//		carmen_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
		navigator_get_map(CARMEN_NAVIGATOR_MAP_v, superimposed_is_set);
	}
	else
		carmen_mapper_unsubscribe_map_message((carmen_handler_t) mapper_handler);
//		carmen_mapper_unsubscribe_compact_map_message((carmen_handler_t) carmen_mapper_compact_map_message_handler);
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_MapLevel1_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_NAVIGATOR_MAP_LEVEL1_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_OFFLINE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_RoadMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_ROAD_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Utility_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_NAVIGATOR_UTILITY_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Costs_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_COST_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Likelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LOCALIZE_LMAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_GlobalLikelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LOCALIZE_GMAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Lane_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LANE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_CompleteMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_COMPLETE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_REMISSION_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_MOVING_OBJECTS_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_None_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_NONE_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Map_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
//		carmen_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
		navigator_get_map(CARMEN_NAVIGATOR_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
	else
		carmen_mapper_unsubscribe_map_message((carmen_handler_t) mapper_handler);
//		carmen_mapper_unsubscribe_compact_map_message((carmen_handler_t) carmen_mapper_compact_map_message_handler);
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_MapLevel1_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_NAVIGATOR_MAP_LEVEL1_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_OFFLINE_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_RoadMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_ROAD_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Utility_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_NAVIGATOR_UTILITY_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Costs_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_COST_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Likelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_LOCALIZE_LMAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_GlobalLikelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_LOCALIZE_GMAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Lane_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_LANE_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_REMISSION_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if (gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_MOVING_OBJECTS_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_TrackRobot_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->track_robot = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->robot.map && global_gui->nav_panel_config->track_robot)
		carmen_map_graphics_adjust_scrollbars(global_gui->controls_.map_view, &global_gui->robot);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLateralOffset_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	/*global_gui->nav_panel_config->xxx = Alberto: esta faltando este estado... */ gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawPath_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->draw_path = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawWaipoints_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->draw_waypoints = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawRobotWaipoints_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->draw_robot_waypoints = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowParticles_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_particles = gtk_check_menu_item_get_active(togglebutton);

	if ((global_gui->nav_panel_config->show_particles == 1) && !global_gui->nav_panel_config->show_gaussians)
	{
		carmen_localize_ackerman_subscribe_particle_prediction_message(&global_gui->prediction_particles_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
		carmen_localize_ackerman_subscribe_particle_correction_message(&global_gui->correction_particles_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}
	else if (!global_gui->nav_panel_config->show_particles && !global_gui->nav_panel_config->show_gaussians)
	{
		carmen_localize_ackerman_subscribe_particle_prediction_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
		carmen_localize_ackerman_subscribe_particle_correction_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowFusedOdometry_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_fused_odometry = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowGaussians_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_gaussians = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLaserData_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_lasers = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_lasers)
		carmen_localize_ackerman_subscribe_sensor_message(&global_gui->sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowNearbyLanes_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_nearby_lanes = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_nearby_lanes)
	 	carmen_frenet_path_planner_subscribe_set_of_paths_message(&global_gui->frenet_path_planer_set_of_paths_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
	 	carmen_frenet_path_planner_subscribe_set_of_paths_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowNearbyLanesWidth_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_nearby_lanes_width = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowPathPlans_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	global_gui->nav_panel_config->show_path_plans = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_path_plans)
		carmen_frenet_path_planner_subscribe_set_of_paths_message(&global_gui->frenet_path_planer_set_of_paths_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_frenet_path_planner_subscribe_set_of_paths_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowOAMotionPlan_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_oa_motion_plan = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_oa_motion_plan)
		carmen_obstacle_avoider_subscribe_motion_planner_path_message(&global_gui->oa_motion_plan_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_obstacle_avoider_subscribe_motion_planner_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowMPPMotionPlan_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_mpp_motion_plan = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_mpp_motion_plan)
		carmen_subscribe_message((char *) RRT_PATH_NAME, (char *) RRT_PATH_FMT, &global_gui->mpp_motion_plan_msg_rrt, sizeof(rrt_path_message), NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_unsubscribe_message((char *) RRT_PATH_NAME, NULL);
//		carmen_model_predictive_planner_subscribe_motion_plan_message(&global_gui->mpp_motion_plan_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
//	else
//		carmen_model_predictive_planner_subscribe_motion_plan_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowOffroadPlan_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_offroad_plan = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowCommandPlan_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_command_plan = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_command_plan)
		carmen_obstacle_avoider_subscribe_path_message(&global_gui->obstacle_avoider_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_obstacle_avoider_subscribe_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuGoals_EditRddfGoals_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->edit_rddf_goals = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->load_rddf_from_file((char *) "rddf_edited.txt");

	if (!global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->save_rddf_to_file((char *) "rddf_edited.txt");
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowDynamicObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_dynamic_objects = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowDynamicPoints_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_dynamic_points = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_dynamic_points)
		carmen_mapper_subscribe_virtual_laser_message(&global_gui->virtual_laser_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_mapper_subscribe_virtual_laser_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowAnnotations_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_annotations = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_annotations)
		carmen_rddf_subscribe_annotation_message(&global_gui->rddf_annotation_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_rddf_subscribe_annotation_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLaneMarkings_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_lane_markings = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_lane_markings)
		carmen_lane_subscribe(global_gui->lane_markings_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_lane_subscribe(NULL, NULL, CARMEN_UNSUBSCRIBE);
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowCollisionRange_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_collision_range = gtk_check_menu_item_get_active(togglebutton);
}


//extern "C" G_MODULE_EXPORT
void on_menuSimulatorShowTruePosition_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_true_pos = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_ShowObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_simulator_objects = gtk_check_menu_item_get_active(togglebutton);
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_ClearObjects_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* data	__attribute__((unused)))
{
	carmen_simulator_ackerman_clear_objects();
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddPerson_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_PERSON;
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddLineFollower_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_LINE_FOLLOWER;
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddOtherRobot_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_OTHER_ROBOT;
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddBike_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_BIKE;
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddCar_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_CAR;
}

//extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddTruck_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_orange, &carmen_black);
	global_gui->placement_status = PLACING_TRUCK;
}


//extern "C" G_MODULE_EXPORT
void on_menuStartLocation_GlobalLocalization_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* data	__attribute__((unused)))
{
	carmen_verbose("Global localization\n");
	navigator_update_robot(NULL);
	return;
}

//extern "C" G_MODULE_EXPORT
void on_menuHelp_About_activate(GtkWidget *widget __attribute__((unused)),
					   GdkEvent *event __attribute__((unused)),
					   GtkGui* data	__attribute__((unused)))
{
	printf("on_menuHelp_About_activate\n");
}

////extern "C" G_MODULE_EXPORT
//void on_comboGoalSource_changed(GtkWidget *widget __attribute__((unused)),
//					   GtkGui* gui)
//{
//	if (global_gui)
//		carmen_behavior_selector_set_goal_source((carmen_behavior_selector_goal_source_t)global_gui->get_goal_source_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboGoalSource)));
//}

//extern "C" G_MODULE_EXPORT
void on_comboPlaceOfInterest_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (global_gui)
	{
		global_gui->get_place_of_interest(gtk_combo_box_get_active_text((GtkComboBox *) global_gui->controls_.comboPlaceOfInterest));
//		superimposed_is_set = 0;
//		navigator_get_map(CARMEN_OFFLINE_MAP_v, superimposed_is_set);
	}
}

std::string
read_file_to_string(std::string file_name)
{
	std::ifstream text_file(file_name);
	stringstream buffer;
	buffer << text_file.rdbuf();

	return (buffer.str());
}

//extern "C" G_MODULE_EXPORT
void on_comboMission_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (global_gui)
	{
		global_gui->get_mission(gtk_combo_box_get_active_text((GtkComboBox *) global_gui->controls_.comboMission));
		if (strcmp(mission, "None") != 0)
		{
			printf("%s\n", mission);
			string mission_as_string = read_file_to_string(mission);
			if (mission_as_string.size() == 0)
				exit(printf("Could not read mission file %s\n", mission));
			if(first_mission == 1)
				carmen_user_app_server_publish_execute_mission_message(mission_as_string.c_str(), carmen_get_time());
			else
				{
					carmen_user_app_server_publish_update_mission_message(1, carmen_get_time());
					carmen_user_app_server_publish_execute_mission_message(mission_as_string.c_str(), carmen_get_time());
				}
		}
		// global_gui->get_predefined_route(gtk_combo_box_get_active_text((GtkComboBox *) global_gui->controls_.comboMission));
		// if (strcmp(predefined_route, "None") != 0)
		// 	carmen_route_planner_set_predefined_route(predefined_route, predefined_route_code);
	}
}

//extern "C" G_MODULE_EXPORT
void on_buttonComputeRoute_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->offroad_planner_plan = NULL;

	if (global_gui->final_goal_placed_and_oriented == 1)
	{
		carmen_point_t final_goal_pose;
		final_goal_pose.x = global_gui->final_goal.pose.x;
		final_goal_pose.y = global_gui->final_goal.pose.y;
		final_goal_pose.theta = global_gui->final_goal.pose.theta;
		carmen_route_planner_set_destination(place_of_interest, final_goal_pose);
		global_gui->final_goal_placed_and_oriented = 0;
	}
	else
		carmen_route_planner_set_destination(place_of_interest, global_gui->destination);

	if (global_gui)
	{
		// gtk_combo_box_set_active((GtkComboBox *) global_gui->controls_.comboPredefinedRoute, 0);
		// global_gui->reset_predefined_route();
		gtk_combo_box_set_active((GtkComboBox *) global_gui->controls_.comboMission, 0);
		global_gui->reset_mission();
		global_gui->display_needs_updating = 1;
		global_gui->do_redraw();
	}
}

//extern "C" G_MODULE_EXPORT
void on_comboState_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (global_gui)
		carmen_behavior_selector_set_task((carmen_behavior_selector_task_t)global_gui->get_task_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboState)));
}

//extern "C" G_MODULE_EXPORT
void on_comboFollowRoute_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (global_gui)
		navigator_set_algorithm((carmen_behavior_selector_algorithm_t)global_gui->get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboFollowRoute)), BEHAVIOR_SELECTOR_FOLLOW_ROUTE);
}

//extern "C" G_MODULE_EXPORT
void on_comboParking_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (global_gui)
		navigator_set_algorithm((carmen_behavior_selector_algorithm_t)global_gui->get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboParking)), BEHAVIOR_SELECTOR_PARK);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_fused_odometry_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(0);
//	global_gui->car_panel_gl->set_turn_signal(0);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_robot_ackerman_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(1);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_base_ackerman_motion_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(2);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_base_ackerman_odometry_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(3);
//	global_gui->car_panel_gl->set_turn_signal(1);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_localize_ackerman_globalpos_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(4);
}

//extern "C" G_MODULE_EXPORT
void on_buttonPlaceRobot_clicked(GtkWidget *widget __attribute__((unused)), GtkGui* gui)
{
	global_gui->change_cursor(&carmen_red, &carmen_black);
	global_gui->placement_status = PLACING_ROBOT;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonSetCollisionGeometry_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	carmen_task_manager_publish_set_collision_geometry_message(ENGAGE_GEOMETRY, carmen_get_time());

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonSetDefaultGeometry_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	carmen_task_manager_publish_set_collision_geometry_message(DEFAULT_GEOMETRY, carmen_get_time());

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonGo_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (GTK_TOGGLE_BUTTON(global_gui->controls_.buttonGo)->active)
		navigator_start_moving();
	else
		navigator_stop_moving();

	if (global_gui->global_view)
		global_gui->global_view = 0;
}

//extern "C" G_MODULE_EXPORT
void on_buttonGo_entered(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	GdkColor color;

	if (GTK_TOGGLE_BUTTON(global_gui->controls_.buttonGo)->active)
	{
		gdk_color_parse ("gray", &color);
		gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonGo), GTK_STATE_PRELIGHT, &color);
	}
	else
	{
		gdk_color_parse ("red", &color);
		gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonGo), GTK_STATE_PRELIGHT, &color);
	}
}

//extern "C" G_MODULE_EXPORT
void on_buttonRecord_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	GdkColor color;
	if (GTK_TOGGLE_BUTTON(global_gui->controls_.buttonRecord)->active || record_screen == 1)
	{
		gdk_color_parse ("green", &color);
		gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonRecord), GTK_STATE_NORMAL, &color);
		gui->navigator_graphics_start_recording_message_received();
	}
	else
	{
		gdk_color_parse ("yellow", &color);
		gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonRecord), GTK_STATE_NORMAL, &color);
		gui->navigator_graphics_pause_recording_message_received();
	}

//	if (global_gui->global_view)
//		global_gui->global_view = 0;
}

//extern "C" G_MODULE_EXPORT
/*
void on_buttonRecord_entered(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	GdkColor color;

	if (GTK_TOGGLE_BUTTON(global_gui->controls_.buttonRecord)->active)
		{
			gdk_color_parse ("gray", &color);
			gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonRecord), GTK_STATE_PRELIGHT, &color);
		}
		else
		{
			gdk_color_parse ("green", &color);
			gtk_widget_modify_bg(GTK_WIDGET(global_gui->controls_.buttonRecord), GTK_STATE_PRELIGHT, &color);
		}
}
*/
//extern "C" G_MODULE_EXPORT
void on_buttonPlaceFinalGoal_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	//	gtk_widget_set_sensitive(place_final_goal_button, TRUE);
	// TODO: pode ter erro aqui: gtk_widget_set_sensitive(select_nearest_waypoint_button, TRUE);
	//gtk_widget_set_sensitive(update_map_button, FALSE);
	global_gui->change_cursor(&carmen_yellow, &carmen_black);
	global_gui->placement_status = PLACING_FINAL_GOAL;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonPlaceSimulator_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_red, &carmen_black);
	global_gui->placement_status = PLACING_SIMULATOR;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}


//extern "C" G_MODULE_EXPORT
gint motion_handler(GtkMapViewer *the_map_view, carmen_world_point_t *world_point,
		GdkEventMotion *event __attribute__ ((unused)))
{
	char buffer[1024];
	carmen_map_point_t point;
	carmen_map_t *the_map;

	if (global_gui == NULL || the_map_view == NULL || the_map_view->internal_map == NULL)
		return TRUE;

	the_map = the_map_view->internal_map;

	global_gui->world_point_to_global_world_point(world_point);
	if (carmen_world_to_map(world_point, &point) == -1)
		return TRUE;

	if (!global_gui->freeze_status)
	{
		sprintf(buffer, "Grid Cell: %d, %d  (%.1f, %.1f)", point.x, point.y,
				world_point->pose.x, world_point->pose.y);
		gtk_label_set_text(GTK_LABEL(global_gui->controls_.labelGridCell), buffer);

		if (the_map != NULL)
		{
			int road_contrast = the_map_view->draw_flags & CARMEN_GRAPHICS_ROAD_CONTRAST;
			if (road_contrast)
			{
				road_prob *cell = road_mapper_double_to_prob(&the_map->map[point.x][point.y]);
				sprintf(buffer, "Value: off=%d solid=%d broken=%d center=%d",
						cell->off_road, cell->solid_marking, cell->broken_marking, cell->lane_center);
			}
			else
			{
				sprintf(buffer, "Value: %.4f", the_map->map[point.x][point.y]);
			}

			gtk_label_set_text(GTK_LABEL(global_gui->controls_.labelValue), buffer);
		}
	}

	if ((global_gui->placement_status == ORIENTING_ROBOT) ||
			(global_gui->placement_status == ORIENTING_ROBOT_SEMI_TRAILER) ||
			(global_gui->placement_status == ORIENTING_GOAL) ||
			(global_gui->placement_status == ORIENTING_SIMULATOR) ||
			(global_gui->placement_status == ORIENTING_FINAL_GOAL) ||
			(global_gui->placement_status == ORIENTING_FINAL_GOAL_SEMI_TRAILER) ||
			(global_gui->placement_status == ORIENTING_PERSON) ||
			(global_gui->placement_status == ORIENTING_LINE_FOLLOWER) ||
			(global_gui->placement_status == ORIENTING_OTHER_ROBOT) ||
			(global_gui->placement_status == ORIENTING_BIKE) ||
			(global_gui->placement_status == ORIENTING_CAR) ||
			(global_gui->placement_status == ORIENTING_TRUCK))
	{
		global_gui->cursor_pos = *world_point;
		global_gui->display_needs_updating = 1;
		global_gui->do_redraw();
	}

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
void on_buttonZoomIn_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
//	if (global_gui->display != CARMEN_COMPLETE_MAP_v)
//	{
//		gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
//		return;
//	}

	carmen_map_p offline_map_p = navigator_get_offline_map_pointer();
	if (offline_map_p)
	{
		global_gui->navigator_graphics_change_map(offline_map_p);
		global_gui->navigator_graphics_display_map(offline_map_p, CARMEN_OFFLINE_MAP_v);
	}

//	gdk_window_set_cursor(global_gui->controls_.map_view->image_widget->window, gdk_cursor_new(GDK_BASED_ARROW_DOWN));
//	global_gui->placement_status = SELECTING_FINAL_REGION;
	global_gui->placement_status = NO_PLACEMENT;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}


//extern "C" G_MODULE_EXPORT
void on_buttonZoomOut_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
//	if (global_gui->display != CARMEN_COMPLETE_MAP_v)
//	{
//		gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
//		return;
//	}

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
	carmen_map_p complete_map_p = navigator_get_complete_map_map_pointer();
	if (complete_map_p)
	{
//		superimposed_is_set = 1;
//		navigator_get_map(CARMEN_NONE_v, superimposed_is_set);
//		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);

		global_gui->navigator_graphics_change_map(complete_map_p);
	}
}

//extern "C" G_MODULE_EXPORT
int button_release_handler(GtkMapViewer		   *the_map_view,
		carmen_world_point_t *world_point,
		GdkEventButton	   *event __attribute__ ((unused)))
{
	int rtr;

	global_gui->world_point_to_global_world_point(world_point);

	if (the_map_view->internal_map == NULL)
		return TRUE;

	rtr = global_gui->placing_robot_action(the_map_view, world_point, event);
	if (rtr)
		return TRUE;

	rtr = global_gui->placing_goal_action(the_map_view, world_point, event);
	if (rtr)
		return TRUE;

	rtr = global_gui->placing_person_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->placing_simulator_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_robot_action(the_map_view, world_point, event);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_robot_semi_trailer_action(the_map_view, world_point, event);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_goal_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_person_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_simulator_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->selecting_final_region_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->placing_final_goal_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_final_goal_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->orienting_final_goal_semi_trailer_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->selecting_near_waypoint_action(the_map_view, world_point);
	if (rtr)
		return TRUE;

	rtr = global_gui->select_near_rddf_point(the_map_view, world_point);
	if (rtr)
		return TRUE;

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
int keyboard_press_handler(GtkMapViewer *the_map_view,
		GdkEventKey	   *event)
{
	if (global_gui->placement_status == EDITING_NEAR_RDDF && global_gui->near_rddf_point != NULL)
	{
		double resolution = the_map_view->internal_map->config.resolution / 2.0;

		switch (event->keyval)
		{
			case GDK_Up:
				global_gui->near_rddf_point->pose.y += resolution;
				break;

			case GDK_Down:
				global_gui->near_rddf_point->pose.y -= resolution;
				break;

			case GDK_Left:
				global_gui->near_rddf_point->pose.x -= resolution;
				break;

			case GDK_Right:
				global_gui->near_rddf_point->pose.x += resolution;
				break;

			case GDK_a:
				global_gui->near_rddf_point->pose.theta += carmen_degrees_to_radians(0.5);
				global_gui->near_rddf_point->pose.theta = carmen_normalize_theta(global_gui->near_rddf_point->pose.theta);
				break;

			case GDK_s:
				global_gui->near_rddf_point->pose.theta -= carmen_degrees_to_radians(0.5);
				global_gui->near_rddf_point->pose.theta = carmen_normalize_theta(global_gui->near_rddf_point->pose.theta);
				break;

			case GDK_n:
				global_gui->near_rddf_point_index++;
				if (global_gui->near_rddf_point_index >= global_gui->edited_rddf_goal_size)
					global_gui->near_rddf_point_index = global_gui->edited_rddf_goal_size - 1;
				global_gui->near_rddf_point = &(global_gui->edited_rddf_goal_list[global_gui->near_rddf_point_index]);
				break;

			case GDK_p:
				global_gui->near_rddf_point_index--;
				if (global_gui->near_rddf_point_index < 0)
					global_gui->near_rddf_point_index = 0;
				global_gui->near_rddf_point = &(global_gui->edited_rddf_goal_list[global_gui->near_rddf_point_index]);

				global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_PERSON;
				break;

			case GDK_d:
				global_gui->delete_current_rddf_point();
				break;

			case GDK_f:
				global_gui->release_near_rddf_point();
				break;

			case GDK_c:
				global_gui->freeze_status = (global_gui->freeze_status)? false: true;

				global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_CAR;
				break;

			case GDK_b:
				global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_BIKE;
				break;

			case GDK_t:
				global_gui->object_type =  CARMEN_SIMULATOR_ACKERMAN_TRUCK;
				break;

			default:
				return FALSE;
		}
//
//		global_gui->near_rddf_point
	}
	else
	{
		switch (event->keyval)
		{
		case GDK_p:
			global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_PERSON;
			break;

		case GDK_b:
			global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_BIKE;
			break;

		case GDK_t:
			global_gui->object_type =  CARMEN_SIMULATOR_ACKERMAN_TRUCK;
			break;

		case GDK_c:
			global_gui->freeze_status = (global_gui->freeze_status)? false: true;
			global_gui->object_type = CARMEN_SIMULATOR_ACKERMAN_CAR;
			break;

		case GDK_j:
			global_gui->editing_final_goal = 1;
			if(global_gui->final_goal_placed_and_oriented)
			{
				final_goal_closest_index_in_nearby_lanes = global_gui->get_closest_final_goal_index_in_nearby_lanes();
			}
			break;

		case GDK_k:
			if(global_gui->editing_final_goal && global_gui->final_goal_placed_and_oriented)
			{
				int i = final_goal_closest_index_in_nearby_lanes-1;
				if(i < 0)
					i = 0;
				global_gui->final_goal.pose.x = global_gui->route_planner_route->nearby_lanes[i].x;
				global_gui->final_goal.pose.y = global_gui->route_planner_route->nearby_lanes[i].y;
				global_gui->final_goal.pose.theta = global_gui->route_planner_route->nearby_lanes[i].theta;
				global_gui->final_goal.pose.trailer_theta[0] = global_gui->route_planner_route->nearby_lanes[i].trailer_theta[0];
				global_gui->f_final_goal = fopen("final_goal_pose.txt", "w");
				if(global_gui->f_final_goal == NULL)
					printf("Could not open f_final_goal file\n");
				fprintf(global_gui->f_final_goal, "%lf, %lf, %lf, %lf\n", global_gui->final_goal.pose.x, global_gui->final_goal.pose.y, global_gui->final_goal.pose.theta, global_gui->final_goal.pose.trailer_theta[0]);
				fclose(global_gui->f_final_goal);
				final_goal_closest_index_in_nearby_lanes = i;
			}
			break;

		case GDK_l:
			if(global_gui->editing_final_goal && global_gui->final_goal_placed_and_oriented)
			{
				int i = final_goal_closest_index_in_nearby_lanes+1;
				if(i > global_gui->route_planner_route->nearby_lanes_size - 1)
					i = global_gui->route_planner_route->nearby_lanes_size - 1;
				global_gui->final_goal.pose.x = global_gui->route_planner_route->nearby_lanes[i].x;
				global_gui->final_goal.pose.y = global_gui->route_planner_route->nearby_lanes[i].y;
				global_gui->final_goal.pose.theta = global_gui->route_planner_route->nearby_lanes[i].theta;
				global_gui->final_goal.pose.trailer_theta[0] = global_gui->route_planner_route->nearby_lanes[i].trailer_theta[0];
				global_gui->f_final_goal = fopen("final_goal_pose.txt", "w");
				if(global_gui->f_final_goal == NULL)
					printf("Could not open f_final_goal file\n");
				fprintf(global_gui->f_final_goal, "%lf, %lf, %lf, %lf\n", global_gui->final_goal.pose.x, global_gui->final_goal.pose.y, global_gui->final_goal.pose.theta, global_gui->final_goal.pose.trailer_theta[0]);
				fclose(global_gui->f_final_goal);
				final_goal_closest_index_in_nearby_lanes = i;
			}
			break;

			default:
				return FALSE;
		}
	}

	return FALSE;
}

//extern "C" G_MODULE_EXPORT
int button_press_handler(GtkMapViewer		*the_map_view __attribute__ ((unused)),
		carmen_world_point_p world_point __attribute__ ((unused)),
		GdkEventButton		*event __attribute__ ((unused)))
{
	if (the_map_view->internal_map == NULL)
	{
		return TRUE;
	}

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
void draw_robot_objects(GtkMapViewer *the_map_view)
{
//	static double last_timestamp = 0.0;
//	double time = carmen_get_time();

	double pixel_size;

	if ((global_gui == NULL) || (the_map_view->internal_map == NULL))
		return;

	pixel_size = 1.0 / the_map_view->rescale_size * the_map_view->internal_map->config.resolution;

	if (global_gui->nav_panel_config->show_fused_odometry)
		global_gui->draw_fused_odometry_pose(the_map_view);

	/*
	 * Draw robot features
	 */
	global_gui->draw_parking_assistant_goal(the_map_view);

	if (global_gui->received_robot_pose())
	{
		if (global_gui->nav_panel_config->show_particles)
			global_gui->draw_particles(the_map_view, pixel_size);

		if (global_gui->nav_panel_config->show_gaussians)
			global_gui->draw_gaussians(the_map_view);

		if (global_gui->nav_panel_config->show_lasers)
			global_gui->draw_lasers(the_map_view, pixel_size);

		global_gui->draw_robot(the_map_view);
	}

	global_gui->draw_plan_tree(the_map_view, pixel_size);

	//do some animation when the user is placing something (like robot or goal)
	global_gui->draw_placing_animation(the_map_view);

	if (global_gui->nav_panel_config->show_path_plans)
	{
		if (global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses != 0)
		{
			global_gui->frenet_path_planer_number_of_poses = global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses;
			global_gui->frenet_path_planer_path = (carmen_world_robot_and_trailer_pose_t *) malloc(sizeof(carmen_world_robot_and_trailer_pose_t) * global_gui->frenet_path_planer_number_of_poses);
			int number_of_paths = global_gui->frenet_path_planer_set_of_paths_msg.set_of_paths_size / global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses;

			for (int j = 0; j < number_of_paths; j++)
			{
				for (int i = 0; i < global_gui->frenet_path_planer_number_of_poses; i++)
				{
					global_gui->frenet_path_planer_path[i].pose.x	  = global_gui->frenet_path_planer_set_of_paths_msg.set_of_paths[j * global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses + i].x;
					global_gui->frenet_path_planer_path[i].pose.y	  = global_gui->frenet_path_planer_set_of_paths_msg.set_of_paths[j * global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses + i].y;
					global_gui->frenet_path_planer_path[i].pose.theta = global_gui->frenet_path_planer_set_of_paths_msg.set_of_paths[j * global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses + i].theta;
					global_gui->frenet_path_planer_path[i].pose.trailer_theta[0]  = global_gui->frenet_path_planer_set_of_paths_msg.set_of_paths[j * global_gui->frenet_path_planer_set_of_paths_msg.number_of_poses + i].trailer_theta[0];
					global_gui->frenet_path_planer_path[i].map 	      = global_gui->controls_.map_view->internal_map;
				}

				if (j != global_gui->frenet_path_planer_set_of_paths_msg.selected_path)
					global_gui->draw_path(global_gui->frenet_path_planer_path, global_gui->frenet_path_planer_number_of_poses, carmen_light_green, carmen_light_green, the_map_view);
				else
					global_gui->draw_path(global_gui->frenet_path_planer_path, global_gui->frenet_path_planer_number_of_poses, carmen_light_blue, carmen_light_blue, the_map_view);
			}
			free(global_gui->frenet_path_planer_path);
		}
	}

	if (global_gui->nav_panel_config->show_nearby_lanes)
	{
		if (global_gui->route_planner_route != NULL)
		{
			if (global_gui->route_planner_route->number_of_nearby_lanes != 0)
			{
				for (int j = 0; j < global_gui->route_planner_route->number_of_nearby_lanes; j++)
				{
					int lane_size = global_gui->route_planner_route->nearby_lanes_sizes[j];
					global_gui->route_planer_lane = (carmen_world_robot_and_trailer_pose_t *) malloc(lane_size * sizeof(carmen_world_robot_and_trailer_pose_t));
					int lane_start = global_gui->route_planner_route->nearby_lanes_indexes[j];
					for (int i = 0; i < lane_size; i++)
					{
						global_gui->route_planer_lane[i].pose.x	  	= global_gui->route_planner_route->nearby_lanes[lane_start + i].x;
						global_gui->route_planer_lane[i].pose.y	  	= global_gui->route_planner_route->nearby_lanes[lane_start + i].y;
						global_gui->route_planer_lane[i].pose.theta = global_gui->route_planner_route->nearby_lanes[lane_start + i].theta;
						global_gui->route_planer_lane[i].pose.trailer_theta[0]  = global_gui->route_planner_route->nearby_lanes[lane_start + i].trailer_theta[0];
						global_gui->route_planer_lane[i].map 	   	= global_gui->controls_.map_view->internal_map;
					}

					global_gui->draw_path(global_gui->route_planer_lane, lane_size, carmen_orange, carmen_orange, the_map_view);
					free(global_gui->route_planer_lane);
				}

				if (global_gui->route_planner_route->number_of_poses > 0)
				{
					int lane_size = global_gui->route_planner_route->number_of_poses;
					global_gui->route_planer_lane = (carmen_world_robot_and_trailer_pose_t *) malloc(lane_size * sizeof(carmen_world_robot_and_trailer_pose_t));
					for (int i = 0; i < lane_size; i++)
					{
						global_gui->route_planer_lane[i].pose.x	  	= global_gui->route_planner_route->poses[i].x;
						global_gui->route_planer_lane[i].pose.y	  	= global_gui->route_planner_route->poses[i].y;
						global_gui->route_planer_lane[i].pose.theta = global_gui->route_planner_route->poses[i].theta;
						global_gui->route_planer_lane[i].pose.trailer_theta[0]  = global_gui->route_planner_route->poses[i].trailer_theta[0];
						global_gui->route_planer_lane[i].map 	   	= global_gui->controls_.map_view->internal_map;
					}

					global_gui->draw_path(global_gui->route_planer_lane, lane_size, carmen_light_blue, carmen_light_blue, the_map_view);
					free(global_gui->route_planer_lane);
				}

				if (global_gui->route_planner_route->number_of_poses_back > 0)
				{
					int lane_size = global_gui->route_planner_route->number_of_poses_back;
					global_gui->route_planer_lane = (carmen_world_robot_and_trailer_pose_t *) malloc(lane_size * sizeof(carmen_world_robot_and_trailer_pose_t));
					for (int i = 0; i < lane_size; i++)
					{
						global_gui->route_planer_lane[i].pose.x	  	= global_gui->route_planner_route->poses_back[i].x;
						global_gui->route_planer_lane[i].pose.y	  	= global_gui->route_planner_route->poses_back[i].y;
						global_gui->route_planer_lane[i].pose.theta = global_gui->route_planner_route->poses_back[i].theta;
						global_gui->route_planer_lane[i].pose.trailer_theta[0]  = global_gui->route_planner_route->poses_back[i].trailer_theta[0];
						global_gui->route_planer_lane[i].map 	   	= global_gui->controls_.map_view->internal_map;
					}

					global_gui->draw_path(global_gui->route_planer_lane, lane_size, carmen_light_blue, carmen_light_blue, the_map_view);
					free(global_gui->route_planer_lane);
				}
			}
		}
	}

	if (global_gui->nav_panel_config->show_nearby_lanes_width)
	{
		if (global_gui->route_planner_route != NULL)
		{
			if (global_gui->route_planner_route->number_of_nearby_lanes != 0)
			{
				carmen_world_point_t lane_line_start, lane_line_end;
				lane_line_start.map = global_gui->controls_.map_view->internal_map;
				lane_line_end.map = global_gui->controls_.map_view->internal_map;

				for (int j = 0; j < global_gui->route_planner_route->number_of_nearby_lanes; j++)
				{
					int lane_size = global_gui->route_planner_route->nearby_lanes_sizes[j];
					int lane_start = global_gui->route_planner_route->nearby_lanes_indexes[j];

					for (int i = 0; i < lane_size; i++)
					{
						int traffic_restrictions = global_gui->route_planner_route->traffic_restrictions[lane_start + i];
						double lane_left_width = ROUTE_PLANNER_GET_LANE_LEFT_WIDTH(traffic_restrictions);
						double lane_right_width = ROUTE_PLANNER_GET_LANE_RIGHT_WIDTH(traffic_restrictions);

						carmen_robot_and_trailers_traj_point_t lane_point = global_gui->route_planner_route->nearby_lanes[lane_start + i];
						lane_line_start.pose.x = lane_point.x + (lane_left_width * cos(lane_point.theta + M_PI / 2.0));
						lane_line_start.pose.y = lane_point.y + (lane_left_width * sin(lane_point.theta + M_PI / 2.0));
						lane_line_end.pose.x = lane_point.x;
						lane_line_end.pose.y = lane_point.y;
						carmen_map_graphics_draw_line(the_map_view, &carmen_green, &lane_line_start, &lane_line_end);

						lane_line_start.pose.x = lane_point.x;
						lane_line_start.pose.y = lane_point.y;
						lane_line_end.pose.x = lane_point.x - lane_right_width * cos(lane_point.theta + M_PI / 2.0);
						lane_line_end.pose.y = lane_point.y - lane_right_width * sin(lane_point.theta + M_PI / 2.0);
						carmen_map_graphics_draw_line(the_map_view, &carmen_red, &lane_line_start, &lane_line_end);
					}
				}
			}
		}
	}

	carmen_world_point_t temp_goal;
	temp_goal.pose.x = global_gui->goal.x;
	temp_goal.pose.y = global_gui->goal.y;
	temp_goal.pose.theta = global_gui->goal.theta;
	temp_goal.map = the_map_view->internal_map;
	global_gui->draw_goal_list(the_map_view, temp_goal);

	if (global_gui->nav_panel_config->show_command_plan)
	{
		global_gui->obstacle_avoider_path_size = global_gui->obstacle_avoider_msg.path_length;
		global_gui->obstacle_avoider_path = (carmen_world_robot_and_trailer_pose_t *) malloc(sizeof(carmen_world_robot_and_trailer_pose_t) * global_gui->obstacle_avoider_path_size);

		for (int i = 0; i < global_gui->obstacle_avoider_path_size; i++)
		{
			global_gui->obstacle_avoider_path[i].pose.x	   = global_gui->obstacle_avoider_msg.path[i].x;
			global_gui->obstacle_avoider_path[i].pose.y	   = global_gui->obstacle_avoider_msg.path[i].y;
			global_gui->obstacle_avoider_path[i].pose.theta = global_gui->obstacle_avoider_msg.path[i].theta;
			global_gui->obstacle_avoider_path[i].pose.trailer_theta[0] = global_gui->obstacle_avoider_msg.path[i].trailer_theta[0];
			global_gui->obstacle_avoider_path[i].map = global_gui->controls_.map_view->internal_map;
		}

		global_gui->draw_path(global_gui->obstacle_avoider_path, global_gui->obstacle_avoider_path_size, carmen_red, carmen_red, the_map_view);
		free(global_gui->obstacle_avoider_path);
	}

	if (global_gui->nav_panel_config->show_oa_motion_plan)
	{
		global_gui->oa_motion_plan_size = global_gui->oa_motion_plan_msg.path_length;
		global_gui->oa_motion_plan = (carmen_world_robot_and_trailer_pose_t *) malloc(sizeof(carmen_world_robot_and_trailer_pose_t) * global_gui->oa_motion_plan_size);

		for (int i = 0; i < global_gui->oa_motion_plan_size; i++)
		{
			global_gui->oa_motion_plan[i].pose.x	   = global_gui->oa_motion_plan_msg.path[i].x;
			global_gui->oa_motion_plan[i].pose.y	   = global_gui->oa_motion_plan_msg.path[i].y;
			global_gui->oa_motion_plan[i].pose.theta  = global_gui->oa_motion_plan_msg.path[i].theta;
			global_gui->oa_motion_plan[i].pose.trailer_theta[0]  = global_gui->oa_motion_plan_msg.path[i].trailer_theta[0];
			global_gui->oa_motion_plan[i].map 		   = global_gui->controls_.map_view->internal_map;
		}

		global_gui->draw_path(global_gui->oa_motion_plan, global_gui->oa_motion_plan_size, carmen_orange, carmen_orange, the_map_view);
		free(global_gui->oa_motion_plan);
	}

	if (global_gui->nav_panel_config->show_mpp_motion_plan)
	{
//		global_gui->mpp_motion_plan_size = global_gui->mpp_motion_plan_msg.plan_length;
		global_gui->mpp_motion_plan_size = global_gui->mpp_motion_plan_msg_rrt.size;
		global_gui->mpp_motion_plan = (carmen_world_robot_and_trailer_pose_t *) malloc(sizeof(carmen_world_robot_and_trailer_pose_t) * global_gui->mpp_motion_plan_size);

		for (int i = 0; i < global_gui->mpp_motion_plan_size; i++)
		{
//			global_gui->mpp_motion_plan[i].pose.x	   = global_gui->mpp_motion_plan_msg.plan[i].x;
//			global_gui->mpp_motion_plan[i].pose.y	   = global_gui->mpp_motion_plan_msg.plan[i].y;
//			global_gui->mpp_motion_plan[i].pose.theta  = global_gui->mpp_motion_plan_msg.plan[i].theta;
//			global_gui->mpp_motion_plan[i].map 		   = global_gui->controls_.map_view->internal_map;
			global_gui->mpp_motion_plan[i].pose.x	   = global_gui->mpp_motion_plan_msg_rrt.path[i].p1.x;
			global_gui->mpp_motion_plan[i].pose.y	   = global_gui->mpp_motion_plan_msg_rrt.path[i].p1.y;
			global_gui->mpp_motion_plan[i].pose.theta  = global_gui->mpp_motion_plan_msg_rrt.path[i].p1.theta;
			global_gui->mpp_motion_plan[i].pose.trailer_theta[0]  = global_gui->mpp_motion_plan_msg_rrt.path[i].p1.trailer_theta[0];
			global_gui->mpp_motion_plan[i].map 		   = global_gui->controls_.map_view->internal_map;
		}

		global_gui->draw_path(global_gui->mpp_motion_plan, global_gui->mpp_motion_plan_size, carmen_green, carmen_green, the_map_view);
		free(global_gui->mpp_motion_plan);
	}

	if (global_gui->nav_panel_config->show_offroad_plan)
	{
		if (global_gui->route_planner_route != NULL && global_gui->offroad_planner_plan != NULL)
		{
			if (global_gui->route_planner_route->route_planner_state == EXECUTING_OFFROAD_PLAN && global_gui->offroad_planner_plan->number_of_poses != 0)
			{
				carmen_world_robot_and_trailer_pose_t *offroad_planner_path = (carmen_world_robot_and_trailer_pose_t *) malloc(sizeof(carmen_world_robot_and_trailer_pose_t) * global_gui->offroad_planner_plan->number_of_poses);

				for (int i = 0; i < global_gui->offroad_planner_plan->number_of_poses; i++)
				{
					offroad_planner_path[i].pose.x	   = global_gui->offroad_planner_plan->poses[i].x;
					offroad_planner_path[i].pose.y	   = global_gui->offroad_planner_plan->poses[i].y;
					offroad_planner_path[i].pose.theta = global_gui->offroad_planner_plan->poses[i].theta;
					offroad_planner_path[i].pose.trailer_theta[0]  = global_gui->offroad_planner_plan->poses[i].trailer_theta[0];
					offroad_planner_path[i].map		   = global_gui->controls_.map_view->internal_map;
				}

				global_gui->draw_path(offroad_planner_path, global_gui->offroad_planner_plan->number_of_poses, carmen_grey, carmen_grey, the_map_view);
				free(offroad_planner_path);
			}
		}
	}

	if (global_gui->nav_panel_config->draw_path)
	{
		for (int i = 0; i < global_gui->num_candidate_path; i++)
			global_gui->draw_path(global_gui->canditade_path[i], global_gui->candidate_path_size[i], global_gui->canditade_path_color[i],
					global_gui->path_colour, the_map_view);
	}

	if (global_gui->nav_panel_config->draw_path && global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->draw_path(global_gui->path, global_gui->num_path_points, global_gui->path_colour, carmen_black, the_map_view);

	if (global_gui->nav_panel_config->show_dynamic_objects)
		global_gui->draw_moving_objects(the_map_view);

	if (global_gui->nav_panel_config->show_dynamic_points)
		global_gui->draw_moving_points(the_map_view, pixel_size);

	if (global_gui->nav_panel_config->show_annotations)
		global_gui->draw_annotations(the_map_view, pixel_size);

	if (global_gui->nav_panel_config->show_lane_markings)
		global_gui->draw_lane_lines(the_map_view, pixel_size);

	global_gui->draw_path_vector(the_map_view);

	global_gui->draw_gl_components_car_panel();

	if (global_gui->nav_panel_config->show_true_pos)
		global_gui->draw_simulated_robot(the_map_view);

	if (global_gui->nav_panel_config->show_simulator_objects)
		global_gui->draw_simulated_objects(the_map_view);

//	printf("time - last_timestamp = %lf,  dt %lf\n", time - last_timestamp, time - carmen_get_time());
//	last_timestamp = carmen_get_time();
}
}
