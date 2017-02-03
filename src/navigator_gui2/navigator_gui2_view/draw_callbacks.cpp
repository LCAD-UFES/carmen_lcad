#include "draw_callbacks.h"


namespace View
{

GtkGui *global_gui = NULL;
int superimposed_is_set = 0;

//extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void *data)
{
	GtkGui *gui = static_cast<GtkGui*>(data);

	global_gui = gui;

	if (!GTK_IS_WIDGET(global_gui->controls_.drawArea))
		return TRUE;

	gtk_widget_draw(global_gui->controls_.drawArea, NULL);
	gtk_widget_draw(global_gui->controls_.drawAreaCarPanel, NULL);
	return TRUE;
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

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

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
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_NAVIGATOR_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_OFFLINE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Utility_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_NAVIGATOR_UTILITY_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Costs_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_COST_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Likelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LOCALIZE_LMAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_GlobalLikelihood_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LOCALIZE_GMAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_Lane_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_LANE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_CompleteMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_COMPLETE_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_REMISSION_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_REMISSION_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 0;
		navigator_get_map(CARMEN_MOVING_OBJECTS_MAP_v, superimposed_is_set);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_None_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if(gtk_check_menu_item_get_active(togglebutton))
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
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_NAVIGATOR_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_OFFLINE_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Utility_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if(gtk_check_menu_item_get_active(togglebutton))
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
	if(gtk_check_menu_item_get_active(togglebutton))
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
	if(gtk_check_menu_item_get_active(togglebutton))
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
	if(gtk_check_menu_item_get_active(togglebutton))
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
	if(gtk_check_menu_item_get_active(togglebutton))
	{
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_LANE_MAP_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui)
{
	if(gtk_check_menu_item_get_active(togglebutton))
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
	{
		carmen_map_graphics_adjust_scrollbars(global_gui->controls_.map_view, &global_gui->robot);
	}
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

	if ((global_gui->nav_panel_config->show_particles == 1) &&
			!global_gui->nav_panel_config->show_gaussians)
	{
		carmen_localize_ackerman_subscribe_particle_message(&global_gui->particle_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}
	else if (!global_gui->nav_panel_config->show_particles &&
			!global_gui->nav_panel_config->show_gaussians)
	{
		carmen_localize_ackerman_subscribe_particle_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
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
	{
		carmen_localize_ackerman_subscribe_sensor_message(&global_gui->sensor_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowMotionPath_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_motion_path = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_motion_path)
	{
		carmen_obstacle_avoider_subscribe_motion_planner_path_message(&global_gui->motion_path_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		carmen_obstacle_avoider_subscribe_motion_planner_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
	}
}

//extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowCommandPath_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->show_command_path = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->show_command_path)
	{
		carmen_obstacle_avoider_subscribe_path_message(&global_gui->obstacle_avoider_msg, NULL, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		carmen_obstacle_avoider_subscribe_path_message(NULL, NULL, CARMEN_UNSUBSCRIBE);
	}

}

//extern "C" G_MODULE_EXPORT
void on_menuGoals_EditRddfGoals_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)))
{
	global_gui->nav_panel_config->edit_rddf_goals = gtk_check_menu_item_get_active(togglebutton);

	if (global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->load_rddf_from_file((char*) "rddf_edited.kml");

	if (!global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->save_rddf_to_file((char*) "rddf_edited.kml");
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

//extern "C" G_MODULE_EXPORT
void on_comboGoalSource_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if(global_gui)
		carmen_behavior_selector_set_goal_source((carmen_behavior_selector_goal_source_t)global_gui->get_goal_source_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboGoalSource)));
}

//extern "C" G_MODULE_EXPORT
void on_comboState_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if(global_gui)
		carmen_behavior_selector_set_state((carmen_behavior_selector_state_t)global_gui->get_state_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboState)));
}

//extern "C" G_MODULE_EXPORT
void on_comboFollowLane_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if(global_gui)
		navigator_set_algorithm((carmen_behavior_selector_algorithm_t)global_gui->get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboFollowLane)), BEHAVIOR_SELECTOR_FOLLOWING_LANE);
}

//extern "C" G_MODULE_EXPORT
void on_comboParking_changed(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if(global_gui)
		navigator_set_algorithm((carmen_behavior_selector_algorithm_t)global_gui->get_algorithm_code(gtk_combo_box_get_active_text((GtkComboBox*)global_gui->controls_.comboParking)), BEHAVIOR_SELECTOR_PARKING);
}

//extern "C" G_MODULE_EXPORT
void on_menuCarPanel_fused_odometry_message(GtkRadioMenuItem *togglebutton, GtkGui *gui)
{
	global_gui->car_panel_gl->set_type_message(0);

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
void on_buttonPlaceGoal_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	global_gui->change_cursor(&carmen_yellow, &carmen_black);
	global_gui->placement_status = PLACING_GOAL;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonRemoveGoal_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (!global_gui->behavior_selector_active || global_gui->goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
	{
		global_gui->execute_decrement_point();

		//change_cursor(&carmen_yellow, &carmen_black);
	}
	else
	{
		carmen_behavior_selector_remove_goal();
	}

	global_gui->placement_status = PLACING_GOAL;

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonClearGoals_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	if (!global_gui->behavior_selector_active || global_gui->goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		{
			if (global_gui->queuePoints != NULL)
			{
				pointers *item = global_gui->queuePoints->begin;
				pointers *itemAux;

				while (item != NULL)
				{
					itemAux = item;
					item	= item->next;
					free(itemAux);
				}

				global_gui->queuePoints->begin = NULL;
				global_gui->queuePoints->curr  = NULL;
				global_gui->queuePoints->end   = NULL;

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

	gtk_toggle_button_set_active((GtkToggleButton *) widget, false);
}

//extern "C" G_MODULE_EXPORT
void on_buttonGo_clicked(GtkWidget *widget __attribute__((unused)),
					   GtkGui* gui)
{
	GtkWidget *label;

//	TODO: o que eh isso de ignore click?
//	if (!global_gui->ignore_click)
//	{
		if (GTK_TOGGLE_BUTTON(global_gui->controls_.buttonGo)->active)
		{

			label = GTK_BIN(global_gui->controls_.buttonGo)->child;
			gtk_label_set_text(GTK_LABEL(label), "Stop");
			navigator_start_moving();
		}
		else
		{
			label = GTK_BIN(global_gui->controls_.buttonGo)->child;
			gtk_label_set_text(GTK_LABEL(label), "Go");
			navigator_stop_moving();
		}
//	}
//	else
//	{
//		global_gui->ignore_click = 0;
//	}

	if(global_gui->global_view)
	{
		global_gui->global_view = 0;
	}
}

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

	sprintf(buffer, "Grid Cell: %d, %d  (%.1f m, %.1f m)", point.x, point.y,
			world_point->pose.x, world_point->pose.y);
	gtk_label_set_text(GTK_LABEL(global_gui->controls_.labelGridCell), buffer);

	if (the_map != NULL)
	{
		sprintf(buffer, "Value: %.2f", the_map->map[point.x][point.y]);
		gtk_label_set_text(GTK_LABEL(global_gui->controls_.labelValue), buffer);
	}

	if ((global_gui->placement_status == ORIENTING_ROBOT) ||
			(global_gui->placement_status == ORIENTING_GOAL) ||
			(global_gui->placement_status == ORIENTING_SIMULATOR) ||
			(global_gui->placement_status == ORIENTING_FINAL_GOAL) ||
			(global_gui->placement_status == ORIENTING_PERSON))
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

	gdk_window_set_cursor(global_gui->controls_.map_view->image_widget->window, gdk_cursor_new(GDK_BASED_ARROW_DOWN));
	global_gui->placement_status = SELECTING_FINAL_REGION;

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
		superimposed_is_set = 1;
		navigator_get_map(CARMEN_NONE_v, superimposed_is_set);
		carmen_map_graphics_redraw_superimposed(global_gui->controls_.map_view);

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
	if(rtr)
		return TRUE;

	rtr = global_gui->placing_goal_action(the_map_view, world_point, event);
	if(rtr)
		return TRUE;

	rtr = global_gui->placing_person_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->placing_simulator_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->orienting_robot_action(the_map_view, world_point, event);
	if(rtr)
		return TRUE;

	rtr = global_gui->orienting_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->orienting_person_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->orienting_simulator_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->selecting_final_region_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->placing_final_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->orienting_final_goal_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->selecting_near_waypoint_action(the_map_view, world_point);
	if(rtr)
		return TRUE;

	rtr = global_gui->select_near_rddf_point(the_map_view, world_point);
	if(rtr)
		return TRUE;

	return TRUE;
}

//extern "C" G_MODULE_EXPORT
int keyboard_press_handler(GtkMapViewer *the_map_view,
		GdkEventKey	   *event)
{
	double resolution = the_map_view->internal_map->config.resolution;

	if (global_gui->placement_status == EDITING_NEAR_RDDF && global_gui->near_rddf_point != NULL)
	{
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
				global_gui->near_rddf_point->pose.theta += carmen_degrees_to_radians(1.0);
				global_gui->near_rddf_point->pose.theta = carmen_normalize_theta(global_gui->near_rddf_point->pose.theta);
				break;

			case GDK_s:
				global_gui->near_rddf_point->pose.theta -= carmen_degrees_to_radians(1.0);
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
				break;

			case GDK_d:
				global_gui->delete_current_rddf_point();
				break;

			case GDK_f:
				global_gui->release_near_rddf_point();
				break;

			default:
				return FALSE;
		}
//
//		global_gui->near_rddf_point
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
	//  int colour;
	double pixel_size;

	if ((global_gui == NULL) || (the_map_view->internal_map == NULL))
		return;

	pixel_size = 1 / the_map_view->rescale_size *
			the_map_view->internal_map->config.resolution;

	if (global_gui->nav_panel_config->show_fused_odometry)
		global_gui->draw_fused_odometry_pose(the_map_view);

	/*
	 * Draw robot features
	 */
	global_gui->draw_parking_assistant_goal(the_map_view);

	if (global_gui->received_robot_pose())
	{
		if(global_gui->nav_panel_config->show_particles)
			global_gui->draw_particles(the_map_view, pixel_size);

		if(global_gui->nav_panel_config->show_gaussians)
			global_gui->draw_gaussians(the_map_view);

		if(global_gui->nav_panel_config->show_lasers)
			global_gui->draw_lasers(the_map_view, pixel_size);

		global_gui->draw_robot(the_map_view);
	}

	global_gui->draw_plan_tree(the_map_view, pixel_size);

	global_gui->draw_goal_list(the_map_view, global_gui->goal);

	if(global_gui->nav_panel_config->show_true_pos)
		global_gui->draw_simulated_robot(the_map_view);

	if(global_gui->nav_panel_config->show_simulator_objects)
		global_gui->draw_simulated_objects(the_map_view);

	//do some animation when the user is placing something (like robot or goal)
	global_gui->draw_placing_animation(the_map_view);

	if (global_gui->nav_panel_config->show_motion_path)
	{
		if ((global_gui->motion_path != NULL) || (global_gui->motion_path_size < global_gui->motion_path_msg.path_length))
		{
			free(global_gui->motion_path);
			global_gui->motion_path = NULL;
		}

		global_gui->motion_path_size = global_gui->motion_path_msg.path_length;

		if (global_gui->motion_path == NULL)
			global_gui->motion_path = (carmen_world_point_t*) malloc(sizeof(carmen_world_point_t) * global_gui->motion_path_size);

		int i;
		for (i = 0; i < global_gui->motion_path_msg.path_length; i++)
		{
			global_gui->motion_path[i].pose.x	   = global_gui->motion_path_msg.path[i].x;
			global_gui->motion_path[i].pose.y	   = global_gui->motion_path_msg.path[i].y;
			global_gui->motion_path[i].pose.theta  = global_gui->motion_path_msg.path[i].theta;
			global_gui->motion_path[i].map 		   = global_gui->controls_.map_view->internal_map;
		}

		global_gui->draw_path(global_gui->motion_path, global_gui->motion_path_size, carmen_green, carmen_green, the_map_view);
	}

	if (global_gui->nav_panel_config->show_command_path)
	{
		if ((global_gui->obstacle_avoider_path != NULL) || (global_gui->obstacle_avoider_path_size < global_gui->obstacle_avoider_msg.path_length))
		{
			free(global_gui->obstacle_avoider_path);
			global_gui->obstacle_avoider_path = NULL;
		}

		global_gui->obstacle_avoider_path_size = global_gui->obstacle_avoider_msg.path_length;

		if (global_gui->obstacle_avoider_path == NULL)
			global_gui->obstacle_avoider_path = (carmen_world_point_t*) malloc(sizeof(carmen_world_point_t) * global_gui->obstacle_avoider_path_size);

		int i;

		for (i = 0; i < global_gui->obstacle_avoider_msg.path_length; i++)
		{
			global_gui->obstacle_avoider_path[i].pose.x	   = global_gui->obstacle_avoider_msg.path[i].x;
			global_gui->obstacle_avoider_path[i].pose.y	   = global_gui->obstacle_avoider_msg.path[i].y;
			global_gui->obstacle_avoider_path[i].pose.theta = global_gui->obstacle_avoider_msg.path[i].theta;
			global_gui->obstacle_avoider_path[i].map = global_gui->controls_.map_view->internal_map;
		}

		global_gui->draw_path(global_gui->obstacle_avoider_path, global_gui->obstacle_avoider_path_size, carmen_red, carmen_red, the_map_view);
	}

	if (global_gui->nav_panel_config->draw_path)
	{
		for (int i = 0; i < global_gui->num_candidate_path; i++)
			global_gui->draw_path(global_gui->canditade_path[i], global_gui->candidate_path_size[i], global_gui->path_colour,
					global_gui->path_colour, the_map_view);
	}

	if (global_gui->nav_panel_config->draw_path && global_gui->nav_panel_config->edit_rddf_goals)
		global_gui->draw_path(global_gui->path, global_gui->num_path_points, global_gui->path_colour, carmen_black, the_map_view);

	if (global_gui->nav_panel_config->show_dynamic_objects)
	{
		global_gui->draw_moving_objects(the_map_view);
	}

	if (global_gui->nav_panel_config->show_dynamic_points)
	{
		global_gui->draw_moving_points(the_map_view);
	}

	global_gui->draw_path_vector(the_map_view);
}
}
