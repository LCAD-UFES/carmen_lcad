#ifndef DRAW_CALLBACKS_H
#define DRAW_CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <gdk/gdkkeysyms.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>



#include <gtk_gui.h>

#include <carmen/global_graphics.h>
#include <carmen/map_graphics.h>

#include "gtk_gui.h"

namespace View
{

class GtkGui;

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void *data);

extern "C" G_MODULE_EXPORT
void on_drawingArea_realize (GtkWidget *widget, GtkGui *gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose  *event __attribute__((unused)), GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_drawingAreaCarPanel_realize (GtkWidget *widget, GtkGui *gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawingAreaCarPanel_expose_event (GtkWidget *widget, GdkEventExpose *event __attribute__((unused)), GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_mainWindow_realize (GtkWidget *widget, GtkGui *gui);

/* Menu Handlers */

extern "C" G_MODULE_EXPORT
void on_activeMenuQuit(GtkWidget *widget, GdkEvent* event, GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_menuMaps_Map_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_MapLevel1_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_Utility_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_Costs_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_Likelihood_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_GlobalLikelihood_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_Lane_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_CompleteMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuMaps_RoadMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_None_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Map_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_MapLevel1_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_OfflineMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Utility_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Costs_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Likelihood_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_GlobalLikelihood_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_Lane_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_RemissionMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_MovingObjects_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSuperimposedMaps_RoadMap_toggled (GtkCheckMenuItem* togglebutton, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonZoomIn_clicked(GtkWidget *widget __attribute__((unused)), GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonZoomOut_clicked(GtkWidget *widget __attribute__((unused)), GtkGui* gui);

extern "C" G_MODULE_EXPORT
gint motion_handler(GtkMapViewer *the_map_view, carmen_world_point_t *world_point, GdkEventMotion *event);

extern "C" G_MODULE_EXPORT
int button_release_handler(GtkMapViewer		   *the_map_view,
		carmen_world_point_t *world_point,
		GdkEventButton	   *event  );

extern "C" G_MODULE_EXPORT
int keyboard_press_handler(GtkMapViewer *the_map_view,
		GdkEventKey	   *event);

extern "C" G_MODULE_EXPORT
int button_press_handler(GtkMapViewer		*the_map_view  ,
		carmen_world_point_p world_point  ,
		GdkEventButton		*event  );

extern "C" G_MODULE_EXPORT
void draw_robot_objects(GtkMapViewer *the_map_view);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_TrackRobot_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawPath_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawWaipoints_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_DrawRobotWaipoints_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLateralOffset_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowParticles_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowFusedOdometry_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowGaussians_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLaserData_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowOAMotionPlan_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowMPPMotionPlan_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowCommandPlan_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuGoals_EditRddfGoals_toggled (GtkCheckMenuItem* togglebutton __attribute__ ((unused)),
		GtkGui* gui __attribute__ ((unused)));

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowDynamicObjects_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowDynamicPoints_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowAnnotations_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowLaneMarkings_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuDisplay_ShowCollisionRange_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSimulatorShowTruePosition_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSimulator_ShowObjects_toggled (GtkCheckMenuItem* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuSimulator_ClearObjects_activate(GtkWidget *widget,
					   GdkEvent *event,
					   GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_menuSimulator_AddPerson_activate(GtkWidget *widget,
					   GdkEvent *event,
					   GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_menuStartLocation_GlobalLocalization_activate(GtkWidget *widget,
					   GdkEvent *event,
					   GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_menuHelp_About_activate(GtkWidget *widget,
					   GdkEvent *event,
					   GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_comboGoalSource_changed(GtkWidget *widget, GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_comboState_changed(GtkWidget *widget, GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_comboFollowLane_changed(GtkWidget *widget, GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_comboParking_changed(GtkWidget *widget, GtkGui* data);

extern "C" G_MODULE_EXPORT
void on_buttonPlaceRobot_clicked(GtkWidget *widget , GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonPlaceGoal_clicked(GtkWidget *widget , GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonRemoveGoal_clicked(GtkWidget *widget , GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonClearGoals_clicked(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonGo_clicked(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonGo_entered(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonRecord_clicked(GtkWidget *widget, GtkGui* gui);

//extern "C" G_MODULE_EXPORT
//void on_buttonRecord_entered(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonPlaceFinalGoal_clicked(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_buttonPlaceSimulator_clicked(GtkWidget *widget, GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_menuCarPanel_fused_odometry_message(GtkRadioMenuItem *togglebutton, GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_menuCarPanel_robot_ackerman_message(GtkRadioMenuItem *togglebutton, GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_menuCarPanel_base_ackerman_motion_message(GtkRadioMenuItem *togglebutton, GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_menuCarPanel_base_ackerman_odometry_message(GtkRadioMenuItem *togglebutton, GtkGui *gui);

extern "C" G_MODULE_EXPORT
void on_menuCarPanel_localize_ackerman_globalpos_message(GtkRadioMenuItem *togglebutton, GtkGui *gui);

//extern "C" G_MODULE_EXPORT
//void on_drawArea_realize (GtkWidget * widget,
//					 	            	GtkGui 	  * gui);
//
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_configure_event(GtkWidget         * widget,
//                                 	   GdkEventConfigure * event,
//                                 	   GtkGui 			     * gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_expose_event(GtkWidget      * widget,
//                             	    GdkEventExpose * event,
//                                  GtkGui 			   * gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_button_press_event(GtkWidget      * widget,
//                                        GdkEventButton * event,
//                                        GtkGui 			   * gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_button_release_event(GtkWidget      * widget,
//                                          GdkEventButton * event,
//                                          GtkGui 	       * gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
//                                         GdkEventMotion * event,
//                                         GtkGui 		    * gui);
//
//
//extern "C" G_MODULE_EXPORT
//gboolean on_drawArea_key_press_event(GtkWidget   * widget,
//                                     GdkEventKey * event,
//                                     GtkGui 	   * gui);
//
//extern "C" G_MODULE_EXPORT
//void on_winMain_destroy (GtkObject *object, GtkGui  * gui);
//
//extern "C" G_MODULE_EXPORT
//void on_ntbkViewOptions_switch_page (GtkNotebook    * notebook,
//                                     GtkNotebookPage* page,
//                                     guint            page_num,
//                                     GtkGui         * gui);
//
//extern "C" G_MODULE_EXPORT
//void on_winMain_realize(GtkWidget * widget,
//                        GtkGui    * gui);
//
//extern "C" G_MODULE_EXPORT
//void on_rdbFollowCamera_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_rdbStaticCamera_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_rdbDriverCamera_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawRobot_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawDistanceInformation_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawSaliencies_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawCorrectedPath_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawVisualOdometryPath_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_btnDrawParticles_toggled (GtkToggleButton* togglebutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
//                                       GdkEventFocus *event,
//                                       GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
//                                       GdkEventFocus *event,
//                                       GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
//                                       GdkEventFocus *event,
//                                       GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
//                                        GdkEventFocus* event,
//                                        GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
//                                        GdkEventFocus* event,
//                                        GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
//                                        GdkEventFocus* event,
//                                        GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry,
//                                          GdkEventFocus *event,
//                                          gpointer  data);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry,
//                                          GdkEventFocus *event,
//                                          gpointer  data);
//
//extern "C" G_MODULE_EXPORT
//gboolean on_txtVisSize_focus_out_event(GtkEntry    *entry,
//                                       GdkEventFocus *event,
//                                       GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_txtZPlane_value_changed (GtkSpinButton* spinbutton,
//		gpointer      data);
//
//extern "C" G_MODULE_EXPORT
//void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton,
//		gpointer       data);
//
//extern "C" G_MODULE_EXPORT
//void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton,
//		gpointer       data);
//
//extern "C" G_MODULE_EXPORT
//void on_txtObstacleCutoffAbove_value_changed (GtkSpinButton* spinbutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
//		GtkGui* gui);
//
//
//extern "C" G_MODULE_EXPORT
//void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
//		GtkGui* gui);
//
//extern "C" G_MODULE_EXPORT
//void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
//		gpointer       data);
//
//void updateUIFromCamera(GtkGui* gui);

}

#endif

