#ifndef CARMEN_LASLAM_DRAW_CALLBACKS_H
#define CARMEN_LASLAM_DRAW_CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <gdk/gdkkeysyms.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <gtk_gui.h>

namespace View
{

class GtkGui;

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void * data);


extern "C" G_MODULE_EXPORT
void on_drawArea_realize (GtkWidget * widget,
					 	            	GtkGui 	  * gui);


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_configure_event(GtkWidget         * widget,
                                 	   GdkEventConfigure * event,
                                 	   GtkGui 			     * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_expose_event(GtkWidget      * widget,
                             	    GdkEventExpose * event,
                                  GtkGui 			   * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_press_event(GtkWidget      * widget,
                                        GdkEventButton * event,
                                        GtkGui 			   * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_release_event(GtkWidget      * widget,
                                          GdkEventButton * event,
                                          GtkGui 	       * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
                                         GdkEventMotion * event,
                                         GtkGui 		    * gui);


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_key_press_event(GtkWidget   * widget,
                                     GdkEventKey * event,
                                     GtkGui 	   * gui);

extern "C" G_MODULE_EXPORT
void on_winMain_destroy (GtkObject *object, GtkGui  * gui);

extern "C" G_MODULE_EXPORT
void on_ntbkViewOptions_switch_page (GtkNotebook    * notebook,
                                     GtkNotebookPage* page,
                                     guint            page_num,
                                     GtkGui         * gui);

extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget,
                        GtkGui    * gui);

extern "C" G_MODULE_EXPORT
void on_rdbFollowCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_rdbStaticCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_rdbDriverCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawRobot_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawDistanceInformation_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawLandmarks_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawCorrectedPath_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawVisualOdometryPath_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawParticles_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GtkGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          gpointer  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          gpointer  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisSize_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_txtZPlane_value_changed (GtkSpinButton* spinbutton,
		gpointer      data);

extern "C" G_MODULE_EXPORT
void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton,
		gpointer       data);

extern "C" G_MODULE_EXPORT
void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton,
		gpointer       data);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffAbove_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui);


extern "C" G_MODULE_EXPORT
void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
		gpointer       data);

void updateUIFromCamera(GtkGui* gui);

}

#endif

