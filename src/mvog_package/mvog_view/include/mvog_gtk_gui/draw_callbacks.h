#ifndef MVOG_GTK_GUI_DRAW_CALLBACKS_H
#define MVOG_GTK_GUI_DRAW_CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <gdk/gdkkeysyms.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <mvog_gtk_gui/gtk_gui.h>
//#include <mvog_gtk_gui/callbacks.h>

namespace MVOG
{

class GTKGui;

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void * data);


extern "C" G_MODULE_EXPORT
void on_drawArea_realize (GtkWidget * widget,
					 	            	GTKGui 	  * gui);


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_configure_event(GtkWidget         * widget,
                                 	   GdkEventConfigure * event,
                                 	   GTKGui 			     * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_expose_event(GtkWidget      * widget,
                             	    GdkEventExpose * event,
                                  GTKGui 			   * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_press_event(GtkWidget      * widget,
                                        GdkEventButton * event,
                                        GTKGui 			   * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_release_event(GtkWidget      * widget,
                                          GdkEventButton * event,
                                          GTKGui 	       * gui);

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
                                         GdkEventMotion * event,
                                         GTKGui 		    * gui);


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_key_press_event(GtkWidget   * widget,
                                     GdkEventKey * event,
                                     GTKGui 	   * gui);
extern "C" G_MODULE_EXPORT
void on_winMain_destroy (GtkObject *object, GTKGui  * gui);

extern "C" G_MODULE_EXPORT
void on_ntbkViewOptions_switch_page (GtkNotebook    * notebook,
                                     GtkNotebookPage* page,
                                     guint            page_num,
                                     GTKGui         * gui);

extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget,
                        GTKGui    * gui);

extern "C" G_MODULE_EXPORT
void on_btnView2D_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnView3D_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawRawData_toggled (GtkToggleButton * togglebutton,
                                GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawObstacles_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawPVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawNVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnColorByHeight_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui);

extern "C" G_MODULE_EXPORT
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton,
																				gpointer data);

extern "C" G_MODULE_EXPORT
void on_btn3DZoomOut_pressed (GtkButton *button,
		GTKGui* gui);

extern "C" G_MODULE_EXPORT
void on_btn3DZoomIn_pressed (GtkButton* button,
		GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui);



extern "C" G_MODULE_EXPORT
void on_btnDrawIVolumes_toggled (GtkToggleButton* togglebutton,
		gpointer         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawCloud_toggled (GtkToggleButton* togglebutton,
		gpointer         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawZPlane_toggled (GtkToggleButton* togglebutton,
		gpointer         data);

extern "C" G_MODULE_EXPORT
void on_btnCutOffZPlane_toggled (GtkToggleButton* togglebutton,
		gpointer         data);

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
                                       GTKGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton,
		gpointer         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawProbability_toggled (GtkToggleButton* togglebutton,
		GTKGui* gui);

extern "C" G_MODULE_EXPORT
void on_btnDrawPDensity_toggled (GtkToggleButton* togglebutton,
		gpointer        data);

extern "C" G_MODULE_EXPORT
void on_btnDrawNDensity_toggled (GtkToggleButton* togglebutton,
		gpointer       data);

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
		GTKGui* gui);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui);


extern "C" G_MODULE_EXPORT
void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
		gpointer       data);

void updateUIFromCamera(GTKGui* gui);

void updateUIFromMap(gpointer data);

//void set2DView(AppData* data);

}

#endif //MVOG_GTK_GUI_DRAW_CALLBACKS_H

