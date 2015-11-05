#ifndef MVOG_GTK_GUI_CALLBACKS_H
#define MVOG_GTK_GUI_CALLBACKS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <mvog_gtk_gui/gtk_gui.h>

using namespace std;

namespace MVOG
{

class GTKGui;

/*extern "C" G_MODULE_EXPORT
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
void on_btn3DZoomOut_pressed (GtkButton *button,
							  AppData* data);

extern "C" G_MODULE_EXPORT
void on_btn3DZoomIn_pressed (GtkButton* button,
							 AppData*   data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data);

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data);



extern "C" G_MODULE_EXPORT
void on_btnDrawIVolumes_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawCloud_toggled (GtkToggleButton* togglebutton,
                              AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawZPlane_toggled (GtkToggleButton* togglebutton,
                        			 AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnCutOffZPlane_toggled (GtkToggleButton* togglebutton,
                              	 AppData*         data);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          AppData*  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          AppData*  data);

extern "C" G_MODULE_EXPORT
gboolean on_txtVisSize_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data);

extern "C" G_MODULE_EXPORT
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton,
                                     AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawProbability_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawPDensity_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data);

extern "C" G_MODULE_EXPORT
void on_btnDrawNDensity_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data);

extern "C" G_MODULE_EXPORT
void on_txtZPlane_value_changed (GtkSpinButton* spinbutton,
                                 AppData*       data);

extern "C" G_MODULE_EXPORT
void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton,
                                        AppData*       data);

extern "C" G_MODULE_EXPORT
void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton,
                                         AppData*       data);

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
                                         AppData*       data);

void updateUIFromCamera(AppData* data);

void updateUIFromMap(AppData* data);
*/
} // namespace MVOG

#endif //MVOG_GTK_GUI_CALLBACKS_H

