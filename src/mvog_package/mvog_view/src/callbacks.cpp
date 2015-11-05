#include <mvog_gtk_gui/callbacks.h>

namespace MVOG
{

/*extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget,
                        GTKGui    * gui)
{
	gui->setView();
	gui->updateControls();
	//updateUIFromMap(data);
	//updateUIFromCamera(data);
}

extern "C" G_MODULE_EXPORT
void on_winMain_destroy (GtkObject *object, GTKGui   * gui)
{
  printf("Closing GTK Gui.\n");
}

extern "C" G_MODULE_EXPORT
void on_btnView3D_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setView3D(val);
	gui->setView();
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnView2D_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setView3D(!val);
	gui->setView();
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_ntbkViewOptions_switch_page (GtkNotebook     * notebook,
                                     GtkNotebookPage * page,
                                     guint             page_num,
                                     GTKGui          * gui)
{

}

extern "C" G_MODULE_EXPORT
void on_btnDrawRawData_toggled (GtkToggleButton * togglebutton,
                                GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setDrawRawData(val);
	gui->setView();
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnDrawObstacles_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setDrawRawData(!val);
	gui->setView();
	gui->updateControls();
}


extern "C" G_MODULE_EXPORT
void on_btnDrawPVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
	gui->setDrawPVolumes(val);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnDrawNVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
	gui->setDrawNVolumes(val);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnColorByHeight_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui)
{
  bool val = gtk_toggle_button_get_active(togglebutton);
	gui->setColorByHeight(val);
	gui->updateControls();
}


extern "C" G_MODULE_EXPORT
void on_btn3DZoomOut_pressed (GtkButton *button,
							  AppData* data)
{
	data->c->zoom(-0.2);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btn3DZoomIn_pressed (GtkButton* button,
							 AppData*   data)
{
	data->c->zoom(+0.2);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setPosX(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setPosY(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setPosZ(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setLookX(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setLookY(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        AppData*       data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->c->setLookZ(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
void on_btnDrawIVolumes_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawIVolumes(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawCloud_toggled (GtkToggleButton* togglebutton,
                              AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawCloud(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawZPlane_toggled (GtkToggleButton* togglebutton,
                        			 AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnCutOffZPlane_toggled (GtkToggleButton* togglebutton,
                              	AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setCutOffZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);
}

// **** 2D *************************************************

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->map->setVisCenterX(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->map->setVisCenterY(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisSize_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       AppData*  data)
{
	double val = atof(gtk_entry_get_text(entry));

	data->map->setVisSize(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton,
                                     AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setHighlightUnknown(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawProbability_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawProbability(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawPDensity_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawPDensity(val);

	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawNDensity_toggled (GtkToggleButton* togglebutton,
                                 AppData*         data)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawNDensity(val);

	gtk_widget_draw(data->drawArea, NULL);
}


extern "C" G_MODULE_EXPORT
void on_txtZPlane_value_changed (GtkSpinButton* spinbutton,
                                 AppData*       data)
{
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setZPlane(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton,
                                        AppData*       data)
{
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorScale(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton,
                                         AppData*       data)
{
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorOffset(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
                                         AppData*       data)
{
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setObstacleCutoff(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
}

void updateUIFromCamera(AppData* data)
{
	char s[10] = "";

	// update camera Position text fields
	sprintf(s, "%3.2f", data->c->getPosX());

	gtk_entry_set_text(data->txtCamPosX, s);

	sprintf(s, "%3.2f", data->c->getPosY());
	gtk_entry_set_text(data->txtCamPosY, s);

	sprintf(s, "%3.2f", data->c->getPosZ());
	gtk_entry_set_text(data->txtCamPosZ, s);

	// update camera LookAt text fields

	sprintf(s, "%3.2f", data->c->getLookX());
	gtk_entry_set_text(data->txtCamLookX, s);

	sprintf(s, "%3.2f", data->c->getLookY());
	gtk_entry_set_text(data->txtCamLookY, s);

	sprintf(s, "%3.2f", data->c->getLookZ());
	gtk_entry_set_text(data->txtCamLookZ, s);

	// update map center & size

	sprintf(s, "%3.0f", data->map->getVisCenterX());
	gtk_entry_set_text(data->txtVisCenterX, s);

	sprintf(s, "%3.0f", data->map->getVisCenterY());
	gtk_entry_set_text(data->txtVisCenterY, s);

	sprintf(s, "%3.0f", data->map->getVisSize());
	gtk_entry_set_text(data->txtVisSize, s);

	// update z plane
}

void updateUIFromMap(AppData* data)
{
	bool draw2D = data->map->getViewType2D();

	if(draw2D)
		gtk_notebook_set_current_page(data->ntbkViewOptions, 1);
	else
		gtk_notebook_set_current_page(data->ntbkViewOptions, 0);

	gtk_toggle_button_set_active(data->btnDrawPVolumes, data->map->getDrawPVolumes());
	gtk_toggle_button_set_active(data->btnDrawNVolumes, data->map->getDrawNVolumes());
	gtk_toggle_button_set_active(data->btnDrawIVolumes, data->map->getDrawIVolumes());
	gtk_toggle_button_set_active(data->btnDrawCloud,    data->map->getDrawCloud());
	gtk_toggle_button_set_active(data->btnDrawZPlane,   data->map->getDrawZPlane());
	gtk_toggle_button_set_active(data->btnCutOffZPlane, data->map->getCutOffZPlane());

	gtk_toggle_button_set_active(data->btnHighlightUnknown, data->map->getHighlightUnknown());

	gtk_spin_button_set_value(data->txtZPlane, data->map->getZPlane());
	gtk_spin_button_set_value(data->txtVerticalScale,  data->map->getVerticalColorScale());
	gtk_spin_button_set_value(data->txtVerticalOffset, data->map->getVerticalColorOffset());
	gtk_spin_button_set_value(data->txtObstacleCutoff, data->map->getObstacleCutoff());
}
*/
} // namespace MVOG
