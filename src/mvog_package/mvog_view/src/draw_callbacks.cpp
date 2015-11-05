#include <mvog_gtk_gui/draw_callbacks.h>


double mx;
double my;
bool flatView = false;
bool mouseLeftIsDown = false;
bool mouseRightIsDown = false;

namespace MVOG
{

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void * data)
{
	GTKGui * gui = static_cast<GTKGui*>(data);

  if (!GTK_IS_WIDGET(gui->getControls()->drawArea)) return TRUE;

  gtk_widget_draw(gui->getControls()->drawArea, NULL);

  return TRUE;
}


extern "C" G_MODULE_EXPORT
void on_drawArea_realize (GtkWidget *widget,
					 	              GTKGui 	*gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

	// choose background color
	glClearColor(1.0, 1.0, 1.0, 1.0);	// background color

	// shading type
	glShadeModel(GL_SMOOTH);

	// hidden surface
	glEnable(GL_DEPTH_TEST);

	gdk_gl_drawable_gl_end (gldrawable);
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_configure_event (GtkWidget         *widget,
                                 	    GdkEventConfigure *event,
                                 	    GTKGui 			*gui)
{
  gui->setCanvasWidth (event->width);
  gui->setCanvasHeight(event->height);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_expose_event (GtkWidget       *widget,
                             	     GdkEventExpose  *event,
                                   GTKGui 	 *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context  (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		printf ("error in gdk_gl_drawable_gl_begin\n");
		return FALSE;
	}

	// clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw map
	gui->draw();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);
	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_press_event(GtkWidget      * widget,
                                        GdkEventButton * event,
                                        GTKGui  	     * gui)
{
  gui->mouseDown(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_release_event(GtkWidget      * widget,
                                          GdkEventButton * event,
                                          GTKGui  	     * gui)
{
  gui->mouseUp(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget,
                                         GdkEventMotion * event,
                                         GTKGui  	      * gui)
{
  gui->mouseMove(event->x*3, event->y*3);
	return TRUE;
}



extern "C" G_MODULE_EXPORT
gboolean on_drawArea_key_press_event(GtkWidget   * widget,
                                     GdkEventKey * event,
                                     GTKGui 	   * gui)
{
  //if (event->keyval == GDK_a)

	return TRUE;
}

extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget,
                        GTKGui    * gui)
{
	gui->updateControls();
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
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnView2D_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setView3D(!val);
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
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnDrawObstacles_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
  gui->setDrawRawData(!val);
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
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton,
																				gpointer data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);
	data->map->setHighlightUnknown(val);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnDrawNDensity_toggled (GtkToggleButton* togglebutton,
                                 gpointer        data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawNDensity(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btn3DZoomOut_pressed (GtkButton *button,
							  GTKGui* gui)
{
	gui->getDrawer3D()->zoom(-0.5);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btn3DZoomIn_pressed (GtkButton* button,
							 GTKGui* gui)
{
	gui->getDrawer3D()->zoom(0.5);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosX(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosY(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosZ(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookX(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookY(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event,
                                        GTKGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookZ(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
void on_btnDrawIVolumes_toggled (GtkToggleButton* togglebutton,
                                 gpointer         data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawIVolumes(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnDrawRobot_toggled (GtkToggleButton* togglebutton,
		GTKGui* gui)
{

	bool val = gtk_toggle_button_get_active(togglebutton);

	gui->setDrawRobot(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawZPlane_toggled (GtkToggleButton* togglebutton,
		gpointer  data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnCutOffZPlane_toggled (GtkToggleButton* togglebutton,
		gpointer         data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setCutOffZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

// **** 2D *************************************************

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          gpointer  data)
{
	/*double val = atof(gtk_entry_get_text(entry));

	data->map->setVisCenterX(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);
*/
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry,
                                          GdkEventFocus *event,
                                          gpointer data)
{
	/*double val = atof(gtk_entry_get_text(entry));

	data->map->setVisCenterY(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);
*/
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisSize_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event,
                                       GTKGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));
	gui->getDrawer2D()->setVisSize(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_btnDrawProbability_toggled (GtkToggleButton* togglebutton,
		GTKGui* gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);

	gui->getDrawer2D()->setDrawProbability(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawPDensity_toggled (GtkToggleButton* togglebutton,
		gpointer        data)
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawPDensity(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_txtZPlane_value_changed (GtkSpinButton* spinbutton,
		gpointer      data)
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setZPlane(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton,
		gpointer      data)
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorScale(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton,
		gpointer       data)
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorOffset(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
	*/
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffAbove_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);

	gui->getDrawer3D()->setCutoffAbove(val);
	updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);

	gui->getDrawer3D()->setCutoffBelow(val);
	updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);

	gui->getDrawer3D()->setMaxHeight(val);
	updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton,
		gpointer      data)
{
	/*
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setObstacleCutoff(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
	*/
}

void updateUIFromCamera(GTKGui* gui)
{
	char s[10] = "";

	// update camera Position text fields
	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraPosX());
	gtk_entry_set_text(gui->getControls()->txtCamPosX, s);

	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraPosY());
	gtk_entry_set_text(gui->getControls()->txtCamPosY, s);

	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraPosZ());
	gtk_entry_set_text(gui->getControls()->txtCamPosZ, s);

	// update camera LookAt text fields
	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraLookX());
	gtk_entry_set_text(gui->getControls()->txtCamLookX, s);

	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraLookY());
	gtk_entry_set_text(gui->getControls()->txtCamLookY, s);

	sprintf(s, "%3.2f", gui->getDrawer3D()->getCameraLookZ());
	gtk_entry_set_text(gui->getControls()->txtCamLookZ, s);

	gtk_spin_button_set_value(gui->getControls()->txtObstacleMaxHeight, gui->getDrawer3D()->getMaxHeight());
	gtk_spin_button_set_value(gui->getControls()->txtObstacleCutoffAbove, gui->getDrawer3D()->getCutoffAbove());
	gtk_spin_button_set_value(gui->getControls()->txtObstacleCutoffBelow, gui->getDrawer3D()->getCutoffBelow());


	// update map center & size
	/*sprintf(s, "%3.0f", data->map->getVisCenterX());
	gtk_entry_set_text(data->txtVisCenterX, s);

	sprintf(s, "%3.0f", data->map->getVisCenterY());
	gtk_entry_set_text(data->txtVisCenterY, s);

	sprintf(s, "%3.0f", data->map->getVisSize());
	gtk_entry_set_text(data->txtVisSize, s);

	// update z plane
	*/
}

void updateUIFromMap(gpointer data)
{
	/*bool draw2D = data->map->getViewType2D();

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
	gtk_spin_button_set_value(data->txtObstacleCutoff, data->map->getObstacleCutoff());*/
}

/*
void set2DView(AppData* data)
{
	glMatrixMode(GL_PROJECTION);		// set clipping window
	glLoadIdentity();

	double visCenterX = data->map->getVisCenterX();
	double visCenterY = data->map->getVisCenterY();
	double visSize    = data->map->getVisSize();

	gluOrtho2D(visCenterX - (visSize/2.0),
			   visCenterX + (visSize/2.0),
	           visCenterY - visSize/2.0,
			   visCenterY + visSize/2.0);
}
*/

} // namespace MVOG
