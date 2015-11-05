#include <draw_callbacks.h>


double mx;
double my;
bool flatView = false;
bool mouseLeftIsDown = false;
bool mouseRightIsDown = false;

namespace View
{

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_idle(void * data)
{
	GtkGui * gui = static_cast<GtkGui*>(data);

	if (!GTK_IS_WIDGET(gui->getControls()->drawArea)) return TRUE;

	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return TRUE;
}


extern "C" G_MODULE_EXPORT
void on_drawArea_realize (GtkWidget *widget,
					 	              GtkGui 	*gui __attribute__((unused)))
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

	glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glLightModelf(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glClearColor(0, 0, 0, 0);                   // background color
	glClearStencil(0);                          // clear stencil buffer
	glClearDepth(1.0f);                         // 0 is near, 1 is far
	glDepthFunc(GL_LEQUAL);

//	GLfloat lightKa[] = {1, 1, 1, 1.0f};  // ambient light
//	GLfloat lightKd[] = {1, 1, 1, 1.0f};  // diffuse light
//	GLfloat lightKs[] = {1, 1, 1, 1};           // specular light

//	glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
//	glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

	// position the light
//	float lightPos[4] = {0, 0, 20, 1}; // positional light
//	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
//
//	glEnable(GL_LIGHT0);

	gdk_gl_drawable_gl_end (gldrawable);
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_configure_event (GtkWidget         *widget __attribute__((unused)),
                                 	    GdkEventConfigure *event,
                                 	    GtkGui 			*gui)
{
  gui->setCanvasWidth (event->width);
  gui->setCanvasHeight(event->height);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_expose_event (GtkWidget       *widget,
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
	gui->draw();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);
	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}


extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_press_event(GtkWidget      * widget __attribute__((unused)),
                                        GdkEventButton * event,
                                        GtkGui  	     * gui)
{
  gui->mouseDown(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_release_event(GtkWidget      * widget __attribute__((unused)),
                                          GdkEventButton * event,
                                          GtkGui  	     * gui)
{
  gui->mouseUp(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget __attribute__((unused)),
                                         GdkEventMotion * event,
                                         GtkGui  	      * gui)
{
  gui->mouseMove(event->x*3, event->y*3);
	return TRUE;
}



extern "C" G_MODULE_EXPORT
gboolean on_drawArea_key_press_event(GtkWidget   * widget __attribute__((unused)),
                                     GdkEventKey * event __attribute__((unused)),
                                     GtkGui 	   * gui __attribute__((unused)))
{
  //if (event->keyval == GDK_a)

	return TRUE;
}

extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget __attribute__((unused)),
                        GtkGui    * gui)
{
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_winMain_destroy (GtkObject *object __attribute__((unused)), GtkGui   * gui __attribute__((unused)))
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  printf("Closing GTK Gui.\n");
}

extern "C" G_MODULE_EXPORT
void on_ntbkViewOptions_switch_page (GtkNotebook     * notebook __attribute__((unused)),
                                     GtkNotebookPage * page __attribute__((unused)),
                                     guint             page_num __attribute__((unused)),
                                     GtkGui          * gui __attribute__((unused)))
{

}

extern "C" G_MODULE_EXPORT
void on_rdbFollowCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->useFollowCamera = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_rdbStaticCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->useStaticCamera = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_rdbDriverCamera_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->useDriverCamera = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawDistanceInformation_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->drawDistanceInformation = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawSaliencies_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->drawSaliencyPoints = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawCorrectedPath_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->drawCorrectedPath = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawVisualOdometryPath_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->drawVisualOdometryPath = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawParticles_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{
	gui->getOptions()->drawParticles = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GtkGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosX(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosY_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GtkGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosY(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosZ_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GtkGui*  gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraPosZ(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookX_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event __attribute__((unused)),
                                        GtkGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookX(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookY_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event __attribute__((unused)),
                                        GtkGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookY(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamLookZ_focus_out_event(GtkEntry*      entry,
                                        GdkEventFocus* event __attribute__((unused)),
                                        GtkGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookZ(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_btnDrawRobot_toggled (GtkToggleButton* togglebutton,
		GtkGui* gui)
{

	gui->getOptions()->drawRobot = gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterX_focus_out_event(GtkEntry    *entry __attribute__((unused)),
                                          GdkEventFocus *event __attribute__((unused)),
                                          gpointer  data __attribute__((unused)))
{
	/*double val = atof(gtk_entry_get_text(entry));

	data->map->setVisCenterX(val);

	updateUIFromCamera(data);
	gtk_widget_draw(data->drawArea, NULL);
*/
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_txtVisCenterY_focus_out_event(GtkEntry    *entry __attribute__((unused)),
                                          GdkEventFocus *event __attribute__((unused)),
                                          gpointer data __attribute__((unused)))
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
                                       GdkEventFocus *event __attribute__((unused)),
                                       GtkGui* gui)
{
	atof(gtk_entry_get_text(entry));
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_txtZPlane_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
		gpointer      data __attribute__((unused)))
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setZPlane(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalScale_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
		gpointer      data __attribute__((unused)))
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorScale(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_txtVerticalOffset_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
		gpointer       data __attribute__((unused)))
{
	/*double val = gtk_spin_button_get_value(spinbutton);

	data->map->setVerticalColorOffset(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
	*/
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffAbove_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui)
{
	gtk_spin_button_get_value(spinbutton);

	//updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui)
{
	gtk_spin_button_get_value(spinbutton);

	//updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
		GtkGui* gui)
{
	gtk_spin_button_get_value(spinbutton);

	//updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoff_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
		gpointer      data __attribute__((unused)))
{
	/*
	double val = gtk_spin_button_get_value(spinbutton);

	data->map->setObstacleCutoff(val);

	updateUIFromMap(data);
	gtk_widget_draw(data->drawArea, NULL);
	*/
}

void updateUIFromCamera(GtkGui* gui)
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

}

}
