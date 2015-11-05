#include <draw_callbacks.h>


double mx;
double my;
bool flatView = false;
bool mouseLeftIsDown = false;
bool mouseRightIsDown = false;

namespace CVIS
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
					 	              GTKGui 	*gui __attribute__((unused)))
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext)) return;

	glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_CULL_FACE);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glClearColor(0, 0, 0, 0);                   // background color
	glClearStencil(0);                          // clear stencil buffer
	glClearDepth(1.0f);                         // 0 is near, 1 is far
	glDepthFunc(GL_LEQUAL);

	GLfloat lightKa[] = {.8f, .8f, .8f, 1.0f};  // ambient light
	GLfloat lightKd[] = {.7f, .7f, .7f, 1.0f};  // diffuse light
	GLfloat lightKs[] = {1, 1, 1, 1};           // specular light

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

	//position the light
	float lightPos[4] = {0, 0, 20, 1}; // positional light
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	glEnable(GL_LIGHT0);

	gdk_gl_drawable_gl_end (gldrawable);
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_configure_event (GtkWidget         *widget __attribute__((unused)),
                                 	    GdkEventConfigure *event,
                                 	    GTKGui 			*gui)
{
  gui->setCanvasWidth (event->width);
  gui->setCanvasHeight(event->height);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_expose_event (GtkWidget       *widget,
                             	     GdkEventExpose  *event __attribute__((unused)),
                                   GTKGui 	 *gui)
{
	GdkGLContext  *glcontext  = gtk_widget_get_gl_context  (widget);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (widget);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		printf ("error in gdk_gl_drawable_gl_begin\n");
		return FALSE;
	}

	glClearColor(gui->options_.backgroundR, gui->options_.backgroundG, gui->options_.backgroundB, 0);
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
                                        GTKGui  	     * gui)
{
  gui->mouseDown(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_button_release_event(GtkWidget      * widget __attribute__((unused)),
                                          GdkEventButton * event,
                                          GTKGui  	     * gui)
{
  gui->mouseUp(event->x, event->y, event->button);
	return TRUE;
}

extern "C" G_MODULE_EXPORT
gboolean on_drawArea_motion_notify_event(GtkWidget      * widget __attribute__((unused)),
                                         GdkEventMotion * event,
                                         GTKGui  	      * gui)
{
  gui->mouseMove(event->x*3, event->y*3);
	return TRUE;
}



extern "C" G_MODULE_EXPORT
gboolean on_drawArea_key_press_event(GtkWidget   * widget __attribute__((unused)),
                                     GdkEventKey * event __attribute__((unused)),
                                     GTKGui 	   * gui __attribute__((unused)))
{
  //if (event->keyval == GDK_a)

	return TRUE;
}

extern "C" G_MODULE_EXPORT
void on_winMain_realize(GtkWidget * widget __attribute__((unused)),
                        GTKGui    * gui)
{
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_winMain_destroy (GtkObject *object __attribute__((unused)), GTKGui   * gui __attribute__((unused)))
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
void on_togglebutton1_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito1enabled = gtk_toggle_button_get_active(togglebutton);
}

extern "C" G_MODULE_EXPORT
void on_togglebutton2_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito2enabled = gtk_toggle_button_get_active(togglebutton);

	if(gui->options_.efeito2enabled)
	{
		gui->getDrawer3D()->setVertexBufferObjects(new CVIS::Kinect(640, 480, 575.815735f));
		gui->getDrawer3D()->getVertexBufferObjects()->AllocatePointCloudVBO(307200, 3, 3);
	}
	else
	{
		gui->getDrawer3D()->setVertexBufferObjects(new CVIS::Kinect(640, 480, 575.815735f));
		gui->getDrawer3D()->getVertexBufferObjects()->AllocatePointCloudVBO(307200, 3, 1);
	}
}

extern "C" G_MODULE_EXPORT
void on_togglebutton3_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito3enabled = gtk_toggle_button_get_active(togglebutton);
}

extern "C" G_MODULE_EXPORT
void on_togglebutton4_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito4enabled = gtk_toggle_button_get_active(togglebutton);
}

extern "C" G_MODULE_EXPORT
void on_togglebutton5_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito5enabled = gtk_toggle_button_get_active(togglebutton);
}

extern "C" G_MODULE_EXPORT
void on_togglebutton10_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.efeito10enabled = gtk_toggle_button_get_active(togglebutton);
}

extern "C" G_MODULE_EXPORT
void on_ntbkViewOptions_switch_page (GtkNotebook     * notebook __attribute__((unused)),
                                     GtkNotebookPage * page __attribute__((unused)),
                                     guint             page_num __attribute__((unused)),
                                     GTKGui          * gui __attribute__((unused)))
{

}

extern "C" G_MODULE_EXPORT
void on_btnDrawColor_toggled (GtkToggleButton * togglebutton,
                                GTKGui          * gui)
{
	bool val = gtk_toggle_button_get_active(togglebutton);
	gui->setDrawColor(val);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnDrawObstacles_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui)
{
	gtk_toggle_button_get_active(togglebutton);
	gui->updateControls();
}


extern "C" G_MODULE_EXPORT
void on_btnDrawPVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui)
{
	gtk_toggle_button_get_active(togglebutton);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnDrawNVolumes_toggled (GtkToggleButton * togglebutton,
                                 GTKGui          * gui)
{
	gtk_toggle_button_get_active(togglebutton);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnColorByHeight_toggled (GtkToggleButton * togglebutton,
                                  GTKGui          * gui)
{
  gtk_toggle_button_get_active(togglebutton);
	gui->updateControls();
}

extern "C" G_MODULE_EXPORT
void on_btnHighlightUnknown_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
																				gpointer data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);
	data->map->setHighlightUnknown(val);
	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnDrawNDensity_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
                                 gpointer        data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawNDensity(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btn3DZoomOut_pressed (GtkButton *button __attribute__((unused)),
							  GTKGui* gui)
{
	gui->getDrawer3D()->zoom(-0.5);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btn3DZoomIn_pressed (GtkButton* button __attribute__((unused)),
							 GTKGui* gui)
{
	gui->getDrawer3D()->zoom(0.5);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
gboolean on_txtCamPosX_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
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
                                       GdkEventFocus *event __attribute__((unused)),
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
                                       GdkEventFocus *event __attribute__((unused)),
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
                                        GdkEventFocus* event __attribute__((unused)),
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
                                        GdkEventFocus* event __attribute__((unused)),
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
                                        GdkEventFocus* event __attribute__((unused)),
                                        GTKGui* gui)
{
	double val = atof(gtk_entry_get_text(entry));

	gui->getDrawer3D()->setCameraLookZ(val);
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);

	return FALSE;
}


extern "C" G_MODULE_EXPORT
void on_btnDrawIVolumes_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
                                 gpointer         data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawIVolumes(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnDrawRobot_toggled (GtkToggleButton* togglebutton,
		GTKGui* gui)
{

	gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawZPlane_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
		gpointer  data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

extern "C" G_MODULE_EXPORT
void on_btnCutOffZPlane_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
		gpointer         data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setCutOffZPlane(val);

	gtk_widget_draw(data->drawArea, NULL);*/
}

// **** 2D *************************************************

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
                                       GTKGui* gui)
{
	atof(gtk_entry_get_text(entry));
	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_btnDrawProbability_toggled (GtkToggleButton* togglebutton,
		GTKGui* gui)
{
  gtk_toggle_button_get_active(togglebutton);

	updateUIFromCamera(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_btnDrawPDensity_toggled (GtkToggleButton* togglebutton __attribute__((unused)),
		gpointer        data __attribute__((unused)))
{
	/*bool val = gtk_toggle_button_get_active(togglebutton);

	data->map->setDrawPDensity(val);

	gtk_widget_draw(data->drawArea, NULL);*/
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
		GTKGui* gui)
{
	gtk_spin_button_get_value(spinbutton);

	//updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleCutoffBelow_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui)
{
	gtk_spin_button_get_value(spinbutton);

	//updateUIFromMap(gui);
	gtk_widget_draw(gui->getControls()->drawArea, NULL);
}

extern "C" G_MODULE_EXPORT
void on_txtObstacleMaxHeight_value_changed (GtkSpinButton* spinbutton,
		GTKGui* gui)
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

/** pointcloud effects controls **/

extern "C" G_MODULE_EXPORT
gboolean on_tbxCloudR_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCloudR = ((double) atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCloudG_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{

	gui->options_.valueCloudG = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCloudB_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCloudB = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_tbxBackgroundR_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.backgroundR = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxBackgroundG_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.backgroundG = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxBackgroundB_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.backgroundB = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_tbxCor1R_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor1R = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor1G_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor1G = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor1B_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor1B = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_tbxCor2R_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor2R = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor2G_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor2G = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor2B_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor2B = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}


extern "C" G_MODULE_EXPORT
gboolean on_tbxCor3R_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor3R = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor3G_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor3G = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
gboolean on_tbxCor3B_focus_out_event(GtkEntry    *entry,
                                       GdkEventFocus *event __attribute__((unused)),
                                       GTKGui* gui)
{
	gui->options_.valueCor3B = ((double)atoi(gtk_entry_get_text(entry))) / 255.0;
	//gtk_widget_draw(gui->getControls()->drawArea, NULL);
	return FALSE;
}

extern "C" G_MODULE_EXPORT
void on_spbChangeSpeed_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);
	gui->options_.changeColorSpeed = val;
}

extern "C" G_MODULE_EXPORT
void on_spbForeRange_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valueForegroundRange = val;
}

extern "C" G_MODULE_EXPORT
void on_spbBackRange_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	double val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valueBackgroundRange = val;
}

extern "C" G_MODULE_EXPORT
void on_spbCloudDensity_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	CVIS::Kinect* kinect = (CVIS::Kinect* )(gui->getDrawer3D()->getVertexBufferObjects());

	kinect->clean_memory = 1;

	int val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valueCloudDensity = val;
}


extern "C" G_MODULE_EXPORT
void on_spbPointSize_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	int val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valuePointSize = val;
}

extern "C" G_MODULE_EXPORT
void on_spbVelTranslation_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	int val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valueVelTranslation = val;
}

extern "C" G_MODULE_EXPORT
void on_spbVelZoom_value_changed (GtkSpinButton* spinbutton __attribute__((unused)),
										GTKGui* gui)
{
	int val = gtk_spin_button_get_value(spinbutton);
	gui->options_.valueVelZoom = val;
}


extern "C" G_MODULE_EXPORT
void on_btnAutoZoom_toggled(GtkToggleButton * togglebutton,
                          GTKGui          * gui)
{
	gui->options_.autoZoom = gtk_toggle_button_get_active(togglebutton);
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

}

}
