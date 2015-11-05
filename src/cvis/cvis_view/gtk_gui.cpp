#include <gtk_gui.h>

namespace CVIS
{

GTKGui::GTKGui()
{
  options_.view3D = true;
  options_.drawColor = true;

  options_.valueCloudR = 0.5;
  options_.valueCloudG = 0.5;
  options_.valueCloudB = 0.5;

  options_.backgroundR = 0.0;
  options_.backgroundG = 0.0;
  options_.backgroundB = 0.0;

  options_.valueCor1R = 1.0;
  options_.valueCor1G = 0.0;
  options_.valueCor1B = 0.0;

  options_.valueCor2R = 0.0;
  options_.valueCor2G = 1.0;
  options_.valueCor2B = 0.0;

  options_.valueCor3R = 0.0;
  options_.valueCor3G = 0.0;
  options_.valueCor3B = 1.0;

  options_.changeColorSpeed = 0.1;
  options_.valueBackgroundRange = 6.0;
  options_.valueForegroundRange = 0.6;

  options_.valueCloudDensity = 1;
  options_.valuePointSize = 1;

  options_.valueVelZoom = 5;
  options_.valueVelTranslation = 5;

  mouseLeftIsDown_  = false;
  mouseMidIsDown_   = false;
  mouseRightIsDown_ = false;

  drawer3D_ = new Drawer3D(this);
}

GTKGui::~GTKGui()
{

}

void GTKGui::updateControls()
{
  char s[10] = "";

  // **** update 3D/2D radio button
  gtk_toggle_button_set_active(controls_.btnView3D,  options_.view3D);
  gtk_toggle_button_set_active(controls_.btnView2D, !options_.view3D);

  if(options_.view3D)
  {
    gtk_widget_show(controls_.frameCamera);
    gtk_widget_show(controls_.frame3DMapOptions);
    gtk_widget_show(controls_.frame3DOtherOptions);
  }
  else
  {
    gtk_widget_hide(controls_.frameCamera);
    gtk_widget_hide(controls_.frame3DMapOptions);
    gtk_widget_hide(controls_.frame3DOtherOptions);
  }

	// update camera Position text fields
	sprintf(s, "%3.2f", drawer3D_->getCameraPosX());
	gtk_entry_set_text(controls_.txtCamPosX, s);

	sprintf(s, "%3.2f", drawer3D_->getCameraPosY());
	gtk_entry_set_text(controls_.txtCamPosY, s);

	sprintf(s, "%3.2f", drawer3D_->getCameraPosZ());
	gtk_entry_set_text(controls_.txtCamPosZ, s);

	// update camera LookAt text fields
	sprintf(s, "%3.2f", drawer3D_->getCameraLookX());
	gtk_entry_set_text(controls_.txtCamLookX, s);

	sprintf(s, "%3.2f", drawer3D_->getCameraLookY());
	gtk_entry_set_text(controls_.txtCamLookY, s);

	sprintf(s, "%3.2f", drawer3D_->getCameraLookZ());
	gtk_entry_set_text(controls_.txtCamLookZ, s);

	sprintf(s, "%d", (int)(this->options_.valueCloudR * 255.0));
	gtk_entry_set_text(controls_.tbxCloudR, s);

	sprintf(s, "%d", (int)(this->options_.valueCloudG * 255.0));
	gtk_entry_set_text(controls_.tbxCloudG, s);

	sprintf(s, "%d", (int)(this->options_.valueCloudB * 255.0));
	gtk_entry_set_text(controls_.tbxCloudB, s);

	sprintf(s, "%d", (int)(this->options_.backgroundR * 255.0));
	gtk_entry_set_text(controls_.tbxBackgroundR, s);

	sprintf(s, "%d", (int)(this->options_.backgroundG * 255.0));
	gtk_entry_set_text(controls_.tbxBackgroundG, s);

	sprintf(s, "%d", (int)(this->options_.backgroundB * 255.0));
	gtk_entry_set_text(controls_.tbxBackgroundB, s);

	sprintf(s, "%d", (int)(this->options_.valueCor1R * 255.0));
	gtk_entry_set_text(controls_.tbxCor1R, s);

	sprintf(s, "%d", (int)(this->options_.valueCor1G * 255.0));
	gtk_entry_set_text(controls_.tbxCor1G, s);

	sprintf(s, "%d", (int)(this->options_.valueCor1B * 255.0));
	gtk_entry_set_text(controls_.tbxCor1B, s);

	sprintf(s, "%d", (int)(this->options_.valueCor2R * 255.0));
	gtk_entry_set_text(controls_.tbxCor2R, s);

	sprintf(s, "%d", (int)(this->options_.valueCor2G * 255.0));
	gtk_entry_set_text(controls_.tbxCor2G, s);

	sprintf(s, "%d", (int)(this->options_.valueCor2B * 255.0));
	gtk_entry_set_text(controls_.tbxCor2B, s);

	sprintf(s, "%d", (int)(this->options_.valueCor3R * 255.0));
	gtk_entry_set_text(controls_.tbxCor3R, s);

	sprintf(s, "%d", (int)(this->options_.valueCor3G * 255.0));
	gtk_entry_set_text(controls_.tbxCor3G, s);

	sprintf(s, "%d", (int)(this->options_.valueCor3B * 255.0));
	gtk_entry_set_text(controls_.tbxCor3B, s);

	gtk_spin_button_set_value(controls_.spbChangeSpeed, this->options_.changeColorSpeed);
	gtk_spin_button_set_value(controls_.spbForeRange, this->options_.valueForegroundRange);
	gtk_spin_button_set_value(controls_.spbBackRange, this->options_.valueBackgroundRange);

	gtk_spin_button_set_value(controls_.spbCloudDensity, this->options_.valueCloudDensity);
	gtk_spin_button_set_value(controls_.spbPointSize, this->options_.valuePointSize);

	gtk_spin_button_set_value(controls_.spbVelTranslation, this->options_.valueVelTranslation);
	gtk_spin_button_set_value(controls_.spbVelZoom, this->options_.valueVelZoom);
}

void GTKGui::mouseDown(double x __attribute__((unused)), double y __attribute__((unused)), int button)
{
  if      (button == 1) mouseLeftIsDown_  = true;
  else if (button == 2) mouseMidIsDown_   = true;
  else if (button == 3) mouseRightIsDown_ = true;
}

void GTKGui::mouseUp(double x __attribute__((unused)), double y __attribute__((unused)), int button)
{
  if      (button == 1) mouseLeftIsDown_  = false;
  else if (button == 2) mouseMidIsDown_   = false;
  else if (button == 3) mouseRightIsDown_ = false;
}

void GTKGui::mouseMove(double x, double y)
{
  if (options_.view3D)
  {
    if (mouseMidIsDown_ || (mouseLeftIsDown_ && mouseRightIsDown_))
    {
      drawer3D_->move((mouseX_ - x) * MOVE_SPEED * options_.valueVelTranslation / canvasHeight_,
                      (mouseY_ - y) * MOVE_SPEED * options_.valueVelTranslation / canvasHeight_);
      updateControls();
    }
    else if (mouseLeftIsDown_)
    {
			drawer3D_->pan ((mouseX_ - x) * PAN_SPEED);
			drawer3D_->tilt((mouseY_ - y) * TILT_SPEED);
      updateControls();
    }
    else if (mouseRightIsDown_)
    {
      drawer3D_->zoom(1.0 - (mouseY_ - y) * ZOOM_SPEED * options_.valueVelZoom);
      updateControls();
    }
  }
  else
  {

  }

  mouseX_ = x;
  mouseY_ = y;
}

void GTKGui::joystickMove(carmen_joystick_status_message* message)
{
	static double xc = 0.0;
	static double yc = 0.0;

	xc += (message->axes[0] / 32792.0) * 10.0;
	yc += (-message->axes[1] / 32792.0) * 10.0;

	if (options_.view3D)
	  {
	    if (message->buttons[1] || (message->buttons[0] && message->buttons[2]))
	    {
	      drawer3D_->move((joystickX_ - xc) * MOVE_SPEED * options_.valueVelTranslation / canvasWidth_,
	                      (joystickY_ - yc) * MOVE_SPEED * options_.valueVelTranslation / canvasHeight_);
	      updateControls();
	    }
	    else if (message->buttons[0])
	    {
				drawer3D_->pan ((joystickX_ - xc) * PAN_SPEED);
				drawer3D_->tilt((joystickY_ - yc) * TILT_SPEED);
	      updateControls();
	    }
	    else if (message->buttons[2])
	    {
	      drawer3D_->zoom(1.0 - (joystickY_ - yc) * ZOOM_SPEED * options_.valueVelZoom);
	      updateControls();
	    }
	  }
	  else
	  {

	  }

	  joystickX_ = xc;
	  joystickY_ = yc;
}


void GTKGui::setView()
{
  if (options_.view3D)
  	drawer3D_->setView();
}

void GTKGui::draw()
{
  if (options_.view3D)
  	drawer3D_->draw();
}

void GTKGui::setUpGTK()
{
  GdkGLConfig *glconfig;
  GtkBuilder  *builder;
  GError      *error = NULL;

  gtk_init(0, NULL);
  gtk_gl_init(0, NULL);

  //Try double-buffered visual
  glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (
								                      GDK_GL_MODE_RGB    |
                                      GDK_GL_MODE_DEPTH  |
                                      GDK_GL_MODE_DOUBLE));
  if (glconfig == NULL)
  {
   	g_print ("*** Cannot find the double-buffered visual.\n");
   	g_print ("*** Trying single-buffered visual.\n");

    // Try single-buffered visual
   	glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (
									                        GDK_GL_MODE_RGB   |
                                          GDK_GL_MODE_DEPTH));
    if (glconfig == NULL)
	    g_print ("*** No appropriate OpenGL-capable visual found.\n");
  }

  // Create new GtkBuilder object
  builder = gtk_builder_new();
  if( ! gtk_builder_add_from_file(builder, "../../carmen/data/gui/cvis.glade", &error ) )
  {
    g_warning( "%s", error->message );
    g_free( error );
  }

  sleep(1.0);

  controls_.winMain  = GTK_WIDGET(gtk_builder_get_object(builder, "winMain" ));
  controls_.drawArea = GTK_WIDGET(gtk_builder_get_object(builder, "drawArea"));


  controls_.btnView2D = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnView2D"));
  controls_.btnView3D = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnView3D"));

  controls_.btnEfeito1 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito1"));
  controls_.btnEfeito2 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito2"));
  controls_.btnEfeito3 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito3"));
  controls_.btnEfeito4 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito4"));
  controls_.btnEfeito5 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito5"));
  controls_.btnEfeito10 = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnEfeito10"));

  controls_.frameCamera         = GTK_WIDGET(gtk_builder_get_object(builder, "frameCamera"));
  controls_.frame3DMapOptions   = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DMapOptions"));
  controls_.frame3DOtherOptions = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DOtherOptions"));

  controls_.vboxRawOptions           = GTK_WIDGET(gtk_builder_get_object(builder, "vboxRawOptions" ));
  controls_.vboxObstacleOptions      = GTK_WIDGET(gtk_builder_get_object(builder, "vboxObstacleOptions" ));
  controls_.vboxColorByHeightOptions = GTK_WIDGET(gtk_builder_get_object(builder, "vboxColorByHeightOptions" ));

  controls_.ntbkViewOptions = GTK_NOTEBOOK(gtk_builder_get_object(builder, "ntbkViewOptions"));

  controls_.txtCamPosX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosX"));
  controls_.txtCamPosY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosY"));
  controls_.txtCamPosZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosZ"));

  controls_.txtCamLookX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookX"));
  controls_.txtCamLookY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookY"));
  controls_.txtCamLookZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookZ"));

  controls_.tbxCloudR = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCloudR"));
  controls_.tbxCloudG = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCloudG"));
  controls_.tbxCloudB = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCloudB"));

  controls_.tbxBackgroundR = GTK_ENTRY(gtk_builder_get_object(builder, "tbxBackgroundR"));
  controls_.tbxBackgroundG = GTK_ENTRY(gtk_builder_get_object(builder, "tbxBackgroundG"));
  controls_.tbxBackgroundB = GTK_ENTRY(gtk_builder_get_object(builder, "tbxBackgroundB"));

  controls_.tbxCor1R = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor1R"));
  controls_.tbxCor1G = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor1G"));
  controls_.tbxCor1B = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor1B"));

  controls_.tbxCor2R = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor2R"));
  controls_.tbxCor2G = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor2G"));
  controls_.tbxCor2B = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor2B"));

  controls_.tbxCor3R = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor3R"));
  controls_.tbxCor3G = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor3G"));
  controls_.tbxCor3B = GTK_ENTRY(gtk_builder_get_object(builder, "tbxCor3B"));

  controls_.spbChangeSpeed = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbChangeSpeed"));
  controls_.spbForeRange = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbForeRange"));
  controls_.spbBackRange = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbBackRange"));

  controls_.spbCloudDensity = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbCloudDensity"));
  controls_.spbPointSize = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbPointSize"));

  controls_.spbVelTranslation = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbVelTranslation"));
  controls_.spbVelZoom = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "spbVelZoom"));

  controls_.btnAutoZoom = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnAutoZoom"));

  // Connect signals
  gtk_builder_connect_signals(builder, this);

  // Redraws
  gtk_container_set_reallocate_redraws (GTK_CONTAINER(controls_.winMain), TRUE);

  // Add OpenGL-capability to drawArea.
  if(!gtk_widget_set_gl_capability (controls_.drawArea, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE))
	  return;

  // Register idle function
  g_idle_add (on_drawArea_idle, this);

  // Destroy builder, since we don't need it anymore
  g_object_unref( G_OBJECT( builder ) );

  // Show window. All other widgets are automatically shown by GtkBuilder
  gtk_widget_show(controls_.winMain);
}

void GTKGui::setUpOpenGL(bool velodyneCloud, bool kinectCloud, bool slam6dCloud)
{
	this->drawKinectCloud_ = kinectCloud;
	this->drawSlam6DCloud_ = slam6dCloud;
	this->drawVelodyneCloud_ = velodyneCloud;

//	if(velodyneCloud)
//	{
//		drawer3D_->setVertexBufferObjects(new CVIS::Velodyne(1125));
//
//		CVIS::Velodyne* velodyne = (CVIS::Velodyne*) drawer3D_->getVertexBufferObjects();
//		velodyne->AllocatePointCloudVBO(140000, 3, 1);
//	}
//	else if(kinectCloud)
//	{
		drawer3D_->setVertexBufferObjects(new CVIS::Kinect(640, 480, 575.815735f));
		drawer3D_->getVertexBufferObjects()->AllocatePointCloudVBO(307200, 6, 1);
//	}
//	else if(slam6dCloud)
//	{
//		drawer3D_->setVertexBufferObjects(new CVIS::Slam6d(640, 480, 575.815735f, 6000));
//		drawer3D_->getVertexBufferObjects()->AllocatePointCloudVBO(76800, 3, 20);
//	}
}

void GTKGui::start()
{
  gtk_main();
}

} // namespace MVOG

