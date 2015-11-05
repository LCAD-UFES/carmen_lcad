#include <mvog_gtk_gui/gtk_gui.h>

namespace MVOG
{

GTKGui::GTKGui()
{
  options_.view3D = true;
  options_.drawPVolumes = false;
  options_.drawNVolumes = false;
  options_.drawRawData = false;
  options_.colorByHeight = true;


  mouseLeftIsDown_  = false;
  mouseMidIsDown_   = false;
  mouseRightIsDown_ = false;

  drawer3D_ = new MapDrawer3D(this);
  drawer2D_ = new MapDrawer2D(this);

  drawer3D_->setRobot(0,0,0,0,0,0);
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

    gtk_toggle_button_set_active(controls_.btnDrawRawData,    options_.drawRawData);
    gtk_toggle_button_set_active(controls_.btnDrawObstacles, !options_.drawRawData);

    if (options_.drawRawData)
    {
      gtk_widget_show(controls_.vboxRawOptions);
      gtk_widget_hide(controls_.vboxObstacleOptions);
    }
    else
    {
      gtk_widget_hide(controls_.vboxRawOptions);
      gtk_widget_show(controls_.vboxObstacleOptions);

      gtk_toggle_button_set_active(controls_.btnColorByHeight,    options_.colorByHeight);

      if (options_.colorByHeight)
        gtk_widget_show(controls_.vboxColorByHeightOptions);
      else
        gtk_widget_hide(controls_.vboxColorByHeightOptions);
    }
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

	gtk_spin_button_set_value(controls_.txtObstacleMaxHeight, drawer3D_->getMaxHeight());
	gtk_spin_button_set_value(controls_.txtObstacleCutoffAbove, drawer3D_->getCutoffAbove());
	gtk_spin_button_set_value(controls_.txtObstacleCutoffBelow, drawer3D_->getCutoffBelow());



}

void GTKGui::setDrawPVolumes(bool drawPVolumes)
{
  options_.drawPVolumes = drawPVolumes;
}

void GTKGui::setDrawNVolumes(bool drawNVolumes)
{
  options_.drawNVolumes = drawNVolumes;
}

void GTKGui::setDrawRobot(bool drawRobot)
{
  options_.drawRobot = drawRobot;
}

bool GTKGui::getDrawRobot()
{
  return options_.drawRobot;
}

void GTKGui::setDrawRawData(bool drawRawData)
{
  options_.drawRawData = drawRawData;
  drawer3D_->getMap()->validate();
}

void GTKGui::setColorByHeight(bool colorByHeight)
{
  options_.colorByHeight = colorByHeight;
}

bool GTKGui::getDrawPVolumes() const
{
  return options_.drawPVolumes;
}

bool GTKGui::getDrawNVolumes() const
{
  return options_.drawNVolumes;
}

bool GTKGui::getDrawRawData() const
{
  return options_.drawRawData;
}

bool GTKGui::getColorByHeight() const
{
  return options_.colorByHeight;
}

void GTKGui::mouseDown(double x, double y, int button)
{
  if      (button == 1) mouseLeftIsDown_  = true;
  else if (button == 2) mouseMidIsDown_   = true;
  else if (button == 3) mouseRightIsDown_ = true;
}

void GTKGui::mouseUp(double x, double y, int button)
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
      drawer3D_->move((mouseX_ - x) * MOVE_SPEED / canvasHeight_,
                      (mouseY_ - y) * MOVE_SPEED / canvasHeight_);
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
      drawer3D_->zoom(1.0 - (mouseY_ - y) * ZOOM_SPEED);
      updateControls();
    }
  }
  else
  {

  }

  mouseX_ = x;
  mouseY_ = y;
}

void GTKGui::setMap(MVOG::Map * map)
{
  drawer2D_->setMap(map);
  drawer3D_->setMap(map);
}


void GTKGui::setView()
{
  if (options_.view3D)
  	drawer3D_->setView();
  else
  	drawer2D_->setView();
}

void GTKGui::draw()
{
  if (options_.view3D)
  	drawer3D_->draw();
  else
  	drawer2D_->draw();
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

  //std::string packagePath = ros::package::getPath("mvog_gtk_gui");
  //std::string filePath = packagePath.append("/data/gui.glade");

  // Create new GtkBuilder object
  builder = gtk_builder_new();
  if( ! gtk_builder_add_from_file(builder, "../../carmen/data/gui/gui.glade", &error ) )
  {
    g_warning( "%s", error->message );
    g_free( error );
  }

  sleep(1.0);

  controls_.winMain  = GTK_WIDGET(gtk_builder_get_object(builder, "winMain" ));
  controls_.drawArea = GTK_WIDGET(gtk_builder_get_object(builder, "drawArea"));


  controls_.btnView2D = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnView2D"));
  controls_.btnView3D = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnView3D"));

  controls_.frameCamera         = GTK_WIDGET(gtk_builder_get_object(builder, "frameCamera"));
  controls_.frame3DMapOptions   = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DMapOptions"));
  controls_.frame3DOtherOptions = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DOtherOptions"));

  controls_.vboxRawOptions           = GTK_WIDGET(gtk_builder_get_object(builder, "vboxRawOptions" ));
  controls_.vboxObstacleOptions      = GTK_WIDGET(gtk_builder_get_object(builder, "vboxObstacleOptions" ));
  controls_.vboxColorByHeightOptions = GTK_WIDGET(gtk_builder_get_object(builder, "vboxColorByHeightOptions" ));

  controls_.btnDrawRawData   = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawRawData"));
  controls_.btnDrawObstacles = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawObstacles"));
  controls_.btnColorByHeight = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnColorByHeight"));

  controls_.ntbkViewOptions = GTK_NOTEBOOK(gtk_builder_get_object(builder, "ntbkViewOptions"));

  controls_.txtCamPosX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosX"));
  controls_.txtCamPosY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosY"));
  controls_.txtCamPosZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosZ"));

  controls_.txtCamLookX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookX"));
  controls_.txtCamLookY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookY"));
  controls_.txtCamLookZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookZ"));

  controls_.txtZPlane = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtZPlane"));
  controls_.txtVerticalScale  = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtVerticalScale"));
  controls_.txtVerticalOffset = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtVerticalOffset"));
  controls_.txtObstacleCutoff = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtObstacleCutoff"));

  controls_.txtObstacleMaxHeight = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtObstacleMaxHeight"));
  controls_.txtObstacleCutoffAbove = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtObstacleCutoffAbove"));
  controls_.txtObstacleCutoffBelow = GTK_SPIN_BUTTON(gtk_builder_get_object(builder, "txtObstacleCutoffBelow"));

  controls_.btnDrawPVolumes = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawPVolumes"));
  controls_.btnDrawNVolumes = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawNVolumes"));
  controls_.btnDrawIVolumes = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawIVolumes"));
  controls_.btnDrawRobot    = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawRobot"));
  controls_.btnDrawZPlane   = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawZPlane"));
  controls_.btnCutOffZPlane = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnCutOffZPlane"));

  controls_.btnHighlightUnknown = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnHighlightUnknown"));

  controls_.txtVisCenterX = GTK_ENTRY(gtk_builder_get_object(builder, "txtVisCenterX"));
  controls_.txtVisCenterY = GTK_ENTRY(gtk_builder_get_object(builder, "txtVisCenterY"));
  controls_.txtVisSize    = GTK_ENTRY(gtk_builder_get_object(builder, "txtVisSize"));

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

void GTKGui::start()
{
  gtk_main();
}

} // namespace MVOG

