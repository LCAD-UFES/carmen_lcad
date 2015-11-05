#include <gtk_gui.h>

namespace View
{

GtkGui::GtkGui()
{
	options_.drawCorrectedPath = false;
	options_.drawDistanceInformation = false;
	options_.drawParticles = false;
	options_.drawRobot = true;
	options_.drawVisualOdometryPath = true;
	options_.drawLandmarkPoints = true;

	options_.useStaticCamera = true;
	options_.useFollowCamera = false;
	options_.useDriverCamera = false;

	mouseLeftIsDown_  = false;
	mouseMidIsDown_   = false;
	mouseRightIsDown_ = false;

	drawer3D_ = new Drawer3D(this);
}

GtkGui::~GtkGui()
{

}

void GtkGui::updateControls()
{
	char s[10] = "";

	gtk_widget_show(controls_.frameCamera);
	gtk_widget_show(controls_.frame3DMapOptions);
	gtk_widget_show(controls_.frame3DOtherOptions);

	gtk_toggle_button_set_active(controls_.rdbStaticCamera,    options_.useStaticCamera);
	gtk_toggle_button_set_active(controls_.rdbFollowCamera,    options_.useFollowCamera);
	gtk_toggle_button_set_active(controls_.rdbDriverCamera,    options_.useDriverCamera);

	gtk_toggle_button_set_active(controls_.btnDrawRobot,    options_.drawRobot);
	gtk_toggle_button_set_active(controls_.btnDrawVisualOdometryPath,    options_.drawVisualOdometryPath);
	gtk_toggle_button_set_active(controls_.btnDrawCorrectedPath,    options_.drawCorrectedPath);
	gtk_toggle_button_set_active(controls_.btnDrawParticles,    options_.drawParticles);
	gtk_toggle_button_set_active(controls_.btnDrawLandmarkPoints,    options_.drawLandmarkPoints);
	gtk_toggle_button_set_active(controls_.btnDrawDistanceInformation,    options_.drawDistanceInformation);

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

}

void GtkGui::mouseDown(double x __attribute__((unused)), double y __attribute__((unused)), int button)
{
	if(options_.useStaticCamera)
	{
		if      (button == 1) mouseLeftIsDown_  = true;
		else if (button == 2) mouseMidIsDown_   = true;
		else if (button == 3) mouseRightIsDown_ = true;
	}
}

void GtkGui::mouseUp(double x __attribute__((unused)), double y __attribute__((unused)), int button)
{
	if(options_.useStaticCamera)
	{
		if      (button == 1) mouseLeftIsDown_  = false;
		else if (button == 2) mouseMidIsDown_   = false;
		else if (button == 3) mouseRightIsDown_ = false;
	}
}

void GtkGui::mouseMove(double x, double y)
{
	if(options_.useStaticCamera)
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

		mouseX_ = x;
		mouseY_ = y;
	}
}


void GtkGui::setView()
{
	drawer3D_->setView();
}

void GtkGui::draw()
{
	drawer3D_->draw();
}

void GtkGui::setRobotParameters(carmen_vector_3D_t robot_size)
{
	drawer3D_->setRobotParameters(robot_size);
}

void GtkGui::setRobotPose(carmen_pose_3D_t robot_pose)
{
	drawer3D_->setRobotPose(robot_pose);
}

void GtkGui::addVisualOdometryPose(carmen_vector_3D_t visual_odometry_pose)
{
	drawer3D_->addVisualOdometryPose(visual_odometry_pose);
}

void GtkGui::addLandmarkPoseToLandmarksList(carmen_vector_3D_t landmark_pose, unsigned char* crop)
{
	drawer3D_->addLandmarkPoseToLandmarksList(landmark_pose, crop);
}

void GtkGui::setUpGTK()
{
	GdkGLConfig *glconfig;
	GtkBuilder  *builder;
	GError      *error = NULL;

	gtk_init(0, NULL);
	gtk_gl_init(0, NULL);

	//Try double-buffered visual
	glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH | GDK_GL_MODE_DOUBLE));

	if (glconfig == NULL)
	{
		g_print ("*** Cannot find the double-buffered visual.\n");
		g_print ("*** Trying single-buffered visual.\n");

		// Try single-buffered visual
		glconfig = gdk_gl_config_new_by_mode (static_cast<GdkGLConfigMode> (GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH));

		if (glconfig == NULL)
			g_print ("*** No appropriate OpenGL-capable visual found.\n");
	}

	// Create new GtkBuilder object
	builder = gtk_builder_new();
	if( ! gtk_builder_add_from_file(builder, "../../carmen/data/gui/laslam.glade", &error ) )
	{
		g_warning( "%s", error->message );
		g_free( error );
	}

	controls_.winMain  = GTK_WIDGET(gtk_builder_get_object(builder, "winMain" ));
	controls_.drawArea = GTK_WIDGET(gtk_builder_get_object(builder, "drawArea"));

	controls_.btnDrawRobot = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawRobot"));
	controls_.btnDrawVisualOdometryPath = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawVisualOdometry"));
	controls_.btnDrawCorrectedPath = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawCorrectedPath"));
	controls_.btnDrawLandmarkPoints = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawLandmarks"));
	controls_.btnDrawDistanceInformation = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawDistanceInformation"));
	controls_.btnDrawParticles = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "btnDrawParticles"));

	controls_.rdbStaticCamera = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "rdbStaticCamera"));
	controls_.rdbFollowCamera = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "rdbFollowCamera"));
	controls_.rdbDriverCamera = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "rdbDriverCamera"));

	controls_.frameCamera         = GTK_WIDGET(gtk_builder_get_object(builder, "frameCamera"));
	controls_.frame3DMapOptions   = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DMapOptions"));
	controls_.frame3DOtherOptions = GTK_WIDGET(gtk_builder_get_object(builder, "frame3DOtherOptions"));

	controls_.txtCamPosX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosX"));
	controls_.txtCamPosY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosY"));
	controls_.txtCamPosZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamPosZ"));

	controls_.txtCamLookX = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookX"));
	controls_.txtCamLookY = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookY"));
	controls_.txtCamLookZ = GTK_ENTRY(gtk_builder_get_object(builder, "txtCamLookZ"));

	controls_.lblX = GTK_LABEL(gtk_builder_get_object(builder, "lblX"));
	controls_.lblY = GTK_LABEL(gtk_builder_get_object(builder, "lblY"));
	controls_.lblZ = GTK_LABEL(gtk_builder_get_object(builder, "lblZ"));
	controls_.lblYaw = GTK_LABEL(gtk_builder_get_object(builder, "lblYaw"));
	controls_.lblPitch = GTK_LABEL(gtk_builder_get_object(builder, "lblPitch"));
	controls_.lblRoll = GTK_LABEL(gtk_builder_get_object(builder, "lblRoll"));

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

void GtkGui::setUpOpenGL()
{
}

void GtkGui::start()
{
	gtk_main();
}

} // namespace MVOG

