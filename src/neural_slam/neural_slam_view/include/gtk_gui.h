#ifndef MVOG_GTK_GUI_GTK_GUI_H
#define MVOG_GTK_GUI_GTK_GUI_H

#define GL_GLEXT_PROTOTYPES

#include <string>

#include <carmen/carmen.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>

#include <drawer_3d.h>
#include <draw_callbacks.h>

namespace View
{

class Drawer3D;

const double PAN_SPEED  = 0.01;
const double TILT_SPEED = 0.01;
const double ZOOM_SPEED = 0.01;
const double MOVE_SPEED = 0.825;

class GtkGui
{
private:

	struct Controls
	{
		GtkWidget * winMain;
		GtkWidget * drawArea;

		GtkToggleButton * btnDrawRobot;
		GtkToggleButton * btnDrawVisualOdometryPath;
		GtkToggleButton * btnDrawCorrectedPath;
		GtkToggleButton * btnDrawParticles;
		GtkToggleButton * btnDrawSaliencyPoints;
		GtkToggleButton * btnDrawDistanceInformation;

		GtkToggleButton * rdbFollowCamera;
		GtkToggleButton * rdbStaticCamera;
		GtkToggleButton * rdbDriverCamera;

		GtkWidget * frameCamera;
		GtkWidget * frame3DMapOptions;
		GtkWidget * frame3DOtherOptions;

		GtkEntry * txtCamPosX;
		GtkEntry * txtCamPosY;
		GtkEntry * txtCamPosZ;

		GtkEntry * txtCamLookX;
		GtkEntry * txtCamLookY;
		GtkEntry * txtCamLookZ;

		GtkLabel * lblX;
		GtkLabel * lblY;
		GtkLabel * lblZ;
		GtkLabel * lblYaw;
		GtkLabel * lblPitch;
		GtkLabel * lblRoll;

	};

	struct Options
	{
		bool drawRobot;
		bool drawVisualOdometryPath;
		bool drawCorrectedPath;
		bool drawParticles;
		bool drawSaliencyPoints;
		bool drawDistanceInformation;

		bool useStaticCamera;
		bool useFollowCamera;
		bool useDriverCamera;
	};

	Controls controls_;
	Options options_;

	// **** mouse events

	bool mouseLeftIsDown_;
	bool mouseMidIsDown_;
	bool mouseRightIsDown_;
	double mouseX_;
	double mouseY_;

	// **** draw window size

	double canvasWidth_;
	double canvasHeight_;

	Drawer3D * drawer3D_;

public:

	GtkGui();
	virtual ~GtkGui();

	Controls * getControls() { return &controls_; }
	Options * getOptions() { return &options_; }

	void start();

	Drawer3D* getDrawer3D() { return drawer3D_; }

	void setRobotParameters(carmen_vector_3D_t robot_size);
	void setRobotPose(carmen_pose_3D_t robot_pose);
	void setLocalizeAckermanPose(carmen_pose_3D_t localize_pose);
	void setRobotCovarianceElipse(double angle, double major_axis, double minor_axis);
	void setNeuralPose(carmen_pose_3D_t neural_pose);
	void setCameraPose(carmen_pose_3D_t camera_pose);
	void setNeuralSaliency(carmen_vector_3D_t neural_saliency);
	void setNeuralCorrespondence(carmen_vector_3D_t neural_correspondence);

	void addOdometryPose(carmen_vector_3D_t odometry_pose);

	void setCanvasWidth (double canvasWidth ) { canvasWidth_  = canvasWidth;  }
	void setCanvasHeight(double canvasHeight) { canvasHeight_ = canvasHeight; }

	double getCanvasWidth()   const { return canvasWidth_;  }
	double getCanvasHeight() const { return canvasHeight_; }

	void setView();
	void updateControls();

	void mouseDown(double x, double y, int button);
	void mouseUp  (double x, double y, int button);
	void mouseMove(double x, double y);

	void draw();

	void setUpGTK();
	void setUpOpenGL();
	void spinOnce() {  gtk_main_iteration(); }
};

void* startGTK(void *);

}

#endif

