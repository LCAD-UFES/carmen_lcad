#ifndef MVOG_GTK_GUI_GTK_GUI_H
#define MVOG_GTK_GUI_GTK_GUI_H

#define GL_GLEXT_PROTOTYPES

#include <string>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>

#include <drawer_3d.h>
#include <draw_callbacks.h>

#include <velodyne.h>
#include <kinect.h>
#include <carmen/joystick_interface.h>
//#include <slam6d.h>

namespace CVIS
{

class Drawer3D;

const double PAN_SPEED  = 0.01;
const double TILT_SPEED = 0.01;
const double ZOOM_SPEED = 0.006;
const double MOVE_SPEED = 0.25;

class GTKGui
{
  private:

	 struct Controls
	    {
	      GtkWidget * winMain;
		  GtkWidget * drawArea;

		  GtkToggleButton * btnView2D;
		  GtkToggleButton * btnView3D;

		  GtkToggleButton * btnEfeito1;
		  GtkToggleButton * btnEfeito2;
		  GtkToggleButton * btnEfeito3;
		  GtkToggleButton * btnEfeito4;
		  GtkToggleButton * btnEfeito5;
		  GtkToggleButton * btnEfeito10;

		  GtkToggleButton * btnAutoZoom;

	      GtkWidget * frameCamera;
	      GtkWidget * frame3DMapOptions;
	      GtkWidget * frame3DOtherOptions;

	      GtkWidget * vboxRawOptions;
	      GtkWidget * vboxObstacleOptions;
	      GtkWidget * vboxColorByHeightOptions;

		  GtkNotebook * ntbkViewOptions;

		  GtkEntry * txtCamPosX;
		  GtkEntry * txtCamPosY;
		  GtkEntry * txtCamPosZ;

		  GtkEntry * txtCamLookX;
		  GtkEntry * txtCamLookY;
		  GtkEntry * txtCamLookZ;

		  GtkEntry * tbxCloudR;
		  GtkEntry * tbxCloudG;
		  GtkEntry * tbxCloudB;

		  GtkEntry * tbxBackgroundR;
		  GtkEntry * tbxBackgroundG;
		  GtkEntry * tbxBackgroundB;

		  GtkEntry * tbxCor1R;
		  GtkEntry * tbxCor1G;
		  GtkEntry * tbxCor1B;

		  GtkEntry * tbxCor2R;
		  GtkEntry * tbxCor2G;
		  GtkEntry * tbxCor2B;

		  GtkEntry * tbxCor3R;
		  GtkEntry * tbxCor3G;
		  GtkEntry * tbxCor3B;

		  GtkSpinButton * spbChangeSpeed;
		  GtkSpinButton * spbForeRange;
		  GtkSpinButton * spbBackRange;

		  GtkSpinButton * spbVelZoom;
		  GtkSpinButton * spbVelTranslation;

		  GtkSpinButton * spbCloudDensity;
		  GtkSpinButton * spbPointSize;
	    };

    Controls controls_;

    // **** mouse events

    bool mouseLeftIsDown_;
    bool mouseMidIsDown_;
    bool mouseRightIsDown_;
    double mouseX_;
    double mouseY_;

    double joystickX_;
    double joystickY_;

    // **** draw window size

    double canvasWidth_;
    double canvasHeight_;

    Drawer3D * drawer3D_;

  public:

    typedef struct
	{
	  bool view3D;
	  bool drawColor;

	  bool efeito1enabled;
	  bool efeito2enabled;
	  bool efeito3enabled;
	  bool efeito4enabled;
	  bool efeito5enabled;
	  bool efeito10enabled;

	  double valueCloudR;
	  double valueCloudG;
	  double valueCloudB;

	  double backgroundR;
	  double backgroundG;
	  double backgroundB;

	  double changeColorSpeed;

	  double valueCor1R;
	  double valueCor1G;
	  double valueCor1B;

	  double valueCor2R;
	  double valueCor2G;
	  double valueCor2B;

	  double valueCor3R;
	  double valueCor3G;
	  double valueCor3B;

	  double valueForegroundRange;
	  double valueBackgroundRange;

	  double valueVelZoom;
	  double valueVelTranslation;

	  int valueCloudDensity;
	  int valuePointSize;

	  bool autoZoom;

	}Options;

    Options options_;
    // *** draw messages options

    bool drawVelodyneCloud_;
    bool drawKinectCloud_;
    bool drawSlam6DCloud_;
    bool drawStereoCloud_;

    GTKGui();
    virtual ~GTKGui();

    Controls * getControls() { return &controls_; }

    void start();

    void setView3D(bool view3D) { options_.view3D = view3D; }
    Drawer3D* getDrawer3D() { return drawer3D_; }

    void setCanvasWidth (double canvasWidth ) { canvasWidth_  = canvasWidth;  }
    void setCanvasHeight(double canvasHeight) { canvasHeight_ = canvasHeight; }

    double getCanvasWidth()   const { return canvasWidth_;  }
    double getCanvasHeight() const { return canvasHeight_; }

    void setView();
    void updateControls();

    void mouseDown(double x, double y, int button);
    void mouseUp  (double x, double y, int button);
    void mouseMove(double x, double y);

    void joystickMove(carmen_joystick_status_message* message);

    void draw();
    void setDrawColor(bool value) { options_.drawColor = value; }

    void setUpGTK();
    void setUpOpenGL(bool velodyneCloud, bool kinectCloud, bool slam6dCloud);
    void spinOnce() {  gtk_main_iteration(); }
};

void* startGTK(void *);

}

#endif

