#ifndef MVOG_GTK_GUI_GTK_GUI_H
#define MVOG_GTK_GUI_GTK_GUI_H

#include <string>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <glade/glade.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <mvog_gtk_gui/map_drawer_2d.h>
#include <mvog_gtk_gui/map_drawer_3d.h>

#include <mvog_gtk_gui/draw_callbacks.h>

namespace MVOG
{

class MapDrawer3D;
class MapDrawer2D;

const double PAN_SPEED  = 0.01;
const double TILT_SPEED = 0.01;
const double ZOOM_SPEED = 0.01;
const double MOVE_SPEED = 0.825;

class GTKGui
{
  private:

    struct Controls
    {
      GtkWidget * winMain;
	    GtkWidget * drawArea;

	    GtkToggleButton * btnView2D;
	    GtkToggleButton * btnView3D;

      GtkWidget * frameCamera;
      GtkWidget * frame3DMapOptions;
      GtkWidget * frame3DOtherOptions;

      GtkToggleButton * btnDrawRawData;
      GtkToggleButton * btnDrawObstacles;

      GtkWidget * vboxRawOptions;
      GtkWidget * vboxObstacleOptions;
      GtkWidget * vboxColorByHeightOptions;

      GtkToggleButton * btnColorByHeight;

	    GtkNotebook * ntbkViewOptions;

	    GtkEntry * txtCamPosX;
	    GtkEntry * txtCamPosY;
	    GtkEntry * txtCamPosZ;

	    GtkEntry * txtCamLookX;
	    GtkEntry * txtCamLookY;
	    GtkEntry * txtCamLookZ;

	    GtkSpinButton * txtZPlane;
	    GtkSpinButton * txtVerticalScale;
	    GtkSpinButton * txtVerticalOffset;
	    GtkSpinButton * txtObstacleCutoff;

	    GtkSpinButton * txtObstacleMaxHeight;
	    GtkSpinButton * txtObstacleCutoffAbove;
	    GtkSpinButton * txtObstacleCutoffBelow;

	    GtkEntry * txtVisCenterX;
	    GtkEntry * txtVisCenterY;
	    GtkEntry * txtVisSize;

	    GtkToggleButton * btnDrawPVolumes;
	    GtkToggleButton * btnDrawNVolumes;
	    GtkToggleButton * btnDrawIVolumes;
	    GtkToggleButton * btnDrawRobot;
	    GtkToggleButton * btnDrawZPlane;
	    GtkToggleButton * btnCutOffZPlane;

	    GtkToggleButton * btnHighlightUnknown;
    };

    struct Options
    {
      bool view3D;

      bool drawRawData;
      bool drawPVolumes;
      bool drawNVolumes;

      bool drawRobot;
      bool colorByHeight;
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

    // **** OpenGL drawers

    MapDrawer2D * drawer2D_;
    MapDrawer3D * drawer3D_;

  public:

    GTKGui();
    virtual ~GTKGui();

    Controls * getControls() { return &controls_; }

    void setMap(MVOG::Map * map);

    void start();

    void setView3D(bool view3D) { options_.view3D = view3D; }
    MapDrawer3D* getDrawer3D() { return drawer3D_; }
    MapDrawer2D* getDrawer2D() { return drawer2D_; }

    void setDrawPVolumes(bool drawPVolumes);
    void setDrawNVolumes(bool drawNVolumes);
    void setDrawRawData (bool drawRawData);
    void setColorByHeight (bool colorByHeight);
    void setDrawRobot(bool drawRobot);
    bool getDrawRobot();

    bool getDrawPVolumes() const;
    bool getDrawNVolumes() const;
    bool getDrawRawData() const;
    bool getColorByHeight() const;

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
    void spinOnce() {  gtk_main_iteration(); }
};

void* startGTK(void *);

} // namespace MVOG

#endif //MVOG_GTK_GUI_GTK_GUI_H

