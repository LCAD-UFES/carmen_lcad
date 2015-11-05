#ifndef MVOG_GTK_GUI_MAP_DRAWER_3D_H
#define MVOG_GTK_GUI_MAP_DRAWER_3D_H

#include <GL/glut.h>

#include <mvog_model/map.h>

#include <mvog_gtk_gui/camera.h>
#include <mvog_gtk_gui/gtk_gui.h>

namespace MVOG
{

class GTKGui;

const double COLOR_RED[3]  = {1.00, 0.00, 0.00};
const double COLOR_GREEN[3]  = {0.00, 1.00, 0.00};
const double COLOR_BLUE[3]  = {0.00, 0.00, 1.00};

const double COLOR_P_VOLUMES[3]  = {0.75, 0.00, 0.00};
const double COLOR_N_VOLUMES[3]  = {0.00, 0.00, 0.75};
const double COLOR_ML_VOLUMES[3] = {0.00, 0.75, 0.00};
const double GRID_COLOR[3]      = {0.70, 0.70, 0.70};

const double RGB_PALLET_COLOR_VOLUMES[10][3] =
{
		{0.2, 0.4, 1.0},
		{0.2, 0.8, 1.0},
		{0.2, 1.0, 0.8},
		{0.2, 1.0, 0.4},
		{0.4, 1.0, 0.2},
		{0.8, 1.0, 0.2},
		{1.0, 0.8, 0.2},
		{1.0, 0.4, 0.2},
		{1.0, 0.2, 0.4},
		{1.0, 0.2, 0.8}
};

struct Robot
{
	double x, y, z;
	double yaw, pitch, roll;
};

class MapDrawer3D
{
	private:

    Map    * map_;
    GTKGui * gui_;
    Camera camera_;

    double maxHeight;
    double cutoffAbove;
    double cutoffBelow;


    Robot robot_;
    int robot_offset_x, robot_offset_y;

    void drawMLolumes(Robot robot);
	void drawPVolumes(Robot robot);
	void drawNVolumes(Robot robot);

    void drawAxes();
    void drawGrid(Robot robot);
    void drawRobot(Robot robot);

    void drawSolidColorVolume(double bottom, double top, const double color[3]);
    void drawHeightColorVolume(double bottom, double top);
    int offsetLimit(int offset);

	public:

		MapDrawer3D(GTKGui * gui);
		~MapDrawer3D();

    void draw();

    void setCutoffAbove(double p);
    double getCutoffAbove();

    void setCutoffBelow(double p);
    double getCutoffBelow();

    void setMaxHeight(double p);
    double getMaxHeight();

    double getCameraPosX() const { return camera_.getPosX(); }
    double getCameraPosY() const { return camera_.getPosY(); }
    double getCameraPosZ() const { return camera_.getPosZ(); }

    double getCameraLookX() const { return camera_.getLookX(); }
    double getCameraLookY() const { return camera_.getLookY(); }
    double getCameraLookZ() const { return camera_.getLookZ(); }

    void setCameraPosX(double x) { camera_.setPosX(x); }
    void setCameraPosY(double y) { camera_.setPosY(y); }
    void setCameraPosZ(double z) { camera_.setPosZ(z); }

    void setCameraLookX(double x) { camera_.setLookX(x); }
    void setCameraLookY(double y) { camera_.setLookY(y); }
    void setCameraLookZ(double z) { camera_.setLookZ(z); }

    void pan(double angle);
    void tilt(double angle);
    void zoom(double r);
    void move(double x, double y);

    void setView();
    void setMap(MVOG::Map * map) { map_ = map; }
    Map * getMap() {return map_;}

    void setRobot(double x, double y, double z, double yaw, double pitch, double roll);
    Robot getRobot();
};

} // namespace MVOG

#endif // MVOG_GTK_GUI_MAP_DRAWER_H
