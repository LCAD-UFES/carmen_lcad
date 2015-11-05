#ifndef DRAWER_3D_H
#define DRAWER_3D_H

#include <camera.h>
#include <vertex_buffer_objects.h>
#include <gtk_gui.h>

namespace CVIS
{

class GTKGui;

const double COLOR_RED[3]  = {1.00, 0.00, 0.00};
const double COLOR_GREEN[3]  = {0.00, 1.00, 0.00};
const double COLOR_BLUE[3]  = {0.00, 0.00, 1.00};

const double COLOR_P_VOLUMES[3]  = {0.75, 0.00, 0.00};
const double COLOR_N_VOLUMES[3]  = {0.00, 0.00, 0.75};
const double COLOR_ML_VOLUMES[3] = {0.00, 0.75, 0.00};
const double GRID_COLOR[3]      = {0.70, 0.70, 0.70};

class Drawer3D
{
	private:
    GTKGui * gui_;
    Camera camera_;
    VertexBufferObjects * vbos_;
    bool drawAxesEnabled;
    int pointSize;

    void drawAxes();

	public:

		Drawer3D(GTKGui * gui);
		~Drawer3D();

    void draw();
  	void drawPointCloud();

  	void setVertexBufferObjects(VertexBufferObjects* vbo) { this->vbos_ = vbo; }
  	VertexBufferObjects* getVertexBufferObjects(){  return vbos_; }

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

    void setDrawAxesEnabled(bool value) { this->drawAxesEnabled = value; }

    void pan(double angle);
    void tilt(double angle);
    void zoom(double r);
    void move(double x, double y);

    void setView();

    void SetPointSize(int value) { this->pointSize = value; }
};

}

#endif
