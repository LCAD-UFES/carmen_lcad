#ifndef CVIS_MAIN_H_
#define CVIS_MAIN_H_

#include <carmen/carmen_graphics.h>
#include <carmen/stereo_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/kinect_interface.h>
#include <carmen/joystick_interface.h>
#include <carmen/fused_odometry_interface.h>
//#include <carmen/slam6d_interface.h>

#include <gtk_gui.h>

class CVISServer
{

  public:
    CVIS::GTKGui * gui_;

    CVISServer ();
    virtual ~CVISServer();
    void CVISInitializeGUI();
    int read_parameters(int argc, char **argv);

    void spinOnce();
};

#endif
