#ifndef CARMEN_LASLAM_SERVER_H
#define CARMEN_LASLAM_SERVER_H

#include <carmen/carmen_graphics.h>
#include <gtk_gui.h>

#include <carmen/visual_odometry_interface.h>
#include <carmen/stereo_util.h>
#include <tf.h>

#include <opencv/cv.h>

class LandmarkSlamServer
{

  public:
    View::GtkGui * gui_;

    LandmarkSlamServer ();
    virtual ~LandmarkSlamServer();

    void InitializeGui(carmen_vector_3D_t robot_size);

    void SpinOnce();
};

#endif
