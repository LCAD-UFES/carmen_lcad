#ifndef NEURAL_SLAM_MAIN_H_
#define NEURAL_SLAM_MAIN_H_

#include <carmen/carmen_graphics.h>
#include <gtk_gui.h>

#include <carmen/stereo_util.h>
#include <tf.h>

#include <opencv2/opencv.hpp>

class NeuralSlamServer
{

  public:
    View::GtkGui * gui_;

    NeuralSlamServer ();
    virtual ~NeuralSlamServer();

    void InitializeGui(carmen_vector_3D_t robot_size);

    void SpinOnce();
};

#endif
