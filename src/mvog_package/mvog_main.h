#ifndef MVOG_MAIN_H_
#define MVOG_MAIN_H_

#include <carmen/carmen_graphics.h>
#include <carmen/mvog_interface.h>
#include <carmen/fused_odometry_interface.h>

#include <mvog_model/mapper.h>
#include <mvog_gtk_gui/gtk_gui.h>

#include <carmen/stereo_util.h>

#include <carmen/visual_odometry_messages.h>

typedef struct
{
	carmen_laser_laser_message laser_message;
	carmen_fused_odometry_message fused_odometry;
}carmen_mvog_odometry_and_laser_t;

class MVOGServer
{

  public:
    MVOG::Mapper * mapper_;
    MVOG::GTKGui * gui_;

    double mvog_map_resolution;
    double mvog_init_map_size_x;
    double mvog_init_map_size_y;
    double mvog_bumblebee_sensor_hfov;
    double mvog_kinect_sensor_hfov;

    bool   mvog_model_negative_space;
    bool 	 mvog_use_bumblebee_sensor;
    bool 	 mvog_use_kinect_sensor;

    MVOGServer ();
    virtual ~MVOGServer();
    void MVOGInitialize();
    int read_parameters(int argc, char **argv);

    void spinOnce();
};

#endif /* MVOG_MAIN_H_ */
