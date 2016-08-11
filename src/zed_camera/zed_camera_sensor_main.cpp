#include <carmen/carmen.h>
#include "zed_camera_sensor_interface.h"

//ZED Includes
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

//opencv includes
#include <opencv2/opencv.hpp>

static int zed_camera_sensor_getkey = 0;
sl::zed::Camera* zed;

carmen_zed_camera_sensor_stereoimage_message stereo_msg;
carmen_zed_camera_sensor_depthmap_message    depth_msg;

int zed_camera_sensor_quality = 0;
double zed_camera_sensor_fps = 0.0;

using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_zed_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int bytes_per_pixel)
{
    stereo_msg.host = carmen_get_host();
    stereo_msg.image_size = width*height*bytes_per_pixel;
    stereo_msg.width = width;
    stereo_msg.isRectified = 1;
    stereo_msg.height = height;
    stereo_msg.raw_left = rawLeft;
    stereo_msg.raw_right = rawRight;

    carmen_zed_camera_sensor_publish_stereoimage_message(&stereo_msg);
}

void
carmen_zed_publish_depthmap_message(unsigned char *rawImage, int width, int height, int bytes_per_pixel)
{
    depth_msg.host = carmen_get_host();
    depth_msg.image_size = width*height*bytes_per_pixel;
    depth_msg.width = width;
    depth_msg.height = height;
    depth_msg.raw_image = rawImage;

    carmen_zed_camera_sensor_publish_depthmap_message(&depth_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     delete zed;
     printf("zed_camera_sensor: disconnected.\n");
     exit(0);
  }
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
      {"zed_camera_sensor", (char*)"quality", CARMEN_PARAM_INT, &zed_camera_sensor_quality, 0, NULL},
      {"zed_camera_sensor", (char*)"fps", CARMEN_PARAM_DOUBLE, &zed_camera_sensor_fps, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int setupCamera() {
    zed = new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode>(zed_camera_sensor_quality), zed_camera_sensor_fps,0);
    sl::zed::InitParams params;
    sl::zed::ERRCODE err = zed->init(params);
    if (err != sl::zed::SUCCESS) {
        std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;
        delete zed;
        exit(0);
    }
    //STANDARD: Structure conservative, no occlusion filling. Application example : Obstacle detection, 3D reconstructions, 3D measures
    //FILL: Occlusion filling, edge sharpening, advanced post-filtering. Application example : Refocusing, Multi-view generation
    sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;

    // the depth is limited to 20. METERS as define in zed::init()
    // Set the maximum distance of depth/disparity estimation (all values after this limit will be reported as TOO_FAR value)
    zed->setDepthClampValue(5000);

    int ConfidenceIdx = 100;
    sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;

    //Jetson only. Execute the calling thread on core 2
    sl::zed::Camera::sticktoCPUCore(2);
}

int main(int argc, char **argv)
{

  if (argc != 2)
      carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <stereo|depth>\n", argv[0], argv[0], argc-1, argv[0]);

  bool isStereo = (!strcmp(argv[1],"stereo") ? true : false );

  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Read application specific parameters (Optional) */
  read_parameters(argc, argv);
  //carmen_param_allow_unfound_variables(0);

  /* Define published messages by your module */
  carmen_zed_camera_sensor_define_depthmap_messages();
  carmen_zed_camera_sensor_define_stereoimage_messages();

  /* Setting up the ZED Camera */
  setupCamera();

  int width = zed->getImageSize().width;
  int height = zed->getImageSize().height;

  if(isStereo) {
      while(1) {
          bool res = zed->grab(dm_type);
          if(!res) {
              zed->setConfidenceThreshold(ConfidenceIdx);

              carmen_zed_publish_stereoimage_message(zed->retrieveImage(static_cast<sl::zed::SIDE> (0)).data, zed->retrieveImage(static_cast<sl::zed::SIDE> (1)).data, width, height, 4);

              ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
              ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
          }
          usleep(1000/zed->getCurrentFPS());
      }
  }
  else {
      while(1) {
          bool res = zed->grab(dm_type);
          if(!res) {
              zed->setConfidenceThreshold(ConfidenceIdx);

              carmen_zed_publish_depthmap_message(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH).data, width, height, 4);

              ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
              ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
          }
      }
  }

}
