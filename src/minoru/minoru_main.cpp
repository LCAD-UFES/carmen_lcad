#include <carmen/carmen.h>
#include <carmen/minoru_interface.h>
#include <string.h>
#include <opencv/cv.h>
#include <libcam.h>
#include <camcalib.h>

static int width = 320;
static int height = 240;
static int fps = 30;
static int is_rectified;
static int fake_bumblebee;

static carmen_minoru_stereoimage_message msg;

static void
convert_BGR_to_RGB_image(const char *BGR, unsigned char *RGB, int width, int height)
{
	int i;
	for(i = 0; i < (height * width); i++)
	{
		RGB[3 * i + 2] = BGR[3 * i];
		RGB[3 * i + 1] = BGR[3 * i + 1];
		RGB[3 * i] = BGR[3 * i + 2];
	}
}

static void
carmen_minoru_format_message(
		IplImage *left,
		IplImage *right,
		int is_rectified)
{
  int width = left->width;
  int height = left->height;
  int channels = left->nChannels;

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  msg.image_size = width*height*channels;
  msg.is_rectified = is_rectified;
  msg.height = height;
  msg.width = width;

  convert_BGR_to_RGB_image(left->imageData, msg.raw_left, width, height);
  convert_BGR_to_RGB_image(right->imageData, msg.raw_right, width, height);

}



/*********************************************************
		   --- Handlers ---
**********************************************************/

static void
carmen_minoru_shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("skeleton_module_filter: disconnected.\n");

    exit(0);
  }
}

static int
carmen_minoru_read_parameters(int argc, char **argv)
{
  int num_items;

  carmen_param_t param_list[] = {
		    {(char*)"minoru", (char*)"fps", CARMEN_PARAM_INT, &fps, 0, NULL},
		    {(char*)"minoru", (char*)"width", CARMEN_PARAM_INT, &width, 0, NULL},
		    {(char*)"minoru", (char*)"height", CARMEN_PARAM_INT, &height, 0, NULL},
		    {(char*)"minoru", (char*)"is_rectified", CARMEN_PARAM_ONOFF, &is_rectified, 0, NULL},
		    {(char*)"minoru", (char*)"fake_bumblebee", CARMEN_PARAM_ONOFF, &fake_bumblebee, 0, NULL},
    };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}

int 
main(int argc, char **argv) 
{
  if (argc != 3)
	carmen_die("%s: Wrong number of parameters. %s requires 2 parameters and received %d parameter(s). \nUsage:\n %s <left camera> <right camera>\n", argv[0], argv[0], argc-1, argv[0]);

  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  signal(SIGINT, carmen_minoru_shutdown_module);

  carmen_minoru_read_parameters(argc, argv);

  if(fake_bumblebee)
  	carmen_minoru_define_bumblebee_fake_messages();
  else
  	carmen_minoru_define_messages();

  IplImage *left_image=cvCreateImage(cvSize(width, height), 8, 3);
  unsigned char *left_data=(unsigned char *)left_image->imageData;

  IplImage *right_image=cvCreateImage(cvSize(width, height), 8, 3);
  unsigned char *right_data=(unsigned char *)right_image->imageData;

  msg.raw_left = (unsigned char *) malloc ((width * height * 3) * sizeof(unsigned char));
  msg.raw_right = (unsigned char *) malloc ((width * height * 3) * sizeof(unsigned char));

  std::string left_camera_device = argv[1];
  std::string right_camera_device = argv[2];

  StereoCamera *stereo_camera = new StereoCamera(left_camera_device.c_str(), right_camera_device.c_str(), width, height, fps);

  camcalib *camera_calibration = new camcalib();

  if (is_rectified)
  {
	  //camera_calibration->SetStereoCamera("minoru");
	  camera_calibration->ParseCalibrationFile("calibration.txt");
	  if (!camera_calibration->rectification_loaded)
		  carmen_die("%s: Error parsing calibration file.\n", argv[0]);
  }

  while(1)
  {

	  if (!stereo_camera->GrabFrames())
	  {
		  carmen_warn("Failed to acquire images\n");
		  continue;
	  }

	  stereo_camera->RetrieveLeftImage(left_image);
	  stereo_camera->RetrieveRightImage(right_image);

	  if (is_rectified)
	  {
		  #pragma omp parallel for
		  for (int cam = 0; cam <= 1; cam++)
		  {
			  if (cam == 0)
			  {
				  camera_calibration->RectifyImage(0, width, height, left_data, 0);
			  }
			  else
			  {
				  camera_calibration->RectifyImage(1, width, height, right_data, 0);
			  }
		  }
	  }

	  carmen_minoru_format_message(left_image, right_image, is_rectified);

	  if(fake_bumblebee)
	  	carmen_minoru_publish_bumblebee_fake_message(&msg);
	  else
	  	carmen_minoru_publish_message(&msg);
  }

  cvReleaseImage(&left_image);
  cvReleaseImage(&right_image);

  delete camera_calibration;
  delete stereo_camera;

  free(msg.raw_left);
  free(msg.raw_right);

  return (0);
}
